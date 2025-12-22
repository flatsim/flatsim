#include "flatsim/simulator.hpp"
#include "flatsim/agent.hpp"
#include "flatsim/agent/loader.hpp"
#include "flatsim/tagged_zmq.hpp"
#include <chrono>
#include <cista/serialization.h>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <vector>

namespace simulator {

    static std::filesystem::path ipc_dir() {
        const char *env = std::getenv("FLATSIM_IPC_DIR");
        std::filesystem::path dir = env && *env ? std::filesystem::path(env) : std::filesystem::path("/tmp");
        if (dir.is_relative()) {
            dir = std::filesystem::absolute(dir);
        }
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        return dir;
    }

    static void remove_ipc_socket_file(const std::string &endpoint) {
        constexpr const char *prefix = "ipc://";
        if (!endpoint.starts_with(prefix)) {
            return;
        }
        std::filesystem::path p(endpoint.substr(std::char_traits<char>::length(prefix)));
        std::error_code ec;
        std::filesystem::remove(p, ec);
    }

    static std::string advertised_host_or_localhost(const std::string &address) {
        if (!address.empty() && address != "*" && address != "0.0.0.0") {
            return address;
        }
        return "127.0.0.1";
    }

    // ============================================================================
    // Initialization
    // ============================================================================

    void Simulator::init_rerun() {
        if (!rec_) {
            recording_id_ = "flatsim_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
            rec_ = std::make_shared<rerun::RecordingStream>(application_id_, recording_id_);
            (void)rec_->connect_grpc(rerun_grpc_addr_);
        }

        if (rec_) {
            rec_->log("", rerun::Clear::RECURSIVE);
            rec_->log_with_static("", true, rerun::Clear::RECURSIVE);
        }
    }

    // Constructor for LOCAL mode (no networking)
    Simulator::Simulator(float width, float height, concord::Datum datum, std::shared_ptr<rerun::RecordingStream> rec)
        : conn_(Conn::LOCAL), ctx_(1), sim_settings_(width, height, datum), rec_(rec) {

        init_rerun();

        world_ = std::make_unique<World>(rec_);
        world_->init(sim_settings_.datum, concord::Size(sim_settings_.width, sim_settings_.height, 0.0));

        // Initialize sensor data helper with physics world reference
        sensor_data_.set_world(world_->physics_ptr());
        sensor_data_.set_datum(sim_settings_.datum);

        std::cout << "[Simulator] LOCAL mode initialized (" << sim_settings_.width << "x" << sim_settings_.height << ")"
                  << std::endl;
    }

    // Constructor for IPC/TCP mode with settings struct
    Simulator::Simulator(Conn conn, const std::string &address, const SimulatorSettings &settings,
                         std::shared_ptr<rerun::RecordingStream> rec)
        : conn_(conn), ctx_(1), address_(address), sim_settings_(settings), rec_(rec) {

        if (conn_ == Conn::LOCAL) {
            throw std::runtime_error("Use the LOCAL mode constructor without address parameter");
        }

        init_rerun();

        // Setup ZMQ sockets
        spawn_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::rep);

        if (conn_ == Conn::IPC) {
            auto dir = ipc_dir();
            const std::string spawn_ep = "ipc://" + (dir / "flatsim_spawn").string();
            remove_ipc_socket_file(spawn_ep);
            spawn_socket_->bind(spawn_ep);
            std::cout << "[Simulator] Spawn socket: " << spawn_ep << std::endl;
        } else {
            spawn_socket_->bind("tcp://*:5555");
            std::cout << "[Simulator] Spawn socket: tcp://*:5555" << std::endl;
        }
        spawn_socket_->set(zmq::sockopt::rcvtimeo, 0);

        world_ = std::make_unique<World>(rec_);
        world_->init(sim_settings_.datum, concord::Size(sim_settings_.width, sim_settings_.height, 0.0));

        // Initialize sensor data helper with physics world reference
        sensor_data_.set_world(world_->physics_ptr());
        sensor_data_.set_datum(sim_settings_.datum);

        std::cout << "[Simulator] Initialized (" << sim_settings_.width << "x" << sim_settings_.height << ")"
                  << std::endl;
    }

    // Constructor for IPC/TCP mode with explicit parameters
    Simulator::Simulator(Conn conn, const std::string &address, float width, float height, concord::Datum datum,
                         std::shared_ptr<rerun::RecordingStream> rec)
        : Simulator(conn, address, SimulatorSettings{width, height, datum}, rec) {}

    Simulator::~Simulator() {
        local_agents_.clear();

        if (spawn_socket_) spawn_socket_->close();
        for (auto &[uuid, sock] : uplink_sockets_) {
            sock->close();
        }
        for (auto &[uuid, sock] : downlink_sockets_) {
            sock->close();
        }
        ctx_.close();
    }

    // ============================================================================
    // Machine Management
    // ============================================================================

    void Simulator::create_machine(const types::Machine &machine) {
        uint32_t group = machine.group > 0 ? machine.group : next_group_++;
        auto [it, inserted] = machines_.try_emplace(machine.uuid, rec_, world_->physics_ptr(), machine, group);
        if (inserted) {
            it->second.create();
        }
    }

    Machine *Simulator::get_machine(const std::string &uuid) {
        auto it = machines_.find(uuid);
        return it != machines_.end() ? &it->second : nullptr;
    }

    void Simulator::apply_control(const types::WheelControl &control, float dt) {
        auto it = machines_.find(control.uuid);
        if (it != machines_.end()) {
            it->second.apply_control(control, dt);
        }
    }

    void Simulator::teleport_machine(const std::string &uuid, const concord::Pose &pose) {
        auto it = machines_.find(uuid);
        if (it != machines_.end()) {
            it->second.teleport(pose);
        }
    }

    bool Simulator::destroy_machine(const std::string &uuid) {
        auto it = machines_.find(uuid);
        if (it == machines_.end()) {
            return false;
        }
        it->second.destroy();
        machines_.erase(it);
        return true;
    }

    types::ser::WorldState Simulator::get_world_state() const {
        types::ser::WorldState state;
        for (const auto &[uuid, machine] : machines_) {
            state.machines.push_back(machine.get_state());
        }
        return state;
    }

    // ============================================================================
    // Local Agent Management
    // ============================================================================

    agent::Agent &Simulator::spawn_agent(const std::filesystem::path &json_path, concord::Pose spawn_pose,
                                         std::optional<std::string> uuid, std::optional<pigment::RGB> color) {
        if (conn_ != Conn::LOCAL) {
            throw std::runtime_error("spawn_agent() only available in LOCAL mode");
        }

        auto machine_config = agent::Loader::load_from_json(json_path, spawn_pose, color);

        // Override UUID if provided
        if (uuid.has_value()) {
            machine_config.uuid = uuid.value();
        }

        std::cout << "[Simulator] Spawning: " << machine_config.name << " (" << machine_config.uuid << ")" << std::endl;

        create_machine(machine_config);

        auto agent_ptr = std::make_unique<agent::Agent>(machine_config, rec_);

        // Set teleport callback so Agent can call back to Simulator
        agent_ptr->set_teleport_callback(
            [this](const std::string &uuid, const concord::Pose &pose) { this->teleport_machine(uuid, pose); });

        local_agents_.push_back(std::move(agent_ptr));
        return *local_agents_.back();
    }

    agent::Agent *Simulator::get_agent(const std::string &uuid) {
        for (auto &agent : local_agents_) {
            if (agent->machine().uuid() == uuid) {
                return agent.get();
            }
        }
        return nullptr;
    }

    agent::Agent &Simulator::get_agent(size_t index) {
        if (index >= local_agents_.size()) {
            throw std::out_of_range("Agent index out of range: " + std::to_string(index));
        }
        return *local_agents_[index];
    }

    // ============================================================================
    // Transport Abstraction
    // ============================================================================

    void Simulator::send_state(const std::string &uuid, const types::ser::MachineState &state) {
        if (conn_ == Conn::LOCAL) {
            // Direct call to local agent
            auto *agent = get_agent(uuid);
            if (agent) {
                agent->update_from_physics(state);
            }
        } else {
            // Send via ZMQ downlink (tagged)
            auto it = downlink_sockets_.find(uuid);
            if (it != downlink_sockets_.end() && it->second) {
                try {
                    auto data = flatsim::wire::pack(flatsim::wire::Kind::STATE, state);
                    it->second->send(zmq::buffer(data), zmq::send_flags::dontwait);
                } catch (const zmq::error_t &) {
                }
            }
        }
    }

    std::optional<types::WheelControl> Simulator::recv_control(const std::string &uuid, int timeout_ms) {
        if (conn_ == Conn::LOCAL) {
            // Direct call to local agent
            auto *agent = get_agent(uuid);
            if (agent) {
                return agent->get_wheel_control();
            }
            return std::nullopt;
        } else {
            // Receive via ZMQ uplink (tagged)
            auto it = uplink_sockets_.find(uuid);
            if (it == uplink_sockets_.end() || !it->second) {
                return std::nullopt;
            }

            try {
                const auto start = std::chrono::steady_clock::now();
                while (true) {
                    const auto elapsed_ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start)
                            .count();
                    const int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
                    if (remaining_ms <= 0) {
                        return std::nullopt;
                    }

                    it->second->set(zmq::sockopt::rcvtimeo, remaining_ms);
                    zmq::message_t msg;
                    auto result = it->second->recv(msg, zmq::recv_flags::none);
                    if (!result) {
                        return std::nullopt;
                    }

                    std::vector<uint8_t> bytes(static_cast<uint8_t *>(msg.data()),
                                               static_cast<uint8_t *>(msg.data()) + msg.size());
                    const auto tagged = flatsim::wire::unpack(std::move(bytes));

                    switch (tagged.kind) {
                    case flatsim::wire::Kind::CONTROL: {
                        auto ctrl_ser = flatsim::wire::deserialize<types::ser::WheelControl>(tagged.payload);
                        last_heartbeat_[uuid] = std::chrono::steady_clock::now();
                        return ctrl_ser.to_control();
                    }
                    case flatsim::wire::Kind::HEARTBEAT: {
                        auto hb = flatsim::wire::deserialize<types::ser::Request>(tagged.payload);
                        if (hb.type == types::ser::MsgType::HEARTBEAT) {
                            last_heartbeat_[uuid] = std::chrono::steady_clock::now();
                        }
                        break;
                    }
                    case flatsim::wire::Kind::LIDAR_CFG: {
                        auto cfg_msg = flatsim::wire::deserialize<types::ser::LidarConfigMsg>(tagged.payload);
                        const std::string msg_uuid(cfg_msg.uuid.view());
                        set_lidar_config(msg_uuid.empty() ? uuid : msg_uuid, cfg_msg.to_config());
                        last_heartbeat_[uuid] = std::chrono::steady_clock::now();
                        break;
                    }
                    default:
                        break;
                    }
                }
            } catch (const zmq::error_t &) {
            }
            return std::nullopt;
        }
    }

    // ============================================================================
    // IPC/TCP Connection Management
    // ============================================================================

    void Simulator::process_spawn_requests() {
        zmq::message_t spawn_request;
        auto spawn_result = spawn_socket_->recv(spawn_request, zmq::recv_flags::dontwait);
        if (!spawn_result) return;

        std::vector<uint8_t> buffer(static_cast<uint8_t *>(spawn_request.data()),
                                    static_cast<uint8_t *>(spawn_request.data()) + spawn_request.size());

        auto *req = cista::deserialize<types::ser::Request>(buffer);
        types::ser::Response resp;

        if (!req) {
            resp.success = false;
        } else if (req->type == types::ser::MsgType::SPAWN) {
            auto machine = req->machine.to_machine();
            std::string uuid = machine.uuid;

            create_machine(machine);

            // Create dedicated sockets
            auto uplink_sock = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pull);
            auto downlink_sock = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pub);
            std::string uplink_ep, downlink_ep;

            if (conn_ == Conn::IPC) {
                auto dir = ipc_dir();
                uplink_ep = "ipc://" + (dir / ("flatsim_uplink_" + uuid)).string();
                downlink_ep = "ipc://" + (dir / ("flatsim_downlink_" + uuid)).string();
                remove_ipc_socket_file(uplink_ep);
                remove_ipc_socket_file(downlink_ep);
                uplink_sock->bind(uplink_ep);
                downlink_sock->bind(downlink_ep);
            } else {
                int base_port = next_tcp_port_;
                next_tcp_port_ += 10;
                uplink_sock->bind("tcp://*:" + std::to_string(base_port));
                downlink_sock->bind("tcp://*:" + std::to_string(base_port + 1));
                const auto host = advertised_host_or_localhost(address_);
                uplink_ep = "tcp://" + host + ":" + std::to_string(base_port);
                downlink_ep = "tcp://" + host + ":" + std::to_string(base_port + 1);
            }

            uplink_sock->set(zmq::sockopt::rcvtimeo, 0);
            uplink_sockets_[uuid] = std::move(uplink_sock);
            downlink_sockets_[uuid] = std::move(downlink_sock);
            last_heartbeat_[uuid] = std::chrono::steady_clock::now();

            resp.success = true;
            resp.state = get_world_state();
            resp.rerun.grpc_address = cista::offset::string(rerun_grpc_addr_);
            resp.rerun.recording_id = cista::offset::string(recording_id_);
            resp.rerun.application_id = cista::offset::string(application_id_);
            resp.zmq.uplink_endpoint = cista::offset::string(uplink_ep);
            resp.zmq.downlink_endpoint = cista::offset::string(downlink_ep);

            std::cout << "[Simulator] Spawned: " << uuid << std::endl;
        } else if (req->type == types::ser::MsgType::DESPAWN) {
            std::string uuid_str(req->uuid.view());
            if (uplink_sockets_.count(uuid_str)) {
                uplink_sockets_[uuid_str]->close();
                uplink_sockets_.erase(uuid_str);
            }
            if (downlink_sockets_.count(uuid_str)) {
                downlink_sockets_[uuid_str]->close();
                downlink_sockets_.erase(uuid_str);
            }
            resp.success = destroy_machine(uuid_str);
            last_heartbeat_.erase(uuid_str);
            std::cout << "[Simulator] Despawned: " << uuid_str << std::endl;
        } else {
            resp.success = false;
        }

        auto data = cista::serialize(resp);
        spawn_socket_->send(zmq::buffer(data), zmq::send_flags::none);
    }

    void Simulator::cleanup_stale_connections() {
        static int cleanup_tick = 0;
        if (++cleanup_tick % 60 != 0) return;

        auto now = std::chrono::steady_clock::now();
        std::vector<std::string> to_remove;

        for (const auto &[uuid, last_hb] : last_heartbeat_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_hb).count();
            if (elapsed > 5) {
                std::cout << "[Simulator] Heartbeat timeout: " << uuid << " (" << elapsed << "s)" << std::endl;
                to_remove.push_back(uuid);
            }
        }

        for (const auto &uuid : to_remove) {
            if (uplink_sockets_.count(uuid)) {
                uplink_sockets_[uuid]->close();
                uplink_sockets_.erase(uuid);
            }
            if (downlink_sockets_.count(uuid)) {
                downlink_sockets_[uuid]->close();
                downlink_sockets_.erase(uuid);
            }
            last_heartbeat_.erase(uuid);
            destroy_machine(uuid);
        }
    }

    // ============================================================================
    // Main Loop
    // ============================================================================

    void Simulator::tick(float dt) {
        static int tick_num = 0;
        tick_num++;
        const uint64_t tick_seq = static_cast<uint64_t>(tick_num);

        // Step 1: Send state to all agents
        for (auto &[uuid, machine] : machines_) {
            auto ms = machine.get_state();
            ms.tick_seq = tick_seq;
            send_state(uuid, ms);
        }

        // Step 2: Tick local agents (they compute controls)
        if (conn_ == Conn::LOCAL) {
            for (auto &agent : local_agents_) {
                agent->tick(dt);
            }
        }

        // Step 3: Receive controls and apply to physics
        for (auto &[uuid, machine] : machines_) {
            auto ctrl = recv_control(uuid, 50);
            if (ctrl) {
                machine.apply_control(*ctrl, dt);
            }
        }

        // Step 4: Physics step
        world_->tick(dt);

        // Step 5: Update machine poses and sensor data
        for (auto &[uuid, machine] : machines_) {
            machine.tick(dt);
            machine.update_sensors(sensor_data_, sim_settings_.datum, dt);
        }

        // Step 6: Send sensor state to agents
        for (auto &[uuid, machine] : machines_) {
            auto sensor_state = types::ser::SensorState::from_sensor_data(uuid, machine.get_sensor_data());
            sensor_state.tick_seq = tick_seq;
            send_sensor_state(uuid, sensor_state);
        }

        // Step 7: IPC/TCP only - connection management
        if (conn_ != Conn::LOCAL) {
            process_spawn_requests();
            cleanup_stale_connections();

            if (tick_num % 60 == 0) {
                std::cout << "[Simulator] Tick #" << tick_num << " - " << machines_.size() << " machines" << std::endl;
            }
        }
    }

    void Simulator::tock() {
        concord::Datum datum = world_->settings().get_datum();
        for (auto &[uuid, machine] : machines_) {
            machine.tock(datum);
        }
        world_->tock();

        for (auto &agent : local_agents_) {
            agent->tock();
        }
    }

    void Simulator::send_sensor_state(const std::string &uuid, const types::ser::SensorState &state) {
        if (conn_ == Conn::LOCAL) {
            // Direct call to local agent
            auto *agent = get_agent(uuid);
            if (agent) {
                agent->update_from_sensors(state);
            }
        } else {
            // Send via ZMQ downlink (tagged)
            auto it = downlink_sockets_.find(uuid);
            if (it != downlink_sockets_.end() && it->second) {
                try {
                    auto data = flatsim::wire::pack(flatsim::wire::Kind::SENSORS, state);
                    it->second->send(zmq::buffer(data), zmq::send_flags::dontwait);
                } catch (const zmq::error_t &) {
                }
            }
        }
    }

    void Simulator::set_lidar_config(const std::string &uuid, const types::LidarConfig &config) {
        auto it = machines_.find(uuid);
        if (it != machines_.end()) {
            it->second.config_mut().lidar = config;
        }
    }

} // namespace simulator
