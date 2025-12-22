#include "flatsim/simulator.hpp"
#include "flatsim/agent.hpp"
#include "flatsim/agent/loader.hpp"
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
        heartbeat_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pull);

        if (conn_ == Conn::IPC) {
            auto dir = ipc_dir();
            const std::string spawn_ep = "ipc://" + (dir / "flatsim_spawn").string();
            const std::string hb_ep = "ipc://" + (dir / "flatsim_heartbeat").string();
            remove_ipc_socket_file(spawn_ep);
            remove_ipc_socket_file(hb_ep);
            spawn_socket_->bind(spawn_ep);
            heartbeat_socket_->bind(hb_ep);
            std::cout << "[Simulator] Spawn socket: " << spawn_ep << std::endl;
            std::cout << "[Simulator] Heartbeat socket: " << hb_ep << std::endl;
        } else {
            spawn_socket_->bind("tcp://*:5555");
            heartbeat_socket_->bind("tcp://*:5556");
            std::cout << "[Simulator] Spawn socket: tcp://*:5555" << std::endl;
            std::cout << "[Simulator] Heartbeat socket: tcp://*:5556" << std::endl;
        }
        spawn_socket_->set(zmq::sockopt::rcvtimeo, 0);
        heartbeat_socket_->set(zmq::sockopt::rcvtimeo, 0);

        world_ = std::make_unique<World>(rec_);
        world_->init(sim_settings_.datum, concord::Size(sim_settings_.width, sim_settings_.height, 0.0));
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
        if (heartbeat_socket_) heartbeat_socket_->close();
        for (auto &[uuid, sock] : control_sockets_) {
            sock->close();
        }
        for (auto &[uuid, sock] : state_sockets_) {
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
                                         std::optional<pigment::RGB> color) {
        if (conn_ != Conn::LOCAL) {
            throw std::runtime_error("spawn_agent() only available in LOCAL mode");
        }

        auto machine_config = agent::Loader::load_from_json(json_path, spawn_pose, color);
        std::cout << "[Simulator] Spawning: " << machine_config.name << " (" << machine_config.uuid << ")" << std::endl;

        create_machine(machine_config);

        auto agent_ptr = std::make_unique<agent::Agent>(machine_config, rec_);
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
            // Send via ZMQ
            auto it = state_sockets_.find(uuid);
            if (it != state_sockets_.end() && it->second) {
                try {
                    auto data = cista::serialize(state);
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
            // Receive via ZMQ
            auto it = control_sockets_.find(uuid);
            if (it == control_sockets_.end() || !it->second) {
                return std::nullopt;
            }

            try {
                it->second->set(zmq::sockopt::rcvtimeo, timeout_ms);
                zmq::message_t msg;
                auto result = it->second->recv(msg, zmq::recv_flags::none);
                if (!result) {
                    return std::nullopt;
                }

                std::vector<uint8_t> buffer(static_cast<uint8_t *>(msg.data()),
                                            static_cast<uint8_t *>(msg.data()) + msg.size());
                auto *ctrl = cista::deserialize<types::ser::WheelControl>(buffer);
                if (ctrl) {
                    return ctrl->to_control();
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
            auto ctrl_sock = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pull);
            auto state_sock = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pub);
            std::string ctrl_ep, state_ep, hb_ep;

            if (conn_ == Conn::IPC) {
                auto dir = ipc_dir();
                ctrl_ep = "ipc://" + (dir / ("flatsim_ctrl_" + uuid)).string();
                state_ep = "ipc://" + (dir / ("flatsim_state_" + uuid)).string();
                hb_ep = "ipc://" + (dir / "flatsim_heartbeat").string();
                remove_ipc_socket_file(ctrl_ep);
                remove_ipc_socket_file(state_ep);
                ctrl_sock->bind(ctrl_ep);
                state_sock->bind(state_ep);
            } else {
                int base_port = next_tcp_port_;
                next_tcp_port_ += 10;
                ctrl_sock->bind("tcp://*:" + std::to_string(base_port));
                state_sock->bind("tcp://*:" + std::to_string(base_port + 1));
                const auto host = advertised_host_or_localhost(address_);
                ctrl_ep = "tcp://" + host + ":" + std::to_string(base_port);
                state_ep = "tcp://" + host + ":" + std::to_string(base_port + 1);
                hb_ep = "tcp://" + host + ":5556";
            }

            ctrl_sock->set(zmq::sockopt::rcvtimeo, 0);
            control_sockets_[uuid] = std::move(ctrl_sock);
            state_sockets_[uuid] = std::move(state_sock);
            last_heartbeat_[uuid] = std::chrono::steady_clock::now();

            resp.success = true;
            resp.state = get_world_state();
            resp.rerun.grpc_address = cista::offset::string(rerun_grpc_addr_);
            resp.rerun.recording_id = cista::offset::string(recording_id_);
            resp.rerun.application_id = cista::offset::string(application_id_);
            resp.zmq.control_endpoint = cista::offset::string(ctrl_ep);
            resp.zmq.state_endpoint = cista::offset::string(state_ep);
            resp.zmq.heartbeat_endpoint = cista::offset::string(hb_ep);

            std::cout << "[Simulator] Spawned: " << uuid << std::endl;
        } else if (req->type == types::ser::MsgType::DESPAWN) {
            std::string uuid_str(req->uuid.view());
            if (control_sockets_.count(uuid_str)) {
                control_sockets_[uuid_str]->close();
                control_sockets_.erase(uuid_str);
            }
            if (state_sockets_.count(uuid_str)) {
                state_sockets_[uuid_str]->close();
                state_sockets_.erase(uuid_str);
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

    void Simulator::process_heartbeats() {
        for (int i = 0; i < 100; ++i) {
            zmq::message_t hb_msg;
            auto hb_result = heartbeat_socket_->recv(hb_msg, zmq::recv_flags::dontwait);
            if (!hb_result) break;

            std::vector<uint8_t> buffer(static_cast<uint8_t *>(hb_msg.data()),
                                        static_cast<uint8_t *>(hb_msg.data()) + hb_msg.size());
            auto *hb_req = cista::deserialize<types::ser::Request>(buffer);
            if (hb_req && hb_req->type == types::ser::MsgType::HEARTBEAT) {
                std::string uuid_str(hb_req->uuid.view());
                last_heartbeat_[uuid_str] = std::chrono::steady_clock::now();
            }
        }
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
            if (control_sockets_.count(uuid)) {
                control_sockets_[uuid]->close();
                control_sockets_.erase(uuid);
            }
            if (state_sockets_.count(uuid)) {
                state_sockets_[uuid]->close();
                state_sockets_.erase(uuid);
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

        // Step 1: Send state to all agents
        for (auto &[uuid, machine] : machines_) {
            send_state(uuid, machine.get_state());
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

        // Step 5: Update machine poses
        for (auto &[uuid, machine] : machines_) {
            machine.tick(dt);
        }

        // Step 6: IPC/TCP only - connection management
        if (conn_ != Conn::LOCAL) {
            process_spawn_requests();
            process_heartbeats();
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

} // namespace simulator
