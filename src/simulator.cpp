#include "flatsim/simulator.hpp"
#include "flatsim/agent.hpp"
#include "flatsim/agent/loader/loader.hpp"
#include "flatsim/tagged_zmq.hpp"
#include "flatsim/transport.hpp"
#include <chrono>
#include <cstdlib>
#include <datapod/serialization/serialize.hpp>
#include <echo/echo.hpp>
#include <filesystem>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <rerun.hpp>
#include <rerun/blueprint/archetypes/eye_controls3d.hpp>
#include <rerun/blueprint/archetypes/map_background.hpp>
#include <rerun/blueprint/archetypes/view_blueprint.hpp>
#include <rerun/blueprint/components/map_provider.hpp>
#include <rerun/blueprint/components/view_class.hpp>
#include <rerun/blueprint/components/view_origin.hpp>
#include <rerun/components/entity_path.hpp>
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

        echo::trace("[Simulator] Recording stream connected to ", rerun_grpc_addr_).rgb(0, 255, 0).bold();

        // Create blueprint stream (separate from data stream)
        if (!blueprint_rec_) {
            blueprint_rec_ =
                std::make_shared<rerun::RecordingStream>(application_id_, recording_id_, rerun::StoreKind::Blueprint);
            (void)blueprint_rec_->connect_grpc(rerun_grpc_addr_);
        }

        auto map_bg = rerun::blueprint::archetypes::MapBackground{}.with_provider(
            rerun::blueprint::components::MapProvider::MapboxDark);
        if (rec_) {
            rec_->log("mapbox", std::move(map_bg));
            rec_->log("", rerun::Clear::RECURSIVE);
            rec_->log_with_static("", true, rerun::Clear::RECURSIVE);
        }
    }

    void Simulator::update_camera_tracking(const std::string &uuid) {
        if (!rec_ || uuid.empty()) {
            return;
        }
        last_joined_agent_uuid_ = uuid;
        std::string chassis_path = "/" + uuid + "/chassis";
        auto eye_controls = rerun::blueprint::archetypes::EyeControls3D().with_tracking_entity(
            rerun::components::EntityPath(chassis_path));

        rec_->log("eye_controls", std::move(eye_controls));
        echo::trace("[Simulator] Camera tracking set to chassis: ", chassis_path);
    }

    // Constructor for LOCAL mode (no networking)
    Simulator::Simulator(float width, float height, datapod::Geo datum, std::shared_ptr<rerun::RecordingStream> rec)
        : conn_(Conn::LOCAL), sim_settings_(width, height, datum), rec_(rec) {

        init_rerun();

        world_ = std::make_unique<World>(rec_);
        world_->init(sim_settings_.datum, datapod::Size(sim_settings_.width, sim_settings_.height, 0.0));

        // Initialize sensor data helper with physics world reference
        sensor_data_.set_world(world_->physics_ptr());
        sensor_data_.set_datum(sim_settings_.datum);
    }

    // Constructor for IPC/TCP/SHM mode with settings struct
    Simulator::Simulator(Conn conn, const std::string &address, const SimulatorSettings &settings,
                         std::shared_ptr<rerun::RecordingStream> rec)
        : conn_(conn), address_(address), sim_settings_(settings), rec_(rec) {

        if (conn_ == Conn::LOCAL) {
            throw std::runtime_error("Use the LOCAL mode constructor without address parameter");
        }

        init_rerun();

        // Setup netpipe RPC server for spawn/despawn
        spawn_server_ = std::make_unique<flatsim::RpcServer>();

        flatsim::Endpoint spawn_endpoint;
        if (conn_ == Conn::IPC) {
            auto dir = ipc_dir();
            const std::string spawn_path = (dir / "flatsim_spawn").string();
            spawn_endpoint = flatsim::Endpoint::ipc(spawn_path);
        } else if (conn_ == Conn::TCP) {
            spawn_endpoint = flatsim::Endpoint::tcp(address_.empty() ? "0.0.0.0" : address_, 5555);
        } else if (conn_ == Conn::SHM) {
            spawn_endpoint = flatsim::Endpoint::shm("flatsim_spawn", 1024 * 1024); // 1MB buffer
        }

        if (!spawn_server_->listen(spawn_endpoint)) {
            throw std::runtime_error("Failed to start spawn server on " + spawn_endpoint.to_string());
        }

        // Register RPC handlers for spawn/despawn
        spawn_server_->register_method(flatsim::RpcMethod::SPAWN, [this](const std::vector<uint8_t> &request) {
            return this->handle_spawn_request(request);
        });

        spawn_server_->register_method(flatsim::RpcMethod::DESPAWN, [this](const std::vector<uint8_t> &request) {
            return this->handle_despawn_request(request);
        });

        spawn_server_->register_method(flatsim::RpcMethod::HEARTBEAT, [this](const std::vector<uint8_t> &request) {
            // Heartbeat - just return success
            types::ser::Response resp;
            resp.success = true;
            auto data = datapod::serialize(resp);
            return std::vector<uint8_t>(data.begin(), data.end());
        });

        world_ = std::make_unique<World>(rec_);
        world_->init(sim_settings_.datum, datapod::Size(sim_settings_.width, sim_settings_.height, 0.0));

        // Initialize sensor data helper with physics world reference
        sensor_data_.set_world(world_->physics_ptr());
        sensor_data_.set_datum(sim_settings_.datum);
    }

    // Constructor for IPC/TCP mode with explicit parameters
    Simulator::Simulator(Conn conn, const std::string &address, float width, float height, datapod::Geo datum,
                         std::shared_ptr<rerun::RecordingStream> rec)
        : Simulator(conn, address, SimulatorSettings{width, height, datum}, rec) {}

    Simulator::~Simulator() {
        local_agents_.clear();

        if (spawn_server_) spawn_server_->close();
        for (auto &[uuid, sock] : uplink_tcp_) sock->close();
        for (auto &[uuid, sock] : downlink_tcp_) sock->close();
        for (auto &[uuid, sock] : uplink_ipc_) sock->close();
        for (auto &[uuid, sock] : downlink_ipc_) sock->close();
        for (auto &[uuid, sock] : uplink_shm_) sock->close();
        for (auto &[uuid, sock] : downlink_shm_) sock->close();
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

    void Simulator::teleport_machine(const std::string &uuid, const datapod::Pose &pose) {
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

    agent::Agent &Simulator::spawn_agent(const std::filesystem::path &json_path, datapod::Pose spawn_pose,
                                         std::optional<std::string> uuid, std::optional<pigment::RGB> color) {
        if (conn_ != Conn::LOCAL) {
            throw std::runtime_error("spawn_agent() only available in LOCAL mode");
        }

        auto machine_config = agent::Loader::load_from_json(json_path, spawn_pose, color);

        // Override UUID if provided
        if (uuid.has_value()) {
            machine_config.uuid = uuid.value();
        }

        create_machine(machine_config);

        auto agent_ptr = std::make_unique<agent::Agent>(machine_config, rec_);

        // Set teleport callback so Agent can call back to Simulator
        agent_ptr->set_teleport_callback(
            [this](const std::string &uuid, const datapod::Pose &pose) { this->teleport_machine(uuid, pose); });

        local_agents_.push_back(std::move(agent_ptr));

        // Update camera tracking to follow this newly spawned agent
        update_camera_tracking(machine_config.uuid);

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
            // Send via netpipe downlink (tagged)
            netpipe::Stream *stream = nullptr;
            if (conn_ == Conn::TCP) {
                auto it = downlink_tcp_.find(uuid);
                if (it != downlink_tcp_.end()) stream = it->second.get();
            } else if (conn_ == Conn::IPC) {
                auto it = downlink_ipc_.find(uuid);
                if (it != downlink_ipc_.end()) stream = it->second.get();
            } else if (conn_ == Conn::SHM) {
                auto it = downlink_shm_.find(uuid);
                if (it != downlink_shm_.end()) stream = it->second.get();
            }

            if (stream) {
                auto mutable_state = state; // datapod::serialize needs non-const
                auto data = flatsim::wire::pack(flatsim::wire::Kind::STATE, mutable_state);
                netpipe::Message msg(data.begin(), data.end());
                stream->send(msg); // Ignore errors
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
            // Receive via netpipe uplink (tagged)
            netpipe::Stream *stream = nullptr;
            if (conn_ == Conn::TCP) {
                auto it = uplink_tcp_.find(uuid);
                if (it != uplink_tcp_.end()) stream = it->second.get();
            } else if (conn_ == Conn::IPC) {
                auto it = uplink_ipc_.find(uuid);
                if (it != uplink_ipc_.end()) stream = it->second.get();
            } else if (conn_ == Conn::SHM) {
                auto it = uplink_shm_.find(uuid);
                if (it != uplink_shm_.end()) stream = it->second.get();
            }

            if (!stream) {
                return std::nullopt;
            }

            const auto start = std::chrono::steady_clock::now();
            while (true) {
                const auto elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start)
                        .count();
                const int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
                if (remaining_ms <= 0) {
                    return std::nullopt;
                }

                stream->set_recv_timeout(static_cast<uint32_t>(remaining_ms));
                auto res = stream->recv();
                if (res.is_err()) {
                    return std::nullopt;
                }

                std::vector<uint8_t> bytes(res.value().begin(), res.value().end());
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
            return std::nullopt;
        }
    }

    // ============================================================================
    // IPC/TCP/SHM Connection Management
    // ============================================================================

    std::vector<uint8_t> Simulator::handle_spawn_request(const std::vector<uint8_t> &request) {
        types::ser::Response resp;
        resp.success = false;

        try {
            auto req = datapod::deserialize<datapod::Mode::NONE, types::ser::Request>(request);

            if (req.type != types::ser::MsgType::SPAWN) {
                return std::vector<uint8_t>(); // Empty response for wrong type
            }

            auto machine = req.machine.to_machine();
            std::string uuid = machine.uuid;

            create_machine(machine);

            // Create dedicated streams for this agent
            std::string uplink_ep, downlink_ep;

            if (conn_ == Conn::TCP) {
                int base_port = next_tcp_port_;
                next_tcp_port_ += 10;

                auto uplink = std::make_unique<netpipe::TcpStream>();
                auto downlink = std::make_unique<netpipe::TcpStream>();

                netpipe::TcpEndpoint up_ep{dp::String("0.0.0.0"), static_cast<uint16_t>(base_port)};
                netpipe::TcpEndpoint down_ep{dp::String("0.0.0.0"), static_cast<uint16_t>(base_port + 1)};

                auto up_res = uplink->listen(up_ep);
                auto down_res = downlink->listen(down_ep);

                if (up_res.is_err() || down_res.is_err()) {
                    echo::error("[Simulator] Failed to create TCP streams for ", uuid);
                    destroy_machine(uuid);
                    auto data = datapod::serialize(resp);
                    return std::vector<uint8_t>(data.begin(), data.end());
                }

                const auto host = advertised_host_or_localhost(address_);
                uplink_ep = "tcp://" + host + ":" + std::to_string(base_port);
                downlink_ep = "tcp://" + host + ":" + std::to_string(base_port + 1);

                uplink_tcp_[uuid] = std::move(uplink);
                downlink_tcp_[uuid] = std::move(downlink);

            } else if (conn_ == Conn::IPC) {
                auto dir = ipc_dir();
                uplink_ep = "ipc://" + (dir / ("flatsim_uplink_" + uuid)).string();
                downlink_ep = "ipc://" + (dir / ("flatsim_downlink_" + uuid)).string();

                remove_ipc_socket_file(uplink_ep);
                remove_ipc_socket_file(downlink_ep);

                auto uplink = std::make_unique<netpipe::IpcStream>();
                auto downlink = std::make_unique<netpipe::IpcStream>();

                auto up_res = uplink->listen_ipc(netpipe::IpcEndpoint{dp::String(uplink_ep.substr(6).c_str())});
                auto down_res = downlink->listen_ipc(netpipe::IpcEndpoint{dp::String(downlink_ep.substr(6).c_str())});

                if (up_res.is_err() || down_res.is_err()) {
                    echo::error("[Simulator] Failed to create IPC streams for ", uuid);
                    destroy_machine(uuid);
                    auto data = datapod::serialize(resp);
                    return std::vector<uint8_t>(data.begin(), data.end());
                }

                uplink_ipc_[uuid] = std::move(uplink);
                downlink_ipc_[uuid] = std::move(downlink);

            } else if (conn_ == Conn::SHM) {
                uplink_ep = "shm://flatsim_uplink_" + uuid;
                downlink_ep = "shm://flatsim_downlink_" + uuid;

                auto uplink = std::make_unique<netpipe::ShmStream>();
                auto downlink = std::make_unique<netpipe::ShmStream>();

                auto up_res = uplink->listen_shm(
                    netpipe::ShmEndpoint{dp::String(("flatsim_uplink_" + uuid).c_str()), 1024 * 1024});
                auto down_res = downlink->listen_shm(
                    netpipe::ShmEndpoint{dp::String(("flatsim_downlink_" + uuid).c_str()), 1024 * 1024});

                if (up_res.is_err() || down_res.is_err()) {
                    echo::error("[Simulator] Failed to create SHM streams for ", uuid);
                    destroy_machine(uuid);
                    auto data = datapod::serialize(resp);
                    return std::vector<uint8_t>(data.begin(), data.end());
                }

                uplink_shm_[uuid] = std::move(uplink);
                downlink_shm_[uuid] = std::move(downlink);
            }

            last_heartbeat_[uuid] = std::chrono::steady_clock::now();
            update_camera_tracking(uuid);

            resp.success = true;
            resp.state = get_world_state();
            resp.rerun.grpc_address = datapod::String(rerun_grpc_addr_);
            resp.rerun.recording_id = datapod::String(recording_id_);
            resp.rerun.application_id = datapod::String(application_id_);
            resp.zmq.uplink_endpoint = datapod::String(uplink_ep);
            resp.zmq.downlink_endpoint = datapod::String(downlink_ep);

        } catch (const std::exception &e) {
            echo::error("[Simulator] Failed to handle spawn request: ", e.what());
        }

        auto data = datapod::serialize(resp);
        return std::vector<uint8_t>(data.begin(), data.end());
    }

    std::vector<uint8_t> Simulator::handle_despawn_request(const std::vector<uint8_t> &request) {
        types::ser::Response resp;
        resp.success = false;

        try {
            auto req = datapod::deserialize<datapod::Mode::NONE, types::ser::Request>(request);

            if (req.type != types::ser::MsgType::DESPAWN) {
                return std::vector<uint8_t>(); // Empty response for wrong type
            }

            std::string uuid_str(req.uuid.view());

            // Close and remove streams
            if (conn_ == Conn::TCP) {
                if (uplink_tcp_.count(uuid_str)) {
                    uplink_tcp_[uuid_str]->close();
                    uplink_tcp_.erase(uuid_str);
                }
                if (downlink_tcp_.count(uuid_str)) {
                    downlink_tcp_[uuid_str]->close();
                    downlink_tcp_.erase(uuid_str);
                }
            } else if (conn_ == Conn::IPC) {
                if (uplink_ipc_.count(uuid_str)) {
                    uplink_ipc_[uuid_str]->close();
                    uplink_ipc_.erase(uuid_str);
                }
                if (downlink_ipc_.count(uuid_str)) {
                    downlink_ipc_[uuid_str]->close();
                    downlink_ipc_.erase(uuid_str);
                }
            } else if (conn_ == Conn::SHM) {
                if (uplink_shm_.count(uuid_str)) {
                    uplink_shm_[uuid_str]->close();
                    uplink_shm_.erase(uuid_str);
                }
                if (downlink_shm_.count(uuid_str)) {
                    downlink_shm_[uuid_str]->close();
                    downlink_shm_.erase(uuid_str);
                }
            }

            resp.success = destroy_machine(uuid_str);
            last_heartbeat_.erase(uuid_str);

        } catch (const std::exception &e) {
            echo::error("[Simulator] Failed to handle despawn request: ", e.what());
        }

        auto data = datapod::serialize(resp);
        return std::vector<uint8_t>(data.begin(), data.end());
    }

    void Simulator::process_spawn_requests() {
        // Accept new connections from spawn server
        // This needs to be called periodically to accept new agent connections
        if (!spawn_server_) {
            return;
        }

        auto client_server = spawn_server_->accept();
        if (client_server) {
            // Spawn a thread to handle this client's RPC requests
            // For now, we'll just serve synchronously in the main thread
            // TODO: Move to background thread for production
            std::thread([server = std::move(client_server)]() mutable {
                server->serve(); // Blocks until client disconnects
            }).detach();
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
                to_remove.push_back(uuid);
            }
        }

        for (const auto &uuid : to_remove) {
            // Close streams based on connection type
            if (conn_ == Conn::TCP) {
                if (uplink_tcp_.count(uuid)) {
                    uplink_tcp_[uuid]->close();
                    uplink_tcp_.erase(uuid);
                }
                if (downlink_tcp_.count(uuid)) {
                    downlink_tcp_[uuid]->close();
                    downlink_tcp_.erase(uuid);
                }
            } else if (conn_ == Conn::IPC) {
                if (uplink_ipc_.count(uuid)) {
                    uplink_ipc_[uuid]->close();
                    uplink_ipc_.erase(uuid);
                }
                if (downlink_ipc_.count(uuid)) {
                    downlink_ipc_[uuid]->close();
                    downlink_ipc_.erase(uuid);
                }
            } else if (conn_ == Conn::SHM) {
                if (uplink_shm_.count(uuid)) {
                    uplink_shm_[uuid]->close();
                    uplink_shm_.erase(uuid);
                }
                if (downlink_shm_.count(uuid)) {
                    downlink_shm_[uuid]->close();
                    downlink_shm_.erase(uuid);
                }
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
            }
        }
    }

    void Simulator::tock() {
        datapod::Geo datum = world_->settings().get_datum();
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
            // Send via netpipe downlink (tagged)
            netpipe::Stream *stream = nullptr;
            if (conn_ == Conn::TCP) {
                auto it = downlink_tcp_.find(uuid);
                if (it != downlink_tcp_.end()) stream = it->second.get();
            } else if (conn_ == Conn::IPC) {
                auto it = downlink_ipc_.find(uuid);
                if (it != downlink_ipc_.end()) stream = it->second.get();
            } else if (conn_ == Conn::SHM) {
                auto it = downlink_shm_.find(uuid);
                if (it != downlink_shm_.end()) stream = it->second.get();
            }

            if (stream) {
                auto mutable_state = state; // datapod::serialize needs non-const
                auto data = flatsim::wire::pack(flatsim::wire::Kind::SENSORS, mutable_state);
                netpipe::Message msg(data.begin(), data.end());
                stream->send(msg); // Ignore errors
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
