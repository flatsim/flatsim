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

        // NEW: Setup listening RpcPeer for accepting agent connections
        listen_peer_ = std::make_unique<flatsim::RpcPeer>();

        flatsim::Endpoint listen_endpoint;
        if (conn_ == Conn::IPC) {
            auto dir = ipc_dir();
            const std::string peer_path = (dir / "flatsim_peer").string();
            listen_endpoint = flatsim::Endpoint::ipc(peer_path);
            remove_ipc_socket_file("ipc://" + peer_path);
        } else if (conn_ == Conn::TCP) {
            listen_endpoint = flatsim::Endpoint::tcp(address_.empty() ? "0.0.0.0" : address_, 5555);
        } else if (conn_ == Conn::SHM) {
            listen_endpoint = flatsim::Endpoint::shm("flatsim_peer", 1024 * 1024); // 1MB buffer
        }

        if (!listen_peer_->listen(listen_endpoint)) {
            throw std::runtime_error("Failed to start peer listener on " + listen_endpoint.to_string() + ": " +
                                     listen_peer_->last_error());
        }

        echo::info("[Simulator] Listening for agent connections on ", listen_endpoint.to_string());

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

        // NEW: Close all peer connections
        if (listen_peer_) listen_peer_->close();
        for (auto &[uuid, peer] : peers_) {
            if (peer) peer->close();
        }
        peers_.clear();
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

    agent::Agent &Simulator::spawn_agent(const std::filesystem::path &machine_path, datapod::Pose spawn_pose,
                                         std::optional<std::string> uuid, std::optional<pigment::RGB> color) {
        if (conn_ != Conn::LOCAL) {
            throw std::runtime_error("spawn_agent() only available in LOCAL mode");
        }

        auto machine_config = agent::Loader::load_from_urdf(machine_path, spawn_pose, color);

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
            // NEW: Send via RpcPeer
            auto it = peers_.find(uuid);
            if (it != peers_.end() && it->second) {
                auto data = datapod::serialize(state);
                it->second->call(flatsim::RpcMethod::STATE, data, 100); // 100ms timeout
            }
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
            // NEW: Send via RpcPeer
            auto it = peers_.find(uuid);
            if (it != peers_.end() && it->second) {
                auto data = datapod::serialize(state);
                it->second->call(flatsim::RpcMethod::SENSORS, data, 100); // 100ms timeout
            }
        }
    }

    // ============================================================================
    // Peer Handler Registration
    // ============================================================================

    void Simulator::register_peer_handlers(flatsim::RpcPeer *peer, const std::string &uuid) {
        if (!peer) return;

        // SPAWN handler
        peer->register_method(flatsim::RpcMethod::SPAWN, [this, uuid](const std::vector<uint8_t> &req) {
            try {
                auto request = datapod::deserialize<datapod::Mode::NONE, types::ser::Request>(req);
                if (request.type != types::ser::MsgType::SPAWN) {
                    return std::vector<uint8_t>{0}; // Failure
                }

                auto machine = request.machine.to_machine();
                create_machine(machine);
                update_camera_tracking(machine.uuid);

                types::ser::Response resp;
                resp.success = true;
                resp.state = get_world_state();
                resp.rerun.grpc_address = datapod::String(rerun_grpc_addr_);
                resp.rerun.recording_id = datapod::String(recording_id_);
                resp.rerun.application_id = datapod::String(application_id_);

                auto data = datapod::serialize(resp);
                return std::vector<uint8_t>(data.begin(), data.end());
            } catch (const std::exception &e) {
                echo::error("[Simulator] SPAWN handler error: ", e.what());
                return std::vector<uint8_t>{0};
            }
        });

        // DESPAWN handler
        peer->register_method(flatsim::RpcMethod::DESPAWN, [this, uuid](const std::vector<uint8_t> &req) {
            try {
                auto request = datapod::deserialize<datapod::Mode::NONE, types::ser::Request>(req);
                std::string despawn_uuid(request.uuid.view());

                bool success = destroy_machine(despawn_uuid);
                last_heartbeat_.erase(despawn_uuid);

                types::ser::Response resp;
                resp.success = success;
                auto data = datapod::serialize(resp);
                return std::vector<uint8_t>(data.begin(), data.end());
            } catch (const std::exception &e) {
                echo::error("[Simulator] DESPAWN handler error: ", e.what());
                return std::vector<uint8_t>{0};
            }
        });

        // CONTROL handler (receives control from agent)
        peer->register_method(flatsim::RpcMethod::CONTROL, [this, uuid](const std::vector<uint8_t> &req) {
            try {
                auto ctrl = datapod::deserialize<datapod::Mode::NONE, types::ser::WheelControl>(req);
                auto wheel_ctrl = ctrl.to_control();

                auto it = machines_.find(uuid);
                if (it != machines_.end()) {
                    it->second.apply_control(wheel_ctrl, 0.016f); // Assume 60Hz
                }

                last_heartbeat_[uuid] = std::chrono::steady_clock::now();
                return std::vector<uint8_t>{1}; // ACK
            } catch (const std::exception &e) {
                echo::error("[Simulator] CONTROL handler error: ", e.what());
                return std::vector<uint8_t>{0};
            }
        });

        // HEARTBEAT handler
        peer->register_method(flatsim::RpcMethod::HEARTBEAT, [this, uuid](const std::vector<uint8_t> &req) {
            last_heartbeat_[uuid] = std::chrono::steady_clock::now();
            return std::vector<uint8_t>{1}; // ACK
        });

        // LIDAR_CFG handler
        peer->register_method(flatsim::RpcMethod::LIDAR_CFG, [this, uuid](const std::vector<uint8_t> &req) {
            try {
                auto cfg_msg = datapod::deserialize<datapod::Mode::NONE, types::ser::LidarConfigMsg>(req);
                const std::string msg_uuid(cfg_msg.uuid.view());
                set_lidar_config(msg_uuid.empty() ? uuid : msg_uuid, cfg_msg.to_config());
                last_heartbeat_[uuid] = std::chrono::steady_clock::now();
                return std::vector<uint8_t>{1}; // ACK
            } catch (const std::exception &e) {
                echo::error("[Simulator] LIDAR_CFG handler error: ", e.what());
                return std::vector<uint8_t>{0};
            }
        });
    }

    // ============================================================================
    // IPC/TCP/SHM Connection Management
    // ============================================================================

    void Simulator::process_spawn_requests() {
        // Accept new peer connections
        if (!listen_peer_) {
            return;
        }

        auto peer = listen_peer_->accept(100, true); // max 100 concurrent, metrics enabled
        if (peer) {
            echo::info("[Simulator] Accepted new peer connection");

            // Wait for SPAWN request to get UUID
            // For now, generate a temporary UUID and register handlers
            // The SPAWN handler will create the actual machine
            std::string temp_uuid =
                "pending_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

            register_peer_handlers(peer.get(), temp_uuid);

            // Store peer (will be updated with real UUID after SPAWN)
            peers_[temp_uuid] = std::move(peer);

            echo::info("[Simulator] Peer registered with temp UUID: ", temp_uuid);
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
            // Close peer
            auto it = peers_.find(uuid);
            if (it != peers_.end()) {
                if (it->second) it->second->close();
                peers_.erase(it);
            }

            last_heartbeat_.erase(uuid);
            destroy_machine(uuid);

            echo::warn("[Simulator] Removed stale connection: ", uuid);
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

        // Step 3: For LOCAL mode, get controls from agents
        // For networked mode, controls are received via peer handlers (asynchronous)
        if (conn_ == Conn::LOCAL) {
            for (auto &agent : local_agents_) {
                auto ctrl = agent->get_wheel_control();
                auto it = machines_.find(ctrl.uuid);
                if (it != machines_.end()) {
                    it->second.apply_control(ctrl, dt);
                }
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

    void Simulator::set_lidar_config(const std::string &uuid, const types::LidarConfig &config) {
        auto it = machines_.find(uuid);
        if (it != machines_.end()) {
            it->second.config_mut().lidar = config;
        }
    }

} // namespace simulator
