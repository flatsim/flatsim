#include "flatsim/simulator.hpp"
#include "flatsim/agent.hpp"
#include "flatsim/simulator/machine.hpp"
#include "flatsim/tagged_zmq.hpp"
#include "flatsim/transport.hpp"
#include <chrono>
#include <cstdlib>
#include <cstring>
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

    // agent47 protocol method IDs (must match agent47::PipeBridge)
    static constexpr dp::u32 AGENT47_METHOD_COMMAND = 1;
    static constexpr dp::u32 AGENT47_METHOD_FEEDBACK = 2;
    static constexpr dp::u32 AGENT47_METHOD_SENSOR = 3;
    static constexpr dp::u32 AGENT47_METHOD_HEARTBEAT = 4;
    static constexpr dp::u32 AGENT47_METHOD_MODEL = 5;

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

        // agent47 PipeBridge transport: listen on a netpipe::Pipe
        netpipe::AnyEndpoint ep;
        if (conn_ == Conn::IPC) {
            auto dir = ipc_dir();
            const std::string sock_path = (dir / "agent47_peer.sock").string();
            remove_ipc_socket_file("ipc://" + sock_path);
            ep = netpipe::AnyEndpoint::ipc_endpoint(dp::String(sock_path.c_str()));
            echo::info("[Simulator] agent47 listening on ipc://", sock_path);
        } else if (conn_ == Conn::TCP) {
            const std::string host = address_.empty() ? "0.0.0.0" : address_;
            ep = netpipe::AnyEndpoint::tcp_endpoint(dp::String(host.c_str()), 5556);
            echo::info("[Simulator] agent47 listening on tcp://", host, ":5556");
        } else if (conn_ == Conn::SHM) {
            ep = netpipe::AnyEndpoint::shm_endpoint(dp::String("agent47_peer"), 1024 * 1024);
            echo::info("[Simulator] agent47 listening on shm://agent47_peer:1048576");
        }

        auto listen_res = netpipe::Pipe::listen(ep);
        if (listen_res.is_err()) {
            throw std::runtime_error("Failed to start agent47 listener");
        }
        agent47_listen_pipe_.emplace(std::move(listen_res.value()));

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

        // Close agent47 peers
        if (agent47_listen_pipe_.has_value()) {
            agent47_listen_pipe_->close();
            agent47_listen_pipe_.reset();
        }
        for (auto &[uuid, peer] : agent47_peers_) {
            (void)uuid;
            if (peer.rpc) peer.rpc.reset();
            if (peer.pipe.has_value()) {
                peer.pipe->close();
                peer.pipe.reset();
            }
        }
        agent47_peers_.clear();
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

        auto model = agent::Agent::load_model_from_urdf(machine_path);

        validate_model_for_flatsim(model);

        // TEMP: the simulation stack is still `types::Machine` based. Convert from dp model here.
        auto machine_config = machine_from_model(model, spawn_pose, color);

        // Override UUID if provided
        if (uuid.has_value()) {
            machine_config.uuid = uuid.value();
        }

        create_machine(machine_config);

        auto agent_ptr = std::make_unique<agent::Agent>(machine_config, rec_);
        local_agents_.push_back(std::move(agent_ptr));

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
            auto it = agent47_peers_.find(uuid);
            if (it == agent47_peers_.end() || !it->second.rpc) {
                return;
            }

            // Convert flatsim state -> agent47 feedback and push to agent via RPC method 2.
            dp::Stamp<agent47::types::Feedback> fb;
            fb.timestamp = dp::Stamp<agent47::types::Feedback>::now();
            fb.value.pose.point.x = state.pose.position.x;
            fb.value.pose.point.y = state.pose.position.y;
            fb.value.pose.point.z = 0.0;
            // Minimal rotation: yaw only (agent47 expects quaternion). We keep identity for now.
            // The agent47 consumer currently mostly uses pose.point and twist.
            fb.value.pose.rotation.w = 1.0;
            fb.value.pose.rotation.x = 0.0;
            fb.value.pose.rotation.y = 0.0;
            fb.value.pose.rotation.z = 0.0;
            fb.value.twist.linear.vx = state.velocity.x;
            fb.value.twist.linear.vy = state.velocity.y;
            fb.value.twist.linear.vz = 0.0;
            fb.value.twist.angular.vx = 0.0;
            fb.value.twist.angular.vy = 0.0;
            fb.value.twist.angular.vz = state.angular_vel;

            auto msg = serialize_agent47_feedback(fb);
            (void)it->second.rpc->call(AGENT47_METHOD_FEEDBACK, msg, 100);
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
            auto it = agent47_peers_.find(uuid);
            if (it == agent47_peers_.end() || !it->second.rpc) {
                return;
            }

            if (state.has_lidar) {
                agent47::types::SensorPacket pkt;
                pkt.timestamp = static_cast<dp::i64>(dp::Stamp<agent47::types::Feedback>::now());
                pkt.kind = agent47::types::SensorKind::Lidar;
                auto lidar = state.lidar;
                pkt.payload = dp::serialize<dp::Mode::WITH_VERSION, types::ser::LidarData>(lidar);
                auto msg = serialize_agent47_sensor(pkt);
                (void)it->second.rpc->call(AGENT47_METHOD_SENSOR, msg, 100);
            }

            if (state.has_gps) {
                agent47::types::SensorPacket pkt;
                pkt.timestamp = static_cast<dp::i64>(dp::Stamp<agent47::types::Feedback>::now());
                pkt.kind = agent47::types::SensorKind::Gnss;
                auto gps = state.gps;
                pkt.payload = dp::serialize<dp::Mode::WITH_VERSION, types::ser::GpsData>(gps);
                auto msg = serialize_agent47_sensor(pkt);
                (void)it->second.rpc->call(AGENT47_METHOD_SENSOR, msg, 100);
            }

            if (state.has_imu) {
                agent47::types::SensorPacket pkt;
                pkt.timestamp = static_cast<dp::i64>(dp::Stamp<agent47::types::Feedback>::now());
                pkt.kind = agent47::types::SensorKind::Imu;
                auto imu = state.imu;
                pkt.payload = dp::serialize<dp::Mode::WITH_VERSION, types::ser::ImuData>(imu);
                auto msg = serialize_agent47_sensor(pkt);
                (void)it->second.rpc->call(AGENT47_METHOD_SENSOR, msg, 100);
            }
        }
    }

    // ============================================================================
    // Peer Handler Registration
    // ============================================================================

    static void write_u8(netpipe::Message &buf, dp::u8 v) { buf.push_back(v); }

    template <typename T> static void write_val(netpipe::Message &buf, const T &val) {
        const dp::u8 *p = reinterpret_cast<const dp::u8 *>(&val);
        for (dp::usize i = 0; i < sizeof(T); ++i) {
            buf.push_back(p[i]);
        }
    }

    netpipe::Message Simulator::serialize_agent47_feedback(const dp::Stamp<agent47::types::Feedback> &fb) {
        netpipe::Message msg;
        write_val(msg, static_cast<dp::i64>(fb.timestamp));

        write_val(msg, fb.value.pose.point.x);
        write_val(msg, fb.value.pose.point.y);
        write_val(msg, fb.value.pose.point.z);
        write_val(msg, fb.value.pose.rotation.w);
        write_val(msg, fb.value.pose.rotation.x);
        write_val(msg, fb.value.pose.rotation.y);
        write_val(msg, fb.value.pose.rotation.z);

        write_val(msg, fb.value.twist.linear.vx);
        write_val(msg, fb.value.twist.linear.vy);
        write_val(msg, fb.value.twist.linear.vz);
        write_val(msg, fb.value.twist.angular.vx);
        write_val(msg, fb.value.twist.angular.vy);
        write_val(msg, fb.value.twist.angular.vz);

        write_val(msg, static_cast<dp::u32>(fb.value.wheels.size()));
        for (const auto &w : fb.value.wheels) {
            write_val(msg, w.angle_rad);
            write_val(msg, w.speed_rps);
        }

        // flags byte (lidar/gnss/imu). Keep 0 for now.
        write_u8(msg, 0);
        return msg;
    }

    netpipe::Message Simulator::serialize_agent47_sensor(const agent47::types::SensorPacket &pkt) {
        auto tmp = pkt;
        auto buf = dp::serialize<dp::Mode::WITH_VERSION, agent47::types::SensorPacket>(tmp);
        return netpipe::Message(buf.begin(), buf.end());
    }

    void Simulator::register_agent47_handlers(Agent47Peer &peer, const std::string &uuid) {
        if (!peer.rpc) {
            return;
        }

        peer.rpc->register_method(AGENT47_METHOD_COMMAND,
                                  [this, uuid](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                                      // Decode in the same layout as agent47::PipeBridge::serialize_command.
                                      // [timestamp_ns:8][lin vx/vy/vz:24][ang vx/vy/vz:24][valid:1]
                                      if (req.size() < (8 + 24 + 24 + 1)) {
                                          return dp::result::ok(netpipe::Message{});
                                      }

                                      const dp::u8 *ptr = req.data();
                                      auto read_i64 = [&ptr]() {
                                          dp::i64 v;
                                          std::memcpy(&v, ptr, sizeof(v));
                                          ptr += sizeof(v);
                                          return v;
                                      };
                                      auto read_f64 = [&ptr]() {
                                          dp::f64 v;
                                          std::memcpy(&v, ptr, sizeof(v));
                                          ptr += sizeof(v);
                                          return v;
                                      };
                                      auto read_u8 = [&ptr]() {
                                          dp::u8 v;
                                          std::memcpy(&v, ptr, sizeof(v));
                                          ptr += sizeof(v);
                                          return v;
                                      };

                                      dp::Stamp<agent47::types::Command> cmd;
                                      cmd.timestamp = read_i64();
                                      cmd.value.twist.linear.vx = read_f64();
                                      cmd.value.twist.linear.vy = read_f64();
                                      cmd.value.twist.linear.vz = read_f64();
                                      cmd.value.twist.angular.vx = read_f64();
                                      cmd.value.twist.angular.vy = read_f64();
                                      cmd.value.twist.angular.vz = read_f64();
                                      cmd.value.valid = (read_u8() != 0);

                                      auto it = agent47_peers_.find(uuid);
                                      if (it != agent47_peers_.end()) {
                                          std::lock_guard<std::mutex> lock(it->second.cmd_mutex);
                                          it->second.last_cmd = cmd;
                                          it->second.has_cmd = true;
                                      }

                                      last_heartbeat_[uuid] = std::chrono::steady_clock::now();
                                      return dp::result::ok(netpipe::Message{});
                                  });

        peer.rpc->register_method(AGENT47_METHOD_HEARTBEAT,
                                  [this, uuid](const netpipe::Message &) -> dp::Res<netpipe::Message> {
                                      last_heartbeat_[uuid] = std::chrono::steady_clock::now();
                                      return dp::result::ok(netpipe::Message{});
                                  });

        peer.rpc->register_method(
            AGENT47_METHOD_MODEL, [this, uuid](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                if (req.empty()) {
                    return dp::result::ok(netpipe::Message{});
                }

                dp::ByteBuf buf(req.begin(), req.end());
                dp::robot::Robot robot;
                try {
                    robot = dp::deserialize<dp::Mode::WITH_VERSION, dp::robot::Robot>(buf);
                } catch (...) {
                    return dp::result::ok(netpipe::Message{});
                }

                const std::string new_uuid(robot.id.uuid.c_str());
                if (new_uuid.empty()) {
                    return dp::result::ok(netpipe::Message{});
                }

                // Convert dp model -> legacy machine config and create in world.
                try {
                    validate_model_for_flatsim(robot.model);
                    auto machine_config = machine_from_model(robot.model, datapod::Pose{}, std::nullopt);
                    machine_config.uuid = new_uuid;
                    machine_config.name = std::string(robot.id.name.c_str());
                    create_machine(machine_config);
                    update_camera_tracking(machine_config.uuid);
                } catch (const std::exception &e) {
                    echo::error("[Simulator] model handler error: ", e.what());
                    return dp::result::ok(netpipe::Message{});
                }

                bind_agent47_peer_to_uuid(uuid, new_uuid);
                last_heartbeat_[new_uuid] = std::chrono::steady_clock::now();

                return dp::result::ok(netpipe::Message{});
            });
    }

    void Simulator::bind_agent47_peer_to_uuid(const std::string &old_uuid, const std::string &new_uuid) {
        if (old_uuid == new_uuid) {
            return;
        }
        auto it = agent47_peers_.find(old_uuid);
        if (it == agent47_peers_.end()) {
            return;
        }
        // Move peer entry to the new uuid key.
        Agent47Peer moved = std::move(it->second);
        agent47_peers_.erase(it);
        agent47_peers_.try_emplace(new_uuid, std::move(moved));

        // Move heartbeat entry too.
        auto hb = last_heartbeat_.find(old_uuid);
        if (hb != last_heartbeat_.end()) {
            last_heartbeat_[new_uuid] = hb->second;
            last_heartbeat_.erase(hb);
        }
    }

    void Simulator::apply_agent47_command(const std::string &uuid, const agent47::types::Command &cmd, float dt) {
        (void)dt;
        if (!cmd.valid) {
            return;
        }

        // Minimal mapping: interpret vx as desired throttle for all wheels, and vz as turn bias.
        // This is intentionally simple to get the pipe working end-to-end.
        auto it = machines_.find(uuid);
        if (it == machines_.end()) {
            return;
        }

        types::WheelControl wheel_ctrl;
        wheel_ctrl.uuid = uuid;

        const auto &cfg = it->second.config();
        const size_t n = cfg.wheels.size();
        wheel_ctrl.throttle.assign(n, 0.0f);
        wheel_ctrl.steering.assign(n, 0.0f);

        const float base = static_cast<float>(cmd.twist.linear.vx);
        const float yaw = static_cast<float>(cmd.twist.angular.vz);

        // Differential-ish: left gets -(yaw), right gets +(yaw)
        for (size_t i = 0; i < n; ++i) {
            const bool left = (i < cfg.controls.left_side.size()) ? cfg.controls.left_side[i] : false;
            wheel_ctrl.throttle[i] = base + (left ? -yaw : yaw);
        }

        it->second.apply_control(wheel_ctrl, 0.016f);
    }

    // ============================================================================
    // IPC/TCP/SHM Connection Management
    // ============================================================================

    void Simulator::process_agent47_connections() {
        if (!agent47_listen_pipe_.has_value()) {
            return;
        }

        auto conn_res = agent47_listen_pipe_->accept();
        if (conn_res.is_err()) {
            return;
        }

        auto conn = std::move(conn_res.value());
        std::string uuid = "agent_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

        Agent47Peer p;
        p.pipe.emplace(std::move(conn));
        p.rpc = std::make_unique<netpipe::Remote<netpipe::Bidirect>>(*p.pipe->stream().get(),
                                                                     /*max_concurrent=*/100,
                                                                     /*enable_metrics=*/false,
                                                                     /*recv_timeout_ms=*/100,
                                                                     /*handler_threads=*/2,
                                                                     /*max_handler_queue=*/100,
                                                                     /*handler_timeout_ms=*/0,
                                                                     /*max_incoming=*/100);

        agent47_peers_.try_emplace(uuid, std::move(p));
        register_agent47_handlers(agent47_peers_.at(uuid), uuid);

        echo::info("[Simulator] agent47 peer connected uuid=", uuid);
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
            auto it = agent47_peers_.find(uuid);
            if (it != agent47_peers_.end()) {
                if (it->second.rpc) it->second.rpc.reset();
                if (it->second.pipe.has_value()) {
                    it->second.pipe->close();
                    it->second.pipe.reset();
                }
                agent47_peers_.erase(it);
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

        // Step 3b: For agent47 mode, apply latest command received via RPC method 1
        if (conn_ != Conn::LOCAL) {
            for (auto &[uuid, peer] : agent47_peers_) {
                std::optional<agent47::types::Command> cmd;
                {
                    std::lock_guard<std::mutex> lock(peer.cmd_mutex);
                    if (peer.has_cmd) {
                        cmd = peer.last_cmd.value;
                        peer.has_cmd = false;
                    }
                }
                if (cmd.has_value()) {
                    apply_agent47_command(uuid, *cmd, dt);
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
            process_agent47_connections();
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
