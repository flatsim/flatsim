#include "flatsim/agent.hpp"
#include "flatsim/agent/sensor/lidar_sensor.hpp"
#include "flatsim/tagged_zmq.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace agent {

    static std::filesystem::path ipc_dir() {
        const char *env = std::getenv("FLATSIM_IPC_DIR");
        std::filesystem::path dir = env && *env ? std::filesystem::path(env) : std::filesystem::path("/tmp");
        if (dir.is_relative()) {
            dir = std::filesystem::absolute(dir);
        }
        return dir;
    }

    static std::string ipc_endpoint(const std::filesystem::path &path) { return "ipc://" + path.string(); }

    // Constructor for networked mode (IPC/TCP)
    Agent::Agent(const std::string &address) : local_mode_(false), ctx_(1), address_(address), rec_(nullptr) {
        // Create spawn socket (REQ)
        spawn_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::req);
        std::string spawn_addr;
        if (address_.empty()) {
            spawn_addr = ipc_endpoint(ipc_dir() / "flatsim_spawn");
        } else if (address_.starts_with("tcp://") || address_.starts_with("ipc://")) {
            // Allow passing a full ZMQ endpoint explicitly.
            spawn_addr = address_;
        } else {
            // Treat non-empty value as TCP host and use the default spawn port.
            spawn_addr = "tcp://" + address_ + ":5555";
        }
        spawn_socket_->connect(spawn_addr);

        // Create uplink/downlink sockets (will connect after spawn)
        uplink_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::push);
        downlink_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::sub);
        downlink_socket_->set(zmq::sockopt::subscribe, "");
        downlink_socket_->set(zmq::sockopt::rcvtimeo, 0);
    }

    // Constructor for local mode (owned by Simulator)
    Agent::Agent(const types::Machine &config, std::shared_ptr<rerun::RecordingStream> rec)
        : local_mode_(true), ctx_(1), rec_(rec), spawned_(true) {

        // No ZMQ sockets needed in local mode
        // Initialize machine with config
        machine_ = Machine(rec_, config);

        // Initialize all managers (sensors, controls, network, power, container)
        machine_.init();
        install_sensor_callbacks();
    }

    Agent::~Agent() {
        // Only do ZMQ cleanup in networked mode
        if (!local_mode_) {
            if (spawned_) {
                despawn();
            }
            if (spawn_socket_) spawn_socket_->close();
            if (uplink_socket_) uplink_socket_->close();
            if (downlink_socket_) downlink_socket_->close();
        }
        ctx_.close();
    }

    void Agent::set_machine(const types::Machine &config) {
        machine_ = Machine(rec_, config);
        machine_.init();
        install_sensor_callbacks();
    }

    bool Agent::spawn() {
        types::ser::Request req;
        req.type = types::ser::MsgType::SPAWN;
        req.machine = types::ser::Machine::from_machine(machine_.config());

        auto data = datapod::serialize(req);
        spawn_socket_->send(zmq::buffer(data), zmq::send_flags::none);

        zmq::message_t reply;
        spawn_socket_->set(zmq::sockopt::rcvtimeo, 5000);
        auto result = spawn_socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(reply.data()),
                                        static_cast<uint8_t *>(reply.data()) + reply.size());
            try {
                auto resp = datapod::deserialize<datapod::Mode::NONE, types::ser::Response>(buffer);
                if (resp.success) {
                    // Connect control and state sockets for this machine
                    std::string uuid = machine_.uuid();
                    std::string uplink_addr(resp.zmq.uplink_endpoint.view());
                    std::string downlink_addr(resp.zmq.downlink_endpoint.view());

                    // Backwards-compatible fallback if talking to an older simulator.
                    if (uplink_addr.empty() || downlink_addr.empty()) {
                        if (address_.empty()) {
                            auto dir = ipc_dir();
                            uplink_addr = ipc_endpoint(dir / ("flatsim_uplink_" + uuid));
                            downlink_addr = ipc_endpoint(dir / ("flatsim_downlink_" + uuid));
                        } else {
                            const std::string host = (address_.starts_with("tcp://") || address_.starts_with("ipc://"))
                                                         ? "127.0.0.1"
                                                         : address_;
                            uplink_addr = "tcp://" + host + ":5600";
                            downlink_addr = "tcp://" + host + ":5601";
                        }
                    }

                    uplink_socket_->connect(uplink_addr);
                    downlink_socket_->connect(downlink_addr);

                    // Create RecordingStream using info from simulator
                    std::string rerun_addr(resp.rerun.grpc_address.view());
                    std::string rec_id(resp.rerun.recording_id.view());
                    std::string app_id(resp.rerun.application_id.view());

                    rec_ = std::make_shared<rerun::RecordingStream>(app_id, rec_id);
                    auto conn_result = rec_->connect_grpc(rerun_addr);
                    if (conn_result.is_ok()) {
                    } else {
                        std::cerr << "[Agent] Warning: Failed to connect to Rerun Viewer" << std::endl;
                    }

                    // Update machine with rerun and initialize all managers
                    machine_ = Machine(rec_, machine_.config());
                    machine_.init();
                    install_sensor_callbacks();

                    // Update state from response
                    for (const auto &ms : resp.state.machines) {
                        if (std::string(ms.uuid.view()) == machine_.uuid()) {
                            machine_.update_state(ms);
                            break;
                        }
                    }

                    spawned_ = true;
                    return true;
                }
            } catch (const std::exception &e) {
                std::cerr << "[Agent] Failed to deserialize spawn response: " << e.what() << std::endl;
            }
        }
        return false;
    }

    bool Agent::despawn() {
        if (!spawned_) {
            return false;
        }

        types::ser::Request req;
        req.type = types::ser::MsgType::DESPAWN;
        req.uuid = datapod::String(machine_.uuid());

        auto data = datapod::serialize(req);
        spawn_socket_->send(zmq::buffer(data), zmq::send_flags::none);

        zmq::message_t reply;
        auto result = spawn_socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(reply.data()),
                                        static_cast<uint8_t *>(reply.data()) + reply.size());
            try {
                auto resp = datapod::deserialize<datapod::Mode::NONE, types::ser::Response>(buffer);
                if (resp.success) {
                    spawned_ = false;
                    return true;
                }
            } catch (const std::exception &e) {
                std::cerr << "[Agent] Failed to deserialize despawn response: " << e.what() << std::endl;
            }
        }
        return false;
    }

    void Agent::set_linear(float linear) { machine_.controls.set_linear(linear); }

    void Agent::set_angular(float angular) { machine_.controls.set_angular(angular); }

    void Agent::set_velocity(float linear, float angular) {
        machine_.controls.set_linear(linear);
        machine_.controls.set_angular(angular);
    }

    void Agent::tick(float dt, int timeout_ms) {
        if (!spawned_) {
            return;
        }

        if (local_mode_) {
            // LOCAL MODE: State is already updated by Simulator via update_from_physics()
            // Use sensor data from simulator if available
            if (sensor_data_.has_gps || sensor_data_.has_imu || sensor_data_.has_lidar) {
                machine_.tick(dt, sensor_data_);
            } else {
                machine_.tick(dt);
            }
            return;
        }

        // NETWORKED MODE: IPC/TCP communication with simulator

        // Send heartbeat to simulator (non-blocking, fire-and-forget)
        static int tick_count = 0;
        tick_count++;
        if (tick_count % 5 == 0) { // Send heartbeat every 30 ticks (~0.5s at 60Hz)
            types::ser::Request hb_req;
            hb_req.type = types::ser::MsgType::HEARTBEAT;
            hb_req.uuid = datapod::String(machine_.uuid());

            auto hb_data = flatsim::wire::pack(flatsim::wire::Kind::HEARTBEAT, hb_req);
            uplink_socket_->send(zmq::buffer(hb_data), zmq::send_flags::dontwait);
        }

        // BLOCKING: Wait for STATE update from simulator FIRST (may receive SENSORS first).
        const auto start = std::chrono::steady_clock::now();
        bool got_state = false;
        while (!got_state) {
            const auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
            const int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
            if (remaining_ms <= 0) {
                break;
            }

            downlink_socket_->set(zmq::sockopt::rcvtimeo, remaining_ms);
            zmq::message_t msg;
            auto result = downlink_socket_->recv(msg, zmq::recv_flags::none);
            if (!result) {
                break;
            }

            std::vector<uint8_t> bytes(static_cast<uint8_t *>(msg.data()),
                                       static_cast<uint8_t *>(msg.data()) + msg.size());

            const auto tagged = flatsim::wire::unpack(std::move(bytes));
            switch (tagged.kind) {
            case flatsim::wire::Kind::STATE: {
                auto ms = flatsim::wire::deserialize<types::ser::MachineState>(tagged.payload);
                if (std::string(ms.uuid.view()) == machine_.uuid()) {
                    machine_.update_state(ms);
                    got_state = true;
                }
                break;
            }
            case flatsim::wire::Kind::SENSORS: {
                auto ss = flatsim::wire::deserialize<types::ser::SensorState>(tagged.payload);
                if (std::string(ss.uuid.view()) == machine_.uuid()) {
                    sensor_data_ = ss.to_sensor_data();
                }
                break;
            }
            default:
                break;
            }
        }

        // Call machine tick to process state update and run all managers
        if (sensor_data_.has_gps || sensor_data_.has_imu || sensor_data_.has_lidar) {
            machine_.tick(dt, sensor_data_);
        } else {
            machine_.tick(dt);
        }

        // Get current control from machine's control manager and send to simulator
        auto wheel_ctrl = machine_.controls.get_wheel_control();
        auto ctrl_ser = types::ser::WheelControl::from_control(wheel_ctrl);
        auto ctrl_data = flatsim::wire::pack(flatsim::wire::Kind::CONTROL, ctrl_ser);
        uplink_socket_->send(zmq::buffer(ctrl_data), zmq::send_flags::dontwait);
    }

    void Agent::tock() {
        if (!spawned_) {
            return;
        }

        // Call machine tock for visualization (container, tracker, etc.)
        machine_.tock();
    }

    // LOCAL mode: Update state from physics (called by Simulator)
    void Agent::update_from_physics(const types::ser::MachineState &state) { machine_.update_state(state); }

    // LOCAL mode: Update sensor data (called by Simulator)
    void Agent::update_from_sensors(const types::ser::SensorState &state) { sensor_data_ = state.to_sensor_data(); }

    // LOCAL mode: Get current wheel control (called by Simulator)
    types::WheelControl Agent::get_wheel_control() const {
        auto ctrl = machine_.controls.get_wheel_control();
        // Apply speed scale to throttle
        for (auto &t : ctrl.throttle) {
            t *= speed_scale_;
        }
        return ctrl;
    }

    void Agent::brake() {
        // Set zero velocity and apply brake
        machine_.controls.set_linear(0.0f);
        machine_.controls.set_angular(0.0f);
        // TODO: When brake force is implemented in WheelControl, set it here
    }

    void Agent::teleport(const datapod::Pose &pose) {
        if (local_mode_) {
            // LOCAL mode: use callback to Simulator
            if (teleport_callback_) {
                teleport_callback_(machine_.uuid(), pose);
            }
        } else {
            // IPC/TCP mode: TODO - send teleport request to simulator
            // For now, just log a warning
            std::cerr << "[Agent] teleport() not yet supported in IPC/TCP mode" << std::endl;
        }
    }

    void Agent::install_sensor_callbacks() {
        machine_.sensors.set_on_add([this](fs::Sensor &sensor) {
            if (local_mode_ || !spawned_ || !uplink_socket_) {
                return;
            }

            auto *lidar = dynamic_cast<fs::LIDARSensor *>(&sensor);
            if (!lidar) {
                return;
            }

            types::LidarConfig cfg;
            cfg.enabled = true;
            cfg.min_range = lidar->get_min_range();
            cfg.max_range = lidar->get_max_range();
            cfg.fov_deg = lidar->get_fov_deg();
            cfg.resolution_deg = lidar->get_resolution_deg();

            auto msg = types::ser::LidarConfigMsg::from_config(machine_.uuid(), cfg);
            auto bytes = flatsim::wire::pack(flatsim::wire::Kind::LIDAR_CFG, msg);
            uplink_socket_->send(zmq::buffer(bytes), zmq::send_flags::dontwait);
        });

        if (local_mode_ || !spawned_ || !uplink_socket_) {
            return;
        }

        machine_.sensors.for_each([this](fs::Sensor &sensor) {
            auto *lidar = dynamic_cast<fs::LIDARSensor *>(&sensor);
            if (!lidar) {
                return;
            }

            types::LidarConfig cfg;
            cfg.enabled = true;
            cfg.min_range = lidar->get_min_range();
            cfg.max_range = lidar->get_max_range();
            cfg.fov_deg = lidar->get_fov_deg();
            cfg.resolution_deg = lidar->get_resolution_deg();

            auto msg = types::ser::LidarConfigMsg::from_config(machine_.uuid(), cfg);
            auto bytes = flatsim::wire::pack(flatsim::wire::Kind::LIDAR_CFG, msg);
            uplink_socket_->send(zmq::buffer(bytes), zmq::send_flags::dontwait);
        });
    }

} // namespace agent
