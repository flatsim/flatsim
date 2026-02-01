#include "flatsim/agent.hpp"
#include "datapod/datapod.hpp"
#include "flatsim/agent/sensor/lidar_sensor.hpp"
#include "flatsim/simulator/machine.hpp"
#include "flatsim/tagged_zmq.hpp"
#include <agent47/model/urdf.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace agent {

    static constexpr dp::u32 AGENT47_METHOD_COMMAND = 1;
    static constexpr dp::u32 AGENT47_METHOD_FEEDBACK = 2;

    datapod::robot::Model Agent::load_model_from_urdf(const std::filesystem::path &urdf_path) {
        std::ifstream file(urdf_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open URDF file: " + urdf_path.string());
        }
        std::stringstream buffer;
        buffer << file.rdbuf();

        auto result = agent47::from_urdf_string(dp::String(buffer.str().c_str()));
        if (result.is_err()) {
            throw std::runtime_error("Failed to parse URDF: " + urdf_path.string());
        }
        return result.value();
    }

    static std::filesystem::path ipc_dir() {
        const char *env = std::getenv("FLATSIM_IPC_DIR");
        std::filesystem::path dir = env && *env ? std::filesystem::path(env) : std::filesystem::path("/tmp");
        if (dir.is_relative()) {
            dir = std::filesystem::absolute(dir);
        }
        return dir;
    }

    static std::string ipc_endpoint(const std::filesystem::path &path) { return "ipc://" + path.string(); }

    // // Constructor for networked mode (IPC/TCP/SHM)
    // Agent::Agent(const std::string &address) : local_mode_(false), address_(address), rec_(nullptr) {
    //     // Determine transport type from address
    //     if (address_.empty()) {
    //         transport_type_ = flatsim::Endpoint::Type::IPC;
    //     } else if (address_.starts_with("shm://")) {
    //         transport_type_ = flatsim::Endpoint::Type::SHM;
    //     } else {
    //         transport_type_ = flatsim::Endpoint::Type::TCP;
    //     }
    //
    //     // NEW: Single RpcPeer will be created after spawn
    //     // Note: peer_ will be initialized in spawn()
    // }

    // Constructor for local mode (owned by Simulator)
    Agent::Agent(const types::Machine &config, std::shared_ptr<rerun::RecordingStream> rec)
        : local_mode_(true), rec_(rec), spawned_(true) {

        // No netpipe needed in local mode
        // Initialize machine with config
        machine_ = Machine(rec_, config);

        // Initialize all managers (sensors, controls, network, power, container)
        machine_.init();
        install_sensor_callbacks();
    }

    // Constructor for local mode (owned by Simulator)
    Agent::Agent(dp::String urdf_path, dp::robot::Identity identity, agent47::Bridge *bridge,
                 std::shared_ptr<rerun::RecordingStream> rec)
        : local_mode_(true), rec_(rec), spawned_(true) {

        agent47_ = std::make_shared<agent47::Agent>(urdf_path, identity, bridge);

        // agent47 owns a datapod::robot::Model; flatsim's Agent-side Machine expects a legacy `types::Machine`
        // config, so adapt via the simulator's URDF->Machine helper.
        auto config = simulator::machine_from_model(agent47_->model_.model, datapod::Pose{}, std::nullopt);
        machine_ = Machine(rec_, config);

        // Initialize all managers (sensors, controls, network, power, container)
        machine_.init();
        install_sensor_callbacks();
    }

    // Constructor for networked mode via agent47 PipeBridge
    Agent::Agent(const types::Machine &config, const std::string &agent47_endpoint,
                 std::shared_ptr<rerun::RecordingStream> rec)
        : local_mode_(false), agent47_endpoint_(agent47_endpoint), rec_(rec), spawned_(false) {
        machine_ = Machine(rec_, config);
        machine_.init();
        install_sensor_callbacks();

        // Create agent47 wrapper with an owned PipeBridge.
        auto *bridge = new agent47::PipeBridge();
        agent47_ = std::make_shared<agent47::Agent>(dp::robot::Robot{}, bridge);
    }

    Agent::~Agent() {
        if (!local_mode_ && spawned_) {
            despawn();
        }
    }

    void Agent::set_machine(const types::Machine &config) {
        machine_ = Machine(rec_, config);
        machine_.init();
        install_sensor_callbacks();
    }

    bool Agent::spawn() {
        if (local_mode_) {
            std::cerr << "[Agent] spawn() should not be called in LOCAL mode" << std::endl;
            return false;
        }

        if (!agent47_ || !agent47_->bridge_) {
            std::cerr << "[Agent] agent47 bridge not configured" << std::endl;
            return false;
        }
        if (!agent47_endpoint_.has_value()) {
            std::cerr << "[Agent] missing agent47 endpoint" << std::endl;
            return false;
        }

        if (!agent47_->bridge_->connect(*agent47_endpoint_)) {
            std::cerr << "[Agent] failed to connect agent47 bridge to " << *agent47_endpoint_ << std::endl;
            return false;
        }

        spawned_ = true;
        return true;
    }

    bool Agent::despawn() {
        if (!spawned_ || local_mode_) {
            return false;
        }

        if (agent47_ && agent47_->bridge_) {
            agent47_->bridge_->disconnect();
        }
        spawned_ = false;
        return true;
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

        // NETWORKED MODE: agent47 PipeBridge
        if (!agent47_ || !agent47_->bridge_) {
            return;
        }

        // Drain sensor packets into flatsim SensorData so Machine can use them.
        // Note: packet.payload is dp::Mode::WITH_VERSION; we deserialize expected cista types.
        for (int i = 0; i < 16; ++i) {
            agent47::types::SensorPacket pkt;
            if (!agent47_->bridge_->sensor(pkt, 0)) {
                break;
            }
            try {
                if (pkt.kind == agent47::types::SensorKind::Lidar) {
                    auto lidar = dp::deserialize<dp::Mode::WITH_VERSION, types::ser::LidarData>(pkt.payload);
                    sensor_data_.lidar = lidar.to_lidar();
                    sensor_data_.has_lidar = true;
                } else if (pkt.kind == agent47::types::SensorKind::Gnss) {
                    auto gps = dp::deserialize<dp::Mode::WITH_VERSION, types::ser::GpsData>(pkt.payload);
                    sensor_data_.gps = gps.to_gps();
                    sensor_data_.has_gps = true;
                } else if (pkt.kind == agent47::types::SensorKind::Imu) {
                    auto imu = dp::deserialize<dp::Mode::WITH_VERSION, types::ser::ImuData>(pkt.payload);
                    sensor_data_.imu = imu.to_imu();
                    sensor_data_.has_imu = true;
                }
            } catch (...) {
                // ignore malformed packets
            }
        }

        // Receive feedback (pose/twist) from simulator.
        dp::Stamp<agent47::types::Feedback> fb;
        if (agent47_->bridge_->recv(fb, timeout_ms)) {
            types::ser::MachineState ms;
            ms.uuid = datapod::String(machine_.uuid());
            ms.pose.position.x = static_cast<float>(fb.value.pose.point.x);
            ms.pose.position.y = static_cast<float>(fb.value.pose.point.y);
            ms.pose.angle = static_cast<float>(fb.value.pose.rotation.to_euler().yaw);
            ms.velocity.x = static_cast<float>(fb.value.twist.linear.vx);
            ms.velocity.y = static_cast<float>(fb.value.twist.linear.vy);
            ms.angular_vel = static_cast<float>(fb.value.twist.angular.vz);
            machine_.update_state(ms);
        }

        // Run the local autonomy/controls stack.
        if (sensor_data_.has_gps || sensor_data_.has_imu || sensor_data_.has_lidar) {
            machine_.tick(dt, sensor_data_);
        } else {
            machine_.tick(dt);
        }

        // Send twist command based on current machine controls (linear/angular).
        float linear = 0.0f;
        float angular = 0.0f;
        machine_.get_velocity(linear, angular);

        dp::Stamp<agent47::types::Command> cmd;
        cmd.timestamp = dp::Stamp<agent47::types::Command>::now();
        cmd.value.valid = true;
        cmd.value.twist.linear.vx = linear;
        cmd.value.twist.angular.vz = angular;
        agent47_->bridge_->send(cmd);
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

    void Agent::install_sensor_callbacks() {
        machine_.sensors.set_on_add([this](fs::Sensor &sensor) {
            if (local_mode_) {
                // LOCAL mode: sensor config is handled directly by simulator
                return;
            }

            if (!spawned_) {
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

            (void)cfg;
        });

        // No sensor config on agent47 bridge yet.
    }

} // namespace agent
