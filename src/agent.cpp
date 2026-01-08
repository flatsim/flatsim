#include "flatsim/agent.hpp"
#include "flatsim/agent/sensor/lidar_sensor.hpp"
#include "flatsim/tagged_zmq.hpp"
#include "flatsim/transport.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <netpipe/netpipe.hpp>

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

    // Constructor for networked mode (IPC/TCP/SHM)
    Agent::Agent(const std::string &address) : local_mode_(false), address_(address), rec_(nullptr) {
        // Determine transport type from address
        if (address_.empty()) {
            transport_type_ = flatsim::Endpoint::Type::IPC;
        } else if (address_.starts_with("shm://")) {
            transport_type_ = flatsim::Endpoint::Type::SHM;
        } else {
            transport_type_ = flatsim::Endpoint::Type::TCP;
        }

        // Create RPC client for spawn/despawn
        rpc_client_ = std::make_unique<flatsim::RpcClient>();

        // Note: uplink/downlink streams will be created after spawn
    }

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

    Agent::~Agent() {
        // Only do netpipe cleanup in networked mode
        if (!local_mode_) {
            if (spawned_) {
                despawn();
            }
            if (rpc_client_) rpc_client_->close();
            if (uplink_tcp_) uplink_tcp_->close();
            if (downlink_tcp_) downlink_tcp_->close();
            if (uplink_ipc_) uplink_ipc_->close();
            if (downlink_ipc_) downlink_ipc_->close();
            if (uplink_shm_) uplink_shm_->close();
            if (downlink_shm_) downlink_shm_->close();
        }
    }

    void Agent::set_machine(const types::Machine &config) {
        machine_ = Machine(rec_, config);
        machine_.init();
        install_sensor_callbacks();
    }

    bool Agent::spawn() {
        // TODO: Implement netpipe RPC-based spawn
        // For now, networked mode spawn is not implemented
        // Use LOCAL mode with Simulator::spawn_agent() instead
        std::cerr << "[Agent] spawn() not yet implemented for netpipe - use LOCAL mode" << std::endl;
        return false;
    }

    bool Agent::despawn() {
        // TODO: Implement netpipe RPC-based despawn
        std::cerr << "[Agent] despawn() not yet implemented for netpipe" << std::endl;
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

        // NETWORKED MODE: netpipe communication with simulator
        // TODO: Implement netpipe-based tick for networked mode
        // For now, just run machine tick without state updates
        machine_.tick(dt);
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
        // TODO: Implement netpipe-based sensor callbacks for networked mode
        // For now, sensor callbacks only work in LOCAL mode
        machine_.sensors.set_on_add([this](fs::Sensor &sensor) {
            if (local_mode_) {
                // LOCAL mode: sensor config is handled directly by simulator
            }
        });
    }

} // namespace agent
