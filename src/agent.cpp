#include "flatsim/agent.hpp"
#include "datapod/datapod.hpp"
#include "flatsim/agent/sensor/lidar_sensor.hpp"
#include "flatsim/simulator/machine.hpp"
#include "flatsim/tagged_zmq.hpp"
#include "flatsim/transport.hpp"
#include <agent47/model/urdf.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <sstream>
#include <stdexcept>

namespace agent {

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

    Agent::~Agent() {
        // Only do netpipe cleanup in networked mode
        if (!local_mode_) {
            if (spawned_) {
                despawn();
            }
            if (peer_) {
                peer_->close();
            }
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

        // Create peer endpoint
        flatsim::Endpoint peer_endpoint;
        if (transport_type_ == flatsim::Endpoint::Type::IPC) {
            auto dir = ipc_dir();
            peer_endpoint = flatsim::Endpoint::ipc((dir / "flatsim_peer").string());
        } else if (transport_type_ == flatsim::Endpoint::Type::TCP) {
            std::string host = address_.empty() ? "127.0.0.1" : address_;
            peer_endpoint = flatsim::Endpoint::tcp(host, 5555);
        } else if (transport_type_ == flatsim::Endpoint::Type::SHM) {
            peer_endpoint = flatsim::Endpoint::shm("flatsim_peer", 1024 * 1024);
        }

        // Create and connect peer
        peer_ = std::make_unique<flatsim::RpcPeer>();
        if (!peer_->connect(peer_endpoint, 100, true)) {
            std::cerr << "[Agent] Failed to connect to simulator at " << peer_endpoint.to_string() << std::endl;
            std::cerr << "[Agent] Error: " << peer_->last_error() << std::endl;
            return false;
        }

        // Register handlers for incoming calls from Simulator
        register_peer_handlers();

        // Prepare spawn request
        types::ser::Request req;
        req.type = types::ser::MsgType::SPAWN;
        req.machine = types::ser::Machine::from_machine(machine_.config());

        auto req_data = datapod::serialize(req);
        auto resp_data = peer_->call(flatsim::RpcMethod::SPAWN, req_data, 5000);

        if (resp_data.empty()) {
            std::cerr << "[Agent] Spawn RPC call failed or timed out" << std::endl;
            std::cerr << "[Agent] Error: " << peer_->last_error() << std::endl;
            return false;
        }

        try {
            auto resp = datapod::deserialize<datapod::Mode::NONE, types::ser::Response>(resp_data);
            if (!resp.success) {
                std::cerr << "[Agent] Spawn request rejected by simulator" << std::endl;
                return false;
            }

            // Create RecordingStream using info from simulator
            std::string rerun_addr(resp.rerun.grpc_address.view());
            std::string rec_id(resp.rerun.recording_id.view());
            std::string app_id(resp.rerun.application_id.view());

            rec_ = std::make_shared<rerun::RecordingStream>(app_id, rec_id);
            auto conn_result = rec_->connect_grpc(rerun_addr);
            if (conn_result.is_err()) {
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

        } catch (const std::exception &e) {
            std::cerr << "[Agent] Failed to deserialize spawn response: " << e.what() << std::endl;
            return false;
        }
    }

    void Agent::register_peer_handlers() {
        if (!peer_) {
            return;
        }

        // Register STATE handler (receives state updates from simulator)
        peer_->register_method(flatsim::RpcMethod::STATE, [this](const std::vector<uint8_t> &req) {
            try {
                auto state = datapod::deserialize<datapod::Mode::NONE, types::ser::MachineState>(req);
                if (std::string(state.uuid.view()) == machine_.uuid()) {
                    machine_.update_state(state);
                }
            } catch (const std::exception &e) {
                std::cerr << "[Agent] Failed to deserialize STATE: " << e.what() << std::endl;
            }
            // Return ACK
            std::vector<uint8_t> ack{1};
            return ack;
        });

        // Register SENSORS handler (receives sensor data from simulator)
        peer_->register_method(flatsim::RpcMethod::SENSORS, [this](const std::vector<uint8_t> &req) {
            try {
                auto sensors = datapod::deserialize<datapod::Mode::NONE, types::ser::SensorState>(req);
                if (std::string(sensors.uuid.view()) == machine_.uuid()) {
                    sensor_data_ = sensors.to_sensor_data();
                }
            } catch (const std::exception &e) {
                std::cerr << "[Agent] Failed to deserialize SENSORS: " << e.what() << std::endl;
            }
            // Return ACK
            std::vector<uint8_t> ack{1};
            return ack;
        });

        // Register TELEPORT handler (receives teleport commands from simulator)
        peer_->register_method(flatsim::RpcMethod::TELEPORT, [this](const std::vector<uint8_t> &req) {
            try {
                auto pose = datapod::deserialize<datapod::Mode::NONE, datapod::Pose>(req);
                // Update machine pose directly
                // TODO: Implement proper teleport handling
                std::cerr << "[Agent] Received TELEPORT command" << std::endl;
            } catch (const std::exception &e) {
                std::cerr << "[Agent] Failed to deserialize TELEPORT: " << e.what() << std::endl;
            }
            // Return ACK
            std::vector<uint8_t> ack{1};
            return ack;
        });
    }

    bool Agent::despawn() {
        if (!spawned_ || local_mode_) {
            return false;
        }

        types::ser::Request req;
        req.type = types::ser::MsgType::DESPAWN;
        req.uuid = datapod::String(machine_.uuid());

        auto req_data = datapod::serialize(req);
        auto resp_data = peer_->call(flatsim::RpcMethod::DESPAWN, req_data, 5000);

        if (resp_data.empty()) {
            std::cerr << "[Agent] Despawn RPC call failed or timed out" << std::endl;
            return false;
        }

        try {
            auto resp = datapod::deserialize<datapod::Mode::NONE, types::ser::Response>(resp_data);
            if (resp.success) {
                spawned_ = false;
                return true;
            }
        } catch (const std::exception &e) {
            std::cerr << "[Agent] Failed to deserialize despawn response: " << e.what() << std::endl;
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

        // NETWORKED MODE: RpcPeer bidirectional communication

        // Send heartbeat to simulator (every 30 ticks)
        static int tick_count = 0;
        tick_count++;
        if (tick_count % 30 == 0) {
            types::ser::Request hb_req;
            hb_req.type = types::ser::MsgType::HEARTBEAT;
            hb_req.uuid = datapod::String(machine_.uuid());

            auto hb_data = datapod::serialize(hb_req);
            peer_->call(flatsim::RpcMethod::HEARTBEAT, hb_data, 500); // Short timeout for heartbeat
        }

        // Call machine tick to process state update and run all managers
        // Note: State updates are received via registered STATE handler (asynchronous)
        if (sensor_data_.has_gps || sensor_data_.has_imu || sensor_data_.has_lidar) {
            machine_.tick(dt, sensor_data_);
        } else {
            machine_.tick(dt);
        }

        // Get current control from machine's control manager and send to simulator
        auto wheel_ctrl = machine_.controls.get_wheel_control();
        auto ctrl_ser = types::ser::WheelControl::from_control(wheel_ctrl);
        auto ctrl_data = datapod::serialize(ctrl_ser);

        // Send control command to simulator
        peer_->call(flatsim::RpcMethod::CONTROL, ctrl_data, timeout_ms);
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
            // Networked mode: send teleport request via RPC
            auto pose_data = datapod::serialize(pose);
            peer_->call(flatsim::RpcMethod::TELEPORT, pose_data, 1000);
        }
    }

    void Agent::install_sensor_callbacks() {
        machine_.sensors.set_on_add([this](fs::Sensor &sensor) {
            if (local_mode_) {
                // LOCAL mode: sensor config is handled directly by simulator
                return;
            }

            if (!peer_ || !spawned_) {
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
            auto bytes = datapod::serialize(msg);
            peer_->call(flatsim::RpcMethod::LIDAR_CFG, bytes, 1000);
        });

        // Send config for existing sensors
        if (local_mode_ || !spawned_ || !peer_) {
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
            auto bytes = datapod::serialize(msg);
            peer_->call(flatsim::RpcMethod::LIDAR_CFG, bytes, 1000);
        });
    }

} // namespace agent
