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
        if (local_mode_) {
            std::cerr << "[Agent] spawn() should not be called in LOCAL mode" << std::endl;
            return false;
        }

        // Connect RPC client to spawn server
        flatsim::Endpoint spawn_endpoint;
        if (transport_type_ == flatsim::Endpoint::Type::IPC) {
            auto dir = ipc_dir();
            spawn_endpoint = flatsim::Endpoint::ipc((dir / "flatsim_spawn").string());
        } else if (transport_type_ == flatsim::Endpoint::Type::TCP) {
            std::string host = address_.empty() ? "127.0.0.1" : address_;
            spawn_endpoint = flatsim::Endpoint::tcp(host, 5555);
        } else if (transport_type_ == flatsim::Endpoint::Type::SHM) {
            spawn_endpoint = flatsim::Endpoint::shm("flatsim_spawn", 1024 * 1024);
        }

        if (!rpc_client_->connect(spawn_endpoint)) {
            std::cerr << "[Agent] Failed to connect to spawn server at " << spawn_endpoint.to_string() << std::endl;
            return false;
        }

        // Prepare spawn request
        types::ser::Request req;
        req.type = types::ser::MsgType::SPAWN;
        req.machine = types::ser::Machine::from_machine(machine_.config());

        auto req_data = datapod::serialize(req);
        auto resp_data = rpc_client_->call(flatsim::RpcMethod::SPAWN, req_data, 5000);

        if (resp_data.empty()) {
            std::cerr << "[Agent] Spawn RPC call failed or timed out" << std::endl;
            return false;
        }

        try {
            auto resp = datapod::deserialize<datapod::Mode::NONE, types::ser::Response>(resp_data);
            if (!resp.success) {
                std::cerr << "[Agent] Spawn request rejected by simulator" << std::endl;
                return false;
            }

            // Get endpoint info from response
            std::string uuid = machine_.uuid();
            std::string uplink_addr(resp.zmq.uplink_endpoint.view());
            std::string downlink_addr(resp.zmq.downlink_endpoint.view());

            // Parse endpoints and create streams
            if (transport_type_ == flatsim::Endpoint::Type::TCP) {
                // Parse TCP endpoints: "tcp://host:port"
                auto parse_tcp = [](const std::string &addr) -> std::pair<std::string, uint16_t> {
                    size_t colon_pos = addr.rfind(':');
                    if (colon_pos != std::string::npos) {
                        std::string host = addr.substr(6, colon_pos - 6); // Skip "tcp://"
                        uint16_t port = std::stoi(addr.substr(colon_pos + 1));
                        return {host, port};
                    }
                    return {"127.0.0.1", 5600};
                };

                auto [up_host, up_port] = parse_tcp(uplink_addr);
                auto [down_host, down_port] = parse_tcp(downlink_addr);

                uplink_tcp_ = std::make_unique<netpipe::TcpStream>();
                downlink_tcp_ = std::make_unique<netpipe::TcpStream>();

                auto up_res = uplink_tcp_->connect(netpipe::TcpEndpoint{dp::String(up_host.c_str()), up_port});
                auto down_res = downlink_tcp_->connect(netpipe::TcpEndpoint{dp::String(down_host.c_str()), down_port});

                if (up_res.is_err() || down_res.is_err()) {
                    std::cerr << "[Agent] Failed to connect uplink/downlink streams" << std::endl;
                    return false;
                }
            } else if (transport_type_ == flatsim::Endpoint::Type::IPC) {
                // Parse IPC endpoints: "ipc:///path"
                auto parse_ipc = [](const std::string &addr) -> std::string {
                    return addr.substr(6); // Skip "ipc://"
                };

                uplink_ipc_ = std::make_unique<netpipe::IpcStream>();
                downlink_ipc_ = std::make_unique<netpipe::IpcStream>();

                auto up_res =
                    uplink_ipc_->connect_ipc(netpipe::IpcEndpoint{dp::String(parse_ipc(uplink_addr).c_str())});
                auto down_res =
                    downlink_ipc_->connect_ipc(netpipe::IpcEndpoint{dp::String(parse_ipc(downlink_addr).c_str())});

                if (up_res.is_err() || down_res.is_err()) {
                    std::cerr << "[Agent] Failed to connect uplink/downlink IPC streams" << std::endl;
                    return false;
                }
            } else if (transport_type_ == flatsim::Endpoint::Type::SHM) {
                // Parse SHM endpoints: "shm://name"
                auto parse_shm = [](const std::string &addr) -> std::string {
                    return addr.substr(6); // Skip "shm://"
                };

                uplink_shm_ = std::make_unique<netpipe::ShmStream>();
                downlink_shm_ = std::make_unique<netpipe::ShmStream>();

                auto up_res = uplink_shm_->connect_shm(
                    netpipe::ShmEndpoint{dp::String(parse_shm(uplink_addr).c_str()), 1024 * 1024});
                auto down_res = downlink_shm_->connect_shm(
                    netpipe::ShmEndpoint{dp::String(parse_shm(downlink_addr).c_str()), 1024 * 1024});

                if (up_res.is_err() || down_res.is_err()) {
                    std::cerr << "[Agent] Failed to connect uplink/downlink SHM streams" << std::endl;
                    return false;
                }
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

    bool Agent::despawn() {
        if (!spawned_ || local_mode_) {
            return false;
        }

        types::ser::Request req;
        req.type = types::ser::MsgType::DESPAWN;
        req.uuid = datapod::String(machine_.uuid());

        auto req_data = datapod::serialize(req);
        auto resp_data = rpc_client_->call(flatsim::RpcMethod::DESPAWN, req_data, 5000);

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

        // NETWORKED MODE: netpipe communication with simulator

        // Get the appropriate stream based on transport type
        netpipe::Stream *uplink = nullptr;
        netpipe::Stream *downlink = nullptr;

        if (transport_type_ == flatsim::Endpoint::Type::TCP) {
            uplink = uplink_tcp_.get();
            downlink = downlink_tcp_.get();
        } else if (transport_type_ == flatsim::Endpoint::Type::IPC) {
            uplink = uplink_ipc_.get();
            downlink = downlink_ipc_.get();
        } else if (transport_type_ == flatsim::Endpoint::Type::SHM) {
            uplink = uplink_shm_.get();
            downlink = downlink_shm_.get();
        }

        if (!uplink || !downlink) {
            std::cerr << "[Agent] Uplink/downlink streams not initialized" << std::endl;
            return;
        }

        // Send heartbeat to simulator (non-blocking, fire-and-forget)
        static int tick_count = 0;
        tick_count++;
        if (tick_count % 30 == 0) { // Send heartbeat every 30 ticks (~0.5s at 60Hz)
            types::ser::Request hb_req;
            hb_req.type = types::ser::MsgType::HEARTBEAT;
            hb_req.uuid = datapod::String(machine_.uuid());

            auto hb_data = flatsim::wire::pack(flatsim::wire::Kind::HEARTBEAT, hb_req);
            netpipe::Message hb_msg(hb_data.begin(), hb_data.end());
            uplink->send(hb_msg); // Ignore errors
        }

        // BLOCKING: Wait for STATE update from simulator FIRST (may receive SENSORS first)
        const auto start = std::chrono::steady_clock::now();
        bool got_state = false;
        while (!got_state) {
            const auto elapsed_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
            const int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
            if (remaining_ms <= 0) {
                break;
            }

            downlink->set_recv_timeout(static_cast<uint32_t>(remaining_ms));
            auto res = downlink->recv();
            if (res.is_err()) {
                break;
            }

            std::vector<uint8_t> bytes(res.value().begin(), res.value().end());
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
        netpipe::Message ctrl_msg(ctrl_data.begin(), ctrl_data.end());
        uplink->send(ctrl_msg); // Ignore errors
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
            if (local_mode_) {
                // LOCAL mode: sensor config is handled directly by simulator
                return;
            }

            // Get uplink stream
            netpipe::Stream *uplink = nullptr;
            if (transport_type_ == flatsim::Endpoint::Type::TCP) {
                uplink = uplink_tcp_.get();
            } else if (transport_type_ == flatsim::Endpoint::Type::IPC) {
                uplink = uplink_ipc_.get();
            } else if (transport_type_ == flatsim::Endpoint::Type::SHM) {
                uplink = uplink_shm_.get();
            }

            if (!uplink || !spawned_) {
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
            netpipe::Message np_msg(bytes.begin(), bytes.end());
            uplink->send(np_msg); // Ignore errors
        });

        // Send config for existing sensors
        if (local_mode_ || !spawned_) {
            return;
        }

        netpipe::Stream *uplink = nullptr;
        if (transport_type_ == flatsim::Endpoint::Type::TCP) {
            uplink = uplink_tcp_.get();
        } else if (transport_type_ == flatsim::Endpoint::Type::IPC) {
            uplink = uplink_ipc_.get();
        } else if (transport_type_ == flatsim::Endpoint::Type::SHM) {
            uplink = uplink_shm_.get();
        }

        if (!uplink) {
            return;
        }

        machine_.sensors.for_each([this, uplink](fs::Sensor &sensor) {
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
            netpipe::Message np_msg(bytes.begin(), bytes.end());
            uplink->send(np_msg); // Ignore errors
        });
    }

} // namespace agent
