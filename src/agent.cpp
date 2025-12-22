#include "flatsim/agent.hpp"
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

    Agent::Agent(const std::string &address) : ctx_(1), address_(address), rec_(nullptr) {
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
        std::cout << "[Agent] Connected to spawn socket: " << spawn_addr << std::endl;

        // Create control, state, and heartbeat sockets (will connect after spawn)
        control_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::push);
        state_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::sub);
        heartbeat_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::push);
        state_socket_->set(zmq::sockopt::subscribe, "");
        state_socket_->set(zmq::sockopt::rcvtimeo, 0);
    }

    Agent::~Agent() {
        if (spawned_) {
            despawn();
        }
        spawn_socket_->close();
        control_socket_->close();
        state_socket_->close();
        heartbeat_socket_->close();
        ctx_.close();
    }

    void Agent::set_machine(const types::Machine &config) { machine_ = Machine(rec_, config); }

    bool Agent::spawn() {
        types::ser::Request req;
        req.type = types::ser::MsgType::SPAWN;
        req.machine = types::ser::Machine::from_machine(machine_.config());

        auto data = cista::serialize(req);
        spawn_socket_->send(zmq::buffer(data), zmq::send_flags::none);
        std::cout << "[Agent] Sent SPAWN for: " << machine_.config().name << std::endl;

        zmq::message_t reply;
        spawn_socket_->set(zmq::sockopt::rcvtimeo, 5000);
        auto result = spawn_socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(reply.data()),
                                        static_cast<uint8_t *>(reply.data()) + reply.size());
            auto *resp = cista::deserialize<types::ser::Response>(buffer);
            if (resp && resp->success) {
                // Connect control and state sockets for this machine
                std::string uuid = machine_.uuid();
                std::string ctrl_addr(resp->zmq.control_endpoint.view());
                std::string state_addr(resp->zmq.state_endpoint.view());
                std::string hb_addr(resp->zmq.heartbeat_endpoint.view());

                // Backwards-compatible fallback if talking to an older simulator.
                if (ctrl_addr.empty() || state_addr.empty() || hb_addr.empty()) {
                    if (address_.empty()) {
                        auto dir = ipc_dir();
                        ctrl_addr = ipc_endpoint(dir / ("flatsim_ctrl_" + uuid));
                        state_addr = ipc_endpoint(dir / ("flatsim_state_" + uuid));
                        hb_addr = ipc_endpoint(dir / "flatsim_heartbeat");
                    } else {
                        const std::string host =
                            (address_.starts_with("tcp://") || address_.starts_with("ipc://")) ? "127.0.0.1" : address_;
                        ctrl_addr = "tcp://" + host + ":5600";
                        state_addr = "tcp://" + host + ":5601";
                        hb_addr = "tcp://" + host + ":5556";
                    }
                }

                control_socket_->connect(ctrl_addr);
                state_socket_->connect(state_addr);
                heartbeat_socket_->connect(hb_addr);

                std::cout << "[Agent] Connected to control: " << ctrl_addr << std::endl;
                std::cout << "[Agent] Connected to state: " << state_addr << std::endl;
                std::cout << "[Agent] Connected to heartbeat: " << hb_addr << std::endl;

                // Create RecordingStream using info from simulator
                std::string rerun_addr(resp->rerun.grpc_address.view());
                std::string rec_id(resp->rerun.recording_id.view());
                std::string app_id(resp->rerun.application_id.view());

                rec_ = std::make_shared<rerun::RecordingStream>(app_id, rec_id);
                auto conn_result = rec_->connect_grpc(rerun_addr);
                if (conn_result.is_ok()) {
                    std::cout << "[Agent] Connected to Rerun Viewer at " << rerun_addr << std::endl;
                    std::cout << "[Agent] Recording ID: " << rec_id << std::endl;
                } else {
                    std::cerr << "[Agent] Warning: Failed to connect to Rerun Viewer" << std::endl;
                }

                // Update machine with rerun
                machine_ = Machine(rec_, machine_.config());

                // Update state from response
                for (const auto &ms : resp->state.machines) {
                    if (std::string(ms.uuid.view()) == machine_.uuid()) {
                        machine_.update_state(ms);
                        break;
                    }
                }

                // Initialize control manager (includes tracker initialization)
                control_manager_.init(&machine_.config_mut(), rec_);

                spawned_ = true;
                std::cout << "[Agent] Spawn successful" << std::endl;
                return true;
            }
        }
        std::cout << "[Agent] Spawn failed" << std::endl;
        return false;
    }

    bool Agent::despawn() {
        if (!spawned_) {
            return false;
        }

        types::ser::Request req;
        req.type = types::ser::MsgType::DESPAWN;
        req.uuid = machine_.uuid();

        auto data = cista::serialize(req);
        spawn_socket_->send(zmq::buffer(data), zmq::send_flags::none);

        zmq::message_t reply;
        auto result = spawn_socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(reply.data()),
                                        static_cast<uint8_t *>(reply.data()) + reply.size());
            auto *resp = cista::deserialize<types::ser::Response>(buffer);
            if (resp && resp->success) {
                spawned_ = false;
                return true;
            }
        }
        return false;
    }

    void Agent::set_linear(float linear) { control_manager_.set_linear(linear); }

    void Agent::set_angular(float angular) { control_manager_.set_angular(angular); }

    void Agent::set_velocity(float linear, float angular) {
        control_manager_.set_linear(linear);
        control_manager_.set_angular(angular);
    }

    void Agent::tick(float dt, int timeout_ms) {
        if (!spawned_) {
            return;
        }

        // Update control manager (includes navigation update if enabled)
        control_manager_.tick(machine_.world_pose(), dt);

        // Get current control from control manager and send to simulator
        auto wheel_ctrl = control_manager_.get_wheel_control();
        auto ctrl_ser = types::ser::WheelControl::from_control(wheel_ctrl);
        auto ctrl_data = cista::serialize(ctrl_ser);
        control_socket_->send(zmq::buffer(ctrl_data), zmq::send_flags::dontwait);

        // Send heartbeat to simulator (non-blocking, fire-and-forget)
        static int tick_count = 0;
        if (++tick_count % 5 == 0) { // Send heartbeat every 30 ticks (~0.5s at 60Hz)
            types::ser::Request hb_req;
            hb_req.type = types::ser::MsgType::HEARTBEAT;
            hb_req.uuid = machine_.uuid();

            auto hb_data = cista::serialize(hb_req);
            heartbeat_socket_->send(zmq::buffer(hb_data), zmq::send_flags::dontwait);
        }

        // BLOCKING: Wait for state update from simulator
        state_socket_->set(zmq::sockopt::rcvtimeo, timeout_ms);
        zmq::message_t state_msg;
        auto result = state_socket_->recv(state_msg, zmq::recv_flags::none);

        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(state_msg.data()),
                                        static_cast<uint8_t *>(state_msg.data()) + state_msg.size());
            auto *ms = cista::deserialize<types::ser::MachineState>(buffer);
            if (ms && std::string(ms->uuid.view()) == machine_.uuid()) {
                // Update machine state from simulator
                machine_.update_state(*ms);

                // Call machine tick to process state update
                machine_.tick(dt);
            }
        }
    }

    void Agent::tock() {
        if (!spawned_) {
            return;
        }

        // Call machine tock for visualization
        machine_.tock();

        // Call control manager tock for tracker visualization
        control_manager_.tock(rec_);
    }

} // namespace agent
