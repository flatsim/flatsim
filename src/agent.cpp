#include "flatsim/agent.hpp"
#include <iostream>

namespace agent {

    Agent::Agent(const std::string &address, std::shared_ptr<rerun::RecordingStream> rec)
        : ctx_(1), address_(address), rec_(rec) {
        // Create spawn socket (REQ)
        spawn_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::req);
        std::string spawn_addr = address_.empty() ? "ipc:///tmp/flatsim_spawn" : address_;
        spawn_socket_->connect(spawn_addr);
        std::cout << "[Agent] Connected to spawn socket: " << spawn_addr << std::endl;

        // Create control and state sockets (will connect after spawn)
        control_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::push);
        state_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::sub);
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
        ctx_.close();
    }

    void Agent::set_machine(const types::Machine &config) { machine_ = Machine(config, rec_); }

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
                std::string ctrl_addr =
                    address_.empty() ? "ipc:///tmp/flatsim_ctrl_" + uuid : "tcp://" + address_ + ":5600";
                std::string state_addr =
                    address_.empty() ? "ipc:///tmp/flatsim_state_" + uuid : "tcp://" + address_ + ":5601";

                control_socket_->connect(ctrl_addr);
                state_socket_->connect(state_addr);
                std::cout << "[Agent] Connected to control: " << ctrl_addr << std::endl;
                std::cout << "[Agent] Connected to state: " << state_addr << std::endl;

                // Update state from response
                for (const auto &ms : resp->state.machines) {
                    if (std::string(ms.uuid.view()) == machine_.uuid()) {
                        machine_.update_state(ms);
                        break;
                    }
                }

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

    bool Agent::control(const types::MachineControl &ctrl) {
        if (!spawned_) {
            return false;
        }

        // Serialize and send control (PUSH socket - fire and forget)
        auto ctrl_ser = types::ser::MachineControl::from_control(ctrl);
        auto data = cista::serialize(ctrl_ser);
        control_socket_->send(zmq::buffer(data), zmq::send_flags::dontwait);

        // Receive state update (SUB socket - non-blocking)
        zmq::message_t state_msg;
        auto result = state_socket_->recv(state_msg, zmq::recv_flags::dontwait);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(state_msg.data()),
                                        static_cast<uint8_t *>(state_msg.data()) + state_msg.size());
            auto *ms = cista::deserialize<types::ser::MachineState>(buffer);
            if (ms && std::string(ms->uuid.view()) == machine_.uuid()) {
                machine_.update_state(*ms);
            }
        }

        return true;
    }

    void Agent::tick(float dt) {
        (void)dt;

        // Receive latest state update
        if (spawned_) {
            zmq::message_t state_msg;
            auto result = state_socket_->recv(state_msg, zmq::recv_flags::dontwait);
            if (result) {
                std::vector<uint8_t> buffer(static_cast<uint8_t *>(state_msg.data()),
                                            static_cast<uint8_t *>(state_msg.data()) + state_msg.size());
                auto *ms = cista::deserialize<types::ser::MachineState>(buffer);
                if (ms && std::string(ms->uuid.view()) == machine_.uuid()) {
                    machine_.update_state(*ms);
                }
            }
        }
    }

    void Agent::tock() {
        // Call machine tock for visualization
        machine_.tock();
    }

} // namespace agent
