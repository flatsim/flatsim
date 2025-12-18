#include "flatsim/agent.hpp"
#include <iostream>

namespace agent {

    Agent::Agent(const std::string &address) : ctx_(1), address_(address) {
        socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::req);

        std::string addr = address_.empty() ? "ipc:///tmp/flatsim" : address_;
        socket_->connect(addr);
        std::cout << "[Agent] Connected to " << addr << std::endl;
    }

    Agent::~Agent() {
        socket_->close();
        ctx_.close();
    }

    void Agent::set_machine(const types::Machine &config) { machine_ = Machine(config); }

    bool Agent::spawn() {
        types::ser::Request req;
        req.type = types::ser::MsgType::SPAWN;
        req.machine = types::ser::Machine::from_machine(machine_.config());

        auto data = cista::serialize(req);
        socket_->send(zmq::buffer(data), zmq::send_flags::none);
        std::cout << "[Agent] Sent SPAWN for: " << machine_.config().name << std::endl;

        zmq::message_t reply;
        auto result = socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(reply.data()),
                                        static_cast<uint8_t *>(reply.data()) + reply.size());
            auto *resp = cista::deserialize<types::ser::Response>(buffer);
            if (resp && resp->success) {
                // Update state from response
                for (const auto &ms : resp->state.machines) {
                    if (std::string(ms.uuid.view()) == machine_.uuid()) {
                        machine_.update_state(ms);
                        break;
                    }
                }
                std::cout << "[Agent] Spawn successful" << std::endl;
                return true;
            }
        }
        std::cout << "[Agent] Spawn failed" << std::endl;
        return false;
    }

    bool Agent::despawn() {
        types::ser::Request req;
        req.type = types::ser::MsgType::DESPAWN;
        req.uuid = machine_.uuid();

        auto data = cista::serialize(req);
        socket_->send(zmq::buffer(data), zmq::send_flags::none);

        zmq::message_t reply;
        auto result = socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(reply.data()),
                                        static_cast<uint8_t *>(reply.data()) + reply.size());
            auto *resp = cista::deserialize<types::ser::Response>(buffer);
            return resp && resp->success;
        }
        return false;
    }

    bool Agent::control(const types::MachineControl &ctrl) {
        types::ser::Request req;
        req.type = types::ser::MsgType::CONTROL;
        req.control = types::ser::MachineControl::from_control(ctrl);

        auto data = cista::serialize(req);
        socket_->send(zmq::buffer(data), zmq::send_flags::none);

        zmq::message_t reply;
        auto result = socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(reply.data()),
                                        static_cast<uint8_t *>(reply.data()) + reply.size());
            auto *resp = cista::deserialize<types::ser::Response>(buffer);
            if (resp && resp->success) {
                for (const auto &ms : resp->state.machines) {
                    if (std::string(ms.uuid.view()) == machine_.uuid()) {
                        machine_.update_state(ms);
                        break;
                    }
                }
                return true;
            }
        }
        return false;
    }

    void Agent::tick(float dt) { (void)dt; }

    void Agent::tock() {
        // Visualization goes here
    }

} // namespace agent
