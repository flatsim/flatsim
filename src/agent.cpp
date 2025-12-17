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

    void Agent::tick(float dt) {
        (void)dt;

        std::string msg = "Hello from Agent!";
        socket_->send(zmq::buffer(msg), zmq::send_flags::none);

        zmq::message_t reply;
        auto result = socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::string response(static_cast<char *>(reply.data()), reply.size());
            std::cout << "[Agent] Received: " << response << std::endl;
        }
    }

    void Agent::tock() {}

} // namespace agent
