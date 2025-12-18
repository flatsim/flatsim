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

    void Agent::set_chassis(const types::Chassis &chassis) { chassis_ = chassis; }

    bool Agent::spawn() {
        // Convert to serializable and serialize with cista
        auto ser_chassis = types::ser::Chassis::from_chassis(chassis_);
        auto data = cista::serialize(ser_chassis);

        // Send serialized data
        socket_->send(zmq::buffer(data), zmq::send_flags::none);
        std::cout << "[Agent] Sent chassis: " << chassis_.name << std::endl;

        // Wait for reply
        zmq::message_t reply;
        auto result = socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::string response(static_cast<char *>(reply.data()), reply.size());
            std::cout << "[Agent] Received: " << response << std::endl;
            return response.find("OK") == 0;
        }
        return false;
    }

    void Agent::tick(float dt) { (void)dt; }

    void Agent::tock() {}

} // namespace agent
