#include "flatsim/robot.hpp"
#include <iostream>

namespace fs::robot {

    Robot::Robot(const std::string &address) : ctx_(1), address_(address) {
        socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::req);

        std::string addr = address_.empty() ? "ipc:///tmp/flatsim" : address_;
        socket_->connect(addr);
        std::cout << "[Robot] Connected to " << addr << std::endl;
    }

    Robot::~Robot() {
        socket_->close();
        ctx_.close();
    }

    void Robot::tick(float dt) {
        (void)dt;

        std::string msg = "Hello from Robot!";
        socket_->send(zmq::buffer(msg), zmq::send_flags::none);

        zmq::message_t reply;
        auto result = socket_->recv(reply, zmq::recv_flags::none);
        if (result) {
            std::string response(static_cast<char *>(reply.data()), reply.size());
            std::cout << "[Robot] Received: " << response << std::endl;
        }
    }

    void Robot::tock() {}

} // namespace fs::robot
