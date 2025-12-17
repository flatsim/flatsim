#include "flatsim/simulator.hpp"
#include <iostream>

namespace fs::simulator {

    Simulator::Simulator(Conn conn, const std::string &address) : ctx_(1), conn_(conn), address_(address) {
        socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::rep);

        if (conn_ == Conn::IPC) {
            std::string addr = address_.empty() ? "ipc:///tmp/flatsim" : address_;
            socket_->bind(addr);
            std::cout << "[Simulator] Listening on " << addr << std::endl;
        } else {
            std::string addr = address_.empty() ? "tcp://*:5555" : address_;
            socket_->bind(addr);
            std::cout << "[Simulator] Listening on " << addr << std::endl;
        }

        socket_->set(zmq::sockopt::rcvtimeo, 0);
    }

    Simulator::~Simulator() {
        socket_->close();
        ctx_.close();
    }

    void Simulator::tick(float dt) {
        (void)dt;

        zmq::message_t request;
        auto result = socket_->recv(request, zmq::recv_flags::dontwait);
        if (result) {
            std::string msg(static_cast<char *>(request.data()), request.size());
            std::cout << "[Simulator] Received: " << msg << std::endl;

            std::string reply = "Hello from Simulator!";
            socket_->send(zmq::buffer(reply), zmq::send_flags::none);
        }
    }

    void Simulator::tock() {}

} // namespace fs::simulator
