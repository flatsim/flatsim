#include "flatsim/simulator.hpp"
#include <iostream>
#include <vector>

namespace simulator {

    Simulator::Simulator(Conn conn, const std::string &address, const WorldSettings &settings)
        : ctx_(1), conn_(conn), address_(address), world_settings_(settings) {

        // Setup ZMQ
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

        // Setup physics world
        muli::WorldSettings muli_settings;
        muli_settings.world_bounds =
            muli::AABB(muli::Vec2(-world_settings_.width / 2.0f, -world_settings_.height / 2.0f),
                       muli::Vec2(world_settings_.width / 2.0f, world_settings_.height / 2.0f));

        world_ = std::make_unique<muli::World>(muli_settings);
        std::cout << "[Simulator] Physics world created (" << world_settings_.width << "x" << world_settings_.height
                  << ")" << std::endl;
    }

    Simulator::~Simulator() {
        socket_->close();
        ctx_.close();
    }

    muli::RigidBody *Simulator::create_body(const types::Chassis &chassis) {
        muli::Transform t;
        t.position.x = static_cast<float>(chassis.pose.point.x);
        t.position.y = static_cast<float>(chassis.pose.point.y);
        t.rotation = static_cast<float>(chassis.pose.angle.yaw);

        auto *body = world_->CreateBox(static_cast<float>(chassis.size.x), static_cast<float>(chassis.size.y), t);
        bodies_[chassis.uuid] = body;

        std::cout << "[Simulator] Created body for: " << chassis.name << std::endl;
        return body;
    }

    void Simulator::tick(float dt) {
        // Step physics
        world_->Step(dt);

        // Process ZMQ messages
        zmq::message_t request;
        auto result = socket_->recv(request, zmq::recv_flags::dontwait);
        if (result) {
            // Copy to aligned buffer for cista deserialization
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(request.data()),
                                        static_cast<uint8_t *>(request.data()) + request.size());

            // Deserialize to ser::Chassis then convert to Chassis
            auto *ser_chassis = cista::deserialize<types::ser::Chassis>(buffer);
            if (ser_chassis) {
                auto chassis = ser_chassis->to_chassis();
                create_body(chassis);
                socket_->send(zmq::buffer("OK"), zmq::send_flags::none);
            } else {
                socket_->send(zmq::buffer("ERROR"), zmq::send_flags::none);
            }
        }
    }

    void Simulator::tock() {
        // Visualization updates go here
    }

} // namespace simulator
