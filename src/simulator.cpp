#include "flatsim/simulator.hpp"
#include <cista/serialization.h>
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

        // Setup physics world (no gravity for top-down 2D)
        muli::WorldSettings muli_settings;
        muli_settings.world_bounds =
            muli::AABB(muli::Vec2(-world_settings_.width / 2.0f, -world_settings_.height / 2.0f),
                       muli::Vec2(world_settings_.width / 2.0f, world_settings_.height / 2.0f));
        muli_settings.apply_gravity = false;
        muli_settings.gravity = muli::Vec2(0.0f, 0.0f);

        world_ = std::make_unique<muli::World>(muli_settings);
        std::cout << "[Simulator] Physics world created (" << world_settings_.width << "x" << world_settings_.height
                  << ")" << std::endl;
    }

    Simulator::~Simulator() {
        socket_->close();
        ctx_.close();
    }

    void Simulator::create_machine(const types::Machine &machine) {
        uint32_t group = machine.group > 0 ? machine.group : next_group_++;

        Machine m(machine);
        m.create(*world_, group);
        machines_[machine.uuid] = std::move(m);
    }

    void Simulator::apply_control(const types::MachineControl &control, float dt) {
        auto it = machines_.find(control.uuid);
        if (it == machines_.end()) {
            return;
        }
        it->second.apply_control(control, dt);
    }

    bool Simulator::destroy_machine(const std::string &uuid) {
        auto it = machines_.find(uuid);
        if (it == machines_.end()) {
            return false;
        }

        it->second.destroy(*world_);
        machines_.erase(it);
        return true;
    }

    types::ser::WorldState Simulator::get_world_state() const {
        types::ser::WorldState state;

        for (const auto &[uuid, machine] : machines_) {
            state.machines.push_back(machine.get_state());
        }

        return state;
    }

    void Simulator::tick(float dt) {
        // Apply wheel physics (friction, drag) for each machine
        for (auto &[uuid, machine] : machines_) {
            machine.apply_physics();
        }

        // Step physics
        world_->Step(dt);

        // Process ZMQ messages
        zmq::message_t request;
        auto result = socket_->recv(request, zmq::recv_flags::dontwait);
        if (result) {
            // Copy to aligned buffer for cista deserialization
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(request.data()),
                                        static_cast<uint8_t *>(request.data()) + request.size());

            auto *req = cista::deserialize<types::ser::Request>(buffer);
            if (!req) {
                types::ser::Response resp;
                resp.success = false;
                auto data = cista::serialize(resp);
                socket_->send(zmq::buffer(data), zmq::send_flags::none);
                return;
            }

            types::ser::Response resp;
            switch (req->type) {
            case types::ser::MsgType::SPAWN: {
                auto machine = req->machine.to_machine();
                create_machine(machine);
                resp.success = true;
                resp.state = get_world_state();
                break;
            }
            case types::ser::MsgType::GET_STATE: {
                resp.success = true;
                resp.state = get_world_state();
                break;
            }
            case types::ser::MsgType::DESPAWN: {
                std::string uuid_str(req->uuid.view());
                resp.success = destroy_machine(uuid_str);
                break;
            }
            case types::ser::MsgType::CONTROL: {
                auto control = req->control.to_control();
                apply_control(control, dt);
                resp.success = true;
                resp.state = get_world_state();
                break;
            }
            }

            auto data = cista::serialize(resp);
            socket_->send(zmq::buffer(data), zmq::send_flags::none);
        }
    }

    void Simulator::tock() {
        // Visualization updates go here (if needed)
    }

} // namespace simulator
