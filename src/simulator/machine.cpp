#include "flatsim/simulator/machine.hpp"
#include <cmath>
#include <iostream>

namespace simulator {

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world,
                     const types::Machine &config, uint32_t group)
        : rec_(rec), world_(world), config_(config) {
        // Create collision filter (use bit/mask system)
        filter_.group = 0;
        filter_.bit = 1 << group;
        filter_.mask = ~(1 << group);
    }

    void Machine::create() {
        if (!world_ || !rec_) {
            std::cerr << "[Simulator] Cannot create machine - missing world or recorder" << std::endl;
            return;
        }

        // Create chassis which manages all physics
        chassis_ = std::make_unique<Chassis>(world_, rec_, filter_, &config_, &state_);
        chassis_->init(config_);

        std::cout << "[Simulator] Created machine: " << config_.name << " with " << config_.wheels.size() << " wheels"
                  << std::endl;
    }

    void Machine::destroy() {
        if (chassis_ && chassis_->body && world_) {
            world_->Destroy(chassis_->body);
        }
        chassis_.reset();

        std::cout << "[Simulator] Destroyed machine: " << config_.uuid << std::endl;
    }

    void Machine::apply_control(const types::MachineControl &control, float dt) {
        if (!chassis_) return;

        // Apply brake if requested
        if (control.brake > 0.0f) {
            chassis_->brake(control.brake);
            return;
        }

        // Apply steering and throttle through chassis
        chassis_->update(control.steering, control.throttle, dt);
    }

    void Machine::tick(float dt) {
        if (!chassis_) return;

        // Update pose from physics
        config_.bound.pose = chassis_->get_pose();

        // Tick chassis (updates wheels, karosseries, hitches)
        chassis_->tick(dt);
    }

    void Machine::tock() {
        if (!chassis_) return;

        // Create label with role info
        std::string role_prefix;
        switch (config_.role) {
        case types::MachineRole::MASTER:
            role_prefix = "(M)";
            break;
        case types::MachineRole::FOLLOWER:
            role_prefix = "(F)";
            break;
        case types::MachineRole::SLAVE:
            role_prefix = "(S)";
            break;
        }
        std::string label = role_prefix + config_.seqid;

        chassis_->tock(label);
    }

    types::ser::MachineState Machine::get_state() const {
        types::ser::MachineState ms;
        ms.uuid = config_.uuid;

        if (chassis_ && chassis_->body) {
            ms.pose.position.x = chassis_->body->GetPosition().x;
            ms.pose.position.y = chassis_->body->GetPosition().y;
            ms.pose.angle = chassis_->body->GetAngle();
            ms.velocity.x = chassis_->body->GetLinearVelocity().x;
            ms.velocity.y = chassis_->body->GetLinearVelocity().y;
            ms.angular_vel = chassis_->body->GetAngularVelocity();
        }

        return ms;
    }

    Hitch *Machine::find_hitch(const std::string &name) {
        if (!chassis_) return nullptr;

        for (auto &hitch : chassis_->hitches) {
            if (hitch.name == name) {
                return &hitch;
            }
        }
        return nullptr;
    }

    void Machine::teleport(const concord::Pose &pose) {
        if (chassis_) {
            chassis_->teleport(pose);
        }
    }

    void Machine::brake(float brake_force) {
        if (chassis_) {
            chassis_->brake(brake_force);
        }
    }

    void Machine::update_color(const pigment::RGB &new_color) {
        config_.color = new_color;
        if (chassis_) {
            chassis_->update_color(new_color);
        }
    }

} // namespace simulator
