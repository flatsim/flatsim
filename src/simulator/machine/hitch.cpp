#include "flatsim/simulator/machine/hitch.hpp"
#include "flatsim/simulator/machine.hpp"

namespace simulator {

    Hitch::Hitch(const types::Hitch &config) : config_(config) {}

    bool Hitch::connect(muli::World &world, muli::RigidBody *this_body, Machine *other_machine,
                        const std::string &other_hitch_name) {
        if (!this_body || !other_machine || connected_machine_) {
            return false;
        }

        // Only master hitches can initiate connections
        if (!config_.is_master) {
            return false;
        }

        // Find the slave hitch on the other machine
        Hitch *other_hitch = other_machine->find_hitch(other_hitch_name);
        if (!other_hitch || other_hitch->config().is_master || other_hitch->is_connected()) {
            return false;
        }

        // Get connection point in world coordinates
        muli::Vec2 anchor;
        anchor.x = static_cast<float>(config_.pose.point.x);
        anchor.y = static_cast<float>(config_.pose.point.y);

        // Transform to world coordinates using the body's transform
        muli::Vec2 world_anchor = muli::Mul(this_body->GetTransform(), anchor);

        // Create revolute joint between the two bodies
        joint_ = world.CreateRevoluteJoint(this_body, other_machine->body(), world_anchor);
        if (joint_) {
            connected_machine_ = other_machine;
            return true;
        }

        return false;
    }

    void Hitch::disconnect(muli::World &world) {
        if (joint_) {
            world.Destroy(joint_);
            joint_ = nullptr;
            connected_machine_ = nullptr;
        }
    }

    void Hitch::tick(float dt) {
        // Future: Monitor joint health, forces, etc.
    }

    void Hitch::tock() {
        // Future: Visualization, debug rendering
    }

} // namespace simulator
