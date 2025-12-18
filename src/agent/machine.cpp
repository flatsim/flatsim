#include "flatsim/agent/machine.hpp"

namespace agent {

    Machine::Machine(const types::Machine &config) : config_(config) {
        // Create wheel objects
        for (const auto &wheel_cfg : config_.wheels) {
            wheels_.emplace_back(wheel_cfg);
        }

        // Create karosserie objects
        for (const auto &karos_cfg : config_.karosseries) {
            karosseries_.emplace_back(karos_cfg);
        }

        // Create hitch objects
        for (const auto &hitch_cfg : config_.hitches) {
            hitches_.emplace_back(hitch_cfg);
        }

        // Initialize world pose from config
        world_pose_ = config_.pose;
    }

    void Machine::update_state(const types::ser::MachineState &state) {
        // Update machine pose
        world_pose_ = state.pose.to_concord();

        // Update wheel poses (world poses from simulator)
        for (size_t i = 0; i < state.wheels.size() && i < wheels_.size(); ++i) {
            wheels_[i].update_state(state.wheels[i]);
        }
    }

    Hitch *Machine::find_hitch(const std::string &name) {
        for (auto &hitch : hitches_) {
            if (hitch.config().name == name) {
                return &hitch;
            }
        }
        return nullptr;
    }

    void Machine::tick(float dt) {
        // Tick all wheels
        for (auto &wheel : wheels_) {
            wheel.tick(dt);
        }

        // Tick all karosseries
        for (auto &karosserie : karosseries_) {
            karosserie.tick(dt);
        }

        // Tick all hitches
        for (auto &hitch : hitches_) {
            hitch.tick(dt);
        }
    }

    void Machine::tock() {
        // Tock all wheels
        for (auto &wheel : wheels_) {
            wheel.tock();
        }

        // Tock all karosseries
        for (auto &karosserie : karosseries_) {
            karosserie.tock();
        }

        // Tock all hitches
        for (auto &hitch : hitches_) {
            hitch.tock();
        }
    }

} // namespace agent
