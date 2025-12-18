#pragma once

#include <string>
#include <vector>

#include "flatsim/simulator/machine/hitch.hpp"
#include "flatsim/simulator/machine/karosserie.hpp"
#include "flatsim/simulator/machine/wheel.hpp"
#include "flatsim/types.hpp"
#include "muli/world.h"

namespace simulator {

    class Machine {
      private:
        muli::RigidBody *body_ = nullptr;
        std::vector<Wheel> wheels_;
        std::vector<Karosserie> karosseries_;
        std::vector<Hitch> hitches_;
        types::Machine config_;
        muli::CollisionFilter filter_;

        // Helper to compute shifted pose
        static concord::Pose shift_pose(const concord::Pose &parent, const concord::Pose &child);

      public:
        Machine() = default;
        Machine(const types::Machine &config);

        // Create physics objects in world
        void create(muli::World &world, uint32_t group);

        // Destroy physics objects
        void destroy(muli::World &world);

        // Apply control inputs
        void apply_control(const types::MachineControl &control, float dt);

        // Apply physics (friction, drag) - called each tick
        void apply_physics();

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Get state for feedback
        types::ser::MachineState get_state() const;

        // Find hitch by name
        Hitch *find_hitch(const std::string &name);

        // Accessors
        muli::RigidBody *body() const { return body_; }
        const types::Machine &config() const { return config_; }
        const std::string &uuid() const { return config_.uuid; }
        std::vector<Wheel> &wheels() { return wheels_; }
        std::vector<Karosserie> &karosseries() { return karosseries_; }
        std::vector<Hitch> &hitches() { return hitches_; }
    };

} // namespace simulator
