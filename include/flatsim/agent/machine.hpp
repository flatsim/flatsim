#pragma once

#include <memory>
#include <rerun.hpp>
#include <string>
#include <vector>

#include "flatsim/agent/machine/hitch.hpp"
#include "flatsim/agent/machine/karosserie.hpp"
#include "flatsim/agent/machine/wheel.hpp"
#include "flatsim/types.hpp"

namespace agent {

    class Machine {
      private:
        std::vector<Wheel> wheels_;
        std::vector<Karosserie> karosseries_;
        std::vector<Hitch> hitches_;
        types::Machine config_;
        concord::Pose world_pose_; // Updated from simulator state

        // Rerun visualization
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        Machine() = default;
        Machine(const types::Machine &config, std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Update state from simulator feedback
        void update_state(const types::ser::MachineState &state);

        // Find hitch by name
        Hitch *find_hitch(const std::string &name);

        // Set rerun for visualization
        void set_rerun(std::shared_ptr<rerun::RecordingStream> rec);

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Accessors
        const types::Machine &config() const { return config_; }
        const std::string &uuid() const { return config_.uuid; }
        const concord::Pose &world_pose() const { return world_pose_; }
        std::vector<Wheel> &wheels() { return wheels_; }
        std::vector<Karosserie> &karosseries() { return karosseries_; }
        std::vector<Hitch> &hitches() { return hitches_; }
        const std::vector<Wheel> &wheels() const { return wheels_; }
        const std::vector<Karosserie> &karosseries() const { return karosseries_; }
        const std::vector<Hitch> &hitches() const { return hitches_; }
    };

} // namespace agent
