#include "flatsim/agent/machine.hpp"

namespace agent {

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, const types::Machine &config)
        : rec_(rec), config_(config) {
        world_pose_ = config_.bound.pose;
    }

    void Machine::update_state(const types::ser::MachineState &state) { world_pose_ = state.pose.to_concord(); }

    void Machine::tick(float dt) {
        // Future: local logic if needed
        (void)dt;
    }

    void Machine::tock() {
        // Future: visualization if needed
    }

} // namespace agent
