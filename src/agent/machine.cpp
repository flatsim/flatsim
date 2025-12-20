#include "flatsim/agent/machine.hpp"

namespace agent {

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, const types::Machine &config)
        : rec_(rec), config_(config) {
        world_pose_ = config_.bound.pose;
    }

    void Machine::update_state(const types::ser::MachineState &state) { world_pose_ = state.pose.to_concord(); }

    void Machine::tick(float dt) {
        (void)dt;
        // Tick is called after receiving state update from simulator
        // Local agent-side processing can go here (e.g., sensor fusion, prediction)
    }

    void Machine::tock() {
        // Placeholder for future visualization
        // Can be used to log pose, visualize in rerun, etc.
    }

} // namespace agent
