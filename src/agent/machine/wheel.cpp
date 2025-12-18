#include "flatsim/agent/machine/wheel.hpp"

namespace agent {

    Wheel::Wheel(const types::Wheel &config) : config_(config) {}

    void Wheel::update_state(const types::ser::WheelState &state) { world_pose_ = state.pose.to_concord(); }

} // namespace agent
