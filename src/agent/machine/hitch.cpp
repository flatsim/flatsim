#include "flatsim/agent/machine/hitch.hpp"

namespace agent {

    Hitch::Hitch(const types::Hitch &config) : config_(config) {}

    void Hitch::tick(float dt) {
        // Future: Monitor connection status, etc.
    }

    void Hitch::tock() {
        // Future: Visualization, debug rendering
    }

} // namespace agent
