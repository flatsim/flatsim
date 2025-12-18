#include "flatsim/agent/machine/karosserie.hpp"

namespace agent {

    Karosserie::Karosserie(const types::Karosserie &config) : config_(config) {}

    void Karosserie::tick(float dt) {
        // Future: Process sensor data, etc.
    }

    void Karosserie::tock() {
        // Future: Visualization, debug rendering
    }

} // namespace agent
