#include "flatsim/agent/power_manager.hpp"

namespace fs {

    void PowerManager::init(const std::string &name, Power::Type type, float capacity, float consumption_rate,
                            float charge_rate) {
        power = std::make_unique<Power>(name, type, capacity, consumption_rate, charge_rate);
    }

} // namespace fs
