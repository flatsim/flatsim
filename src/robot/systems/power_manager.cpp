#include "flatsim/robot/systems/power_manager.hpp"

namespace fs {

    void PowerManager::init(const std::string &name, Power::Type type, float capacity, float consumption_rate,
                            float charge_rate) {
        power = std::make_unique<Power>(name, type, capacity, consumption_rate, charge_rate);
    }

    void PowerManager::init(const PowerInfo &power_info) {
        auto power_type = (power_info.type == PowerType::BATTERY) ? Power::Type::BATTERY : Power::Type::FUEL;
        power = std::make_unique<Power>(power_info.name, power_type, power_info.capacity, power_info.consumption_rate,
                                        power_info.charge_rate);
    }

} // namespace fs
