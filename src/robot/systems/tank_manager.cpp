#include "flatsim/robot/systems/tank_manager.hpp"

namespace fs {

    void TankManager::init(const std::string &name, Tank::Type type, float capacity, float current_level,
                           float fill_rate) {
        tank = Tank(name, type, capacity, current_level, fill_rate);
    }

    void TankManager::init(const TankInfo &tank_info, const pigment::RGB &color, const std::string &seqid) {
        tank = Tank(tank_info.name, Tank::Type::HARVEST, tank_info.capacity, 0.0f, 0.0f);
        if (tank) {
            tank->init(color, seqid, tank_info.bound);
        }
    }

} // namespace fs
