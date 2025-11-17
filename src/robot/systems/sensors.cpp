#include "flatsim/robot/systems/sensors.hpp"

namespace fs {

    void SensorManager::add(std::unique_ptr<Sensor> sensor) { sensors.push_back(std::move(sensor)); }

    Sensor *SensorManager::get(const std::string &type) const {
        for (const auto &sensor : sensors) {
            if (!sensor) continue;
            if (sensor->get_type() == type) {
                return sensor.get();
            }
        }
        return nullptr;
    }

    void SensorManager::update_all(const concord::Pose &pose, double dt) {
        for (auto &sensor : sensors) {
            if (sensor) {
                sensor->set_robot_pose(pose);
                sensor->update(dt);
            }
        }
    }

} // namespace fs
