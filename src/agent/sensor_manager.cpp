#include "flatsim/agent/sensor_manager.hpp"

namespace fs {

    void SensorManager::add(std::unique_ptr<Sensor> sensor) {
        if (sensor) {
            // Auto-enable shared memory output if robot UUID is set
            if (!robot_uuid.empty()) {
                sensor->enable_shm_output(robot_uuid);
            }
            Sensor *raw = sensor.get();
            sensors.push_back(std::move(sensor));
            if (raw && on_add_) {
                on_add_(*raw);
            }
        }
    }

    Sensor *SensorManager::get(const std::string &type) const {
        for (const auto &sensor : sensors) {
            if (!sensor) continue;
            if (sensor->get_type() == type) {
                return sensor.get();
            }
        }
        return nullptr;
    }

    void SensorManager::update_all(const datapod::Pose &pose, double dt) {
        for (auto &sensor : sensors) {
            if (sensor) {
                sensor->set_robot_pose(pose);
                sensor->update(dt);
            }
        }
    }

    void SensorManager::update_all_with_physics(const datapod::Pose &pose, double linear_vel_x, double linear_vel_y,
                                                double angular_vel, double dt) {
        for (auto &sensor : sensors) {
            if (sensor) {
                sensor->set_robot_pose(pose);
                sensor->set_physics_data(linear_vel_x, linear_vel_y, angular_vel);
                sensor->update(dt);
            }
        }
    }

    void SensorManager::update_from_simulator(const types::SensorData &data, double dt) {
        for (auto &sensor : sensors) {
            if (sensor) {
                // Pass combined sensor data to each sensor
                // Each sensor extracts what it needs (GPS sensor gets GPS data, etc.)
                sensor->update_from_simulator(data);
                sensor->update(dt);
            }
        }
    }

    bool SensorManager::enable_fifo_output(const std::string &uuid) {
        robot_uuid = uuid;
        bool all_success = true;
        for (auto &sensor : sensors) {
            if (sensor) {
                if (!sensor->enable_shm_output(robot_uuid)) {
                    all_success = false;
                }
            }
        }
        return all_success;
    }

    void SensorManager::disable_fifo_output() {
        for (auto &sensor : sensors) {
            if (sensor) {
                sensor->disable_shm_output();
            }
        }
    }

} // namespace fs
