#pragma once

#include "flatsim/robot/sensor.hpp"
#include "flatsim/robot/sensors/gps_sensor.hpp"
#include "flatsim/robot/sensors/imu_sensor.hpp"
#include "flatsim/robot/sensors/lidar_sensor.hpp"

// This header includes all available sensors for easy access
// Add new sensor includes here as you create them

namespace fs {
    namespace sensors {
        // Factory function to create sensors by type
        template<typename SensorType, typename... Args>
        std::unique_ptr<Sensor> create_sensor(Args&&... args) {
            return std::make_unique<SensorType>(std::forward<Args>(args)...);
        }
    }
}
