#pragma once

#include "flatsim/robot/sensor/sensor.hpp"
#include "flatsim/types.hpp"
#include <memory>
#include <vector>

namespace fs {

    /**
     * @brief Manager for all sensors attached to a robot
     *
     * Provides unified interface for sensor management including:
     * - Adding/removing sensors
     * - Type-safe sensor lookup
     * - Batch updates
     * - Lifecycle management
     */
    class SensorManager {
      private:
        std::vector<std::unique_ptr<Sensor>> sensors;

      public:
        SensorManager() = default;
        ~SensorManager() = default;

        // Prevent copying, allow moving
        SensorManager(const SensorManager &) = delete;
        SensorManager &operator=(const SensorManager &) = delete;
        SensorManager(SensorManager &&) = default;
        SensorManager &operator=(SensorManager &&) = default;

        /**
         * @brief Add a sensor to the manager
         * @param sensor Unique pointer to sensor
         */
        void add(std::unique_ptr<Sensor> sensor);

        /**
         * @brief Get sensor by type (template version)
         * @tparam T Sensor type to retrieve
         * @return Pointer to sensor or nullptr if not found
         */
        template <typename T> T *get() const {
            for (const auto &sensor : sensors) {
                if (!sensor) continue;
                T *typed_sensor = dynamic_cast<T *>(sensor.get());
                if (typed_sensor) return typed_sensor;
            }
            return nullptr;
        }

        /**
         * @brief Get sensor by type string
         * @param type Type string identifier
         * @return Pointer to sensor or nullptr if not found
         */
        Sensor *get(const std::string &type) const;

        /**
         * @brief Update all sensors with robot pose
         * @param pose Current robot pose
         * @param dt Time delta in seconds
         */
        void update_all(const concord::Pose &pose, double dt);

        /**
         * @brief Check if manager has any sensors
         * @return true if sensors exist
         */
        bool empty() const { return sensors.empty(); }

        /**
         * @brief Get number of sensors
         * @return Sensor count
         */
        size_t count() const { return sensors.size(); }

        /**
         * @brief Clear all sensors
         */
        void clear() { sensors.clear(); }
    };

} // namespace fs
