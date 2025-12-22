#pragma once

#include "flatsim/agent/sensor/sensor.hpp"
#include <memory>
#include <string>
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
        std::string robot_uuid;

      public:
        SensorManager() = default;
        ~SensorManager() = default;

        // Prevent copying, allow moving
        SensorManager(const SensorManager &) = delete;
        SensorManager &operator=(const SensorManager &) = delete;
        SensorManager(SensorManager &&) = default;
        SensorManager &operator=(SensorManager &&) = default;

        /**
         * @brief Set the robot UUID (must be called before adding sensors for auto-FIFO)
         * @param uuid Robot UUID
         */
        void set_robot_uuid(const std::string &uuid) { robot_uuid = uuid; }

        /**
         * @brief Add a sensor to the manager (auto-enables FIFO if robot_uuid is set)
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
         * @brief Update all sensors with robot pose and physics data
         * @param pose Current robot pose
         * @param linear_vel_x Linear velocity X (world frame)
         * @param linear_vel_y Linear velocity Y (world frame)
         * @param angular_vel Angular velocity (rad/s)
         * @param dt Time delta in seconds
         */
        void update_all_with_physics(const concord::Pose &pose, double linear_vel_x, double linear_vel_y,
                                     double angular_vel, double dt);

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

        /**
         * @brief Enable FIFO output for all sensors
         * @param robot_uuid Robot UUID for creating directory structure
         * @return true if all FIFOs were created successfully
         */
        bool enable_fifo_output(const std::string &robot_uuid);

        /**
         * @brief Disable FIFO output for all sensors
         */
        void disable_fifo_output();
    };

} // namespace fs
