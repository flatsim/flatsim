#pragma once

#include "concord/concord.hpp"
#include <fstream>
#include <memory>
#include <string>

namespace fs {
    /**
     * @brief Abstract base class for all sensors
     *
     * This virtual class defines the interface that all sensors must implement.
     * Each sensor provides periodic updates and can be queried for its current data.
     */
    class Sensor {
      public:
        virtual ~Sensor() = default;

        /**
         * @brief Update the sensor with the given time delta
         * @param dt Time delta in seconds since last update
         */
        virtual void update(double dt) = 0;

        /**
         * @brief Set the current robot pose for sensors that need position information
         * @param pose Current pose of the robot carrying this sensor
         */
        virtual void set_robot_pose(const concord::Pose &pose) {};

        /**
         * @brief Set physics data for sensors that need velocity/acceleration
         * @param linear_vel Linear velocity in world frame (x, y)
         * @param angular_vel Angular velocity (rad/s)
         */
        virtual void set_physics_data(double linear_vel_x, double linear_vel_y, double angular_vel) {};

        /**
         * @brief Get the current sensor data
         * @return Sensor-specific data structure
         */
        virtual void *get_data() = 0;

        /**
         * @brief Get the sensor type name
         * @return String identifier for the sensor type
         */
        virtual std::string get_type() const = 0;

        /**
         * @brief Check if the sensor has valid data
         * @return true if the sensor has valid data, false otherwise
         */
        virtual bool is_data_valid() const = 0;

        /**
         * @brief Get the sensor's update frequency in Hz
         * @return Update frequency in Hz
         */
        virtual double get_frequency() const = 0;

        /**
         * @brief Enable shared memory output
         * @param robot_uuid Robot UUID for naming shared memory segment
         * @return true if shared memory was created successfully
         */
        bool enable_shm_output(const std::string &robot_uuid);

        /**
         * @brief Disable shared memory output
         */
        void disable_shm_output();

        /**
         * @brief Check if shared memory output is enabled
         * @return true if shared memory output is enabled
         */
        bool is_shm_enabled() const { return shm_enabled; }

      protected:
        double last_update_time = 0.0;
        bool data_valid = false;

        // Shared memory output (single frame, overwritten each update)
        bool shm_enabled = false;
        std::string shm_name;
        std::string metadata_path;
        int shm_fd = -1;
        void *shm_ptr = nullptr;
        size_t shm_size = 0;
        uint64_t sequence_number = 0;

        // Maximum shared memory size per sensor
        static constexpr size_t MAX_SHM_SIZE = 2 * 1024 * 1024; // 2MB max

        /**
         * @brief Write data to shared memory (must be implemented by derived classes)
         * @return true if data was written successfully
         */
        virtual bool write_to_shm() { return false; }

        /**
         * @brief Get metadata describing binary format (must be implemented by derived classes)
         * @return String describing the binary serialization format
         */
        virtual std::string get_metadata() const { return ""; }

        /**
         * @brief Helper to write binary data to shared memory ring buffer
         * @param data Pointer to binary data
         * @param size Size of data in bytes
         * @return true if write was successful
         */
        bool write_shm_data(const void *data, size_t size);

        /**
         * @brief Helper to create/update metadata file
         * @return true if metadata file was written successfully
         */
        bool update_metadata_file();
    };
} // namespace fs
