#pragma once

#include "multiverse/types.hpp"
#include <memory>

namespace mvs {
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
        virtual void set_robot_pose(const concord::Pose& pose) {};
        
        /**
         * @brief Get the current sensor data
         * @return Sensor-specific data structure
         */
        virtual void* get_data() = 0;
        
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

    protected:
        double last_update_time = 0.0;
        bool data_valid = false;
    };
} // namespace mvs
