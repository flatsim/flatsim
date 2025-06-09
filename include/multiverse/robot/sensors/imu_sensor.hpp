#pragma once

#include "multiverse/robot/sensor.hpp"
#include "multiverse/types.hpp"
#include <chrono>

namespace mvs {
    /**
     * @brief IMU sensor data structure
     * 
     * Contains acceleration, angular velocity, and orientation data
     */
    struct IMUData {
        // Linear acceleration (m/s²) in body frame
        double accel_x = 0.0;
        double accel_y = 0.0; 
        double accel_z = 0.0;
        
        // Angular velocity (rad/s) in body frame
        double gyro_x = 0.0;
        double gyro_y = 0.0;
        double gyro_z = 0.0;
        
        // Magnetic field (μT) in body frame
        double mag_x = 0.0;
        double mag_y = 0.0;
        double mag_z = 0.0;
        
        // Orientation (quaternion) - world to body frame
        double quat_w = 1.0;
        double quat_x = 0.0;
        double quat_y = 0.0;
        double quat_z = 0.0;
        
        // Euler angles (radians) - roll, pitch, yaw
        double roll = 0.0;   // Rotation around x-axis
        double pitch = 0.0;  // Rotation around y-axis  
        double yaw = 0.0;    // Rotation around z-axis
        
        // Sensor status and accuracy
        enum class CalibrationStatus {
            UNCALIBRATED = 0,
            PARTIALLY_CALIBRATED = 1,
            MOSTLY_CALIBRATED = 2,
            FULLY_CALIBRATED = 3
        };
        
        CalibrationStatus accel_cal = CalibrationStatus::UNCALIBRATED;
        CalibrationStatus gyro_cal = CalibrationStatus::UNCALIBRATED;
        CalibrationStatus mag_cal = CalibrationStatus::UNCALIBRATED;
        
        // Temperature (°C) - affects sensor performance
        double temperature = 25.0;
        
        // Time information
        std::chrono::system_clock::time_point timestamp;
        
        // Constructor
        IMUData() : timestamp(std::chrono::system_clock::now()) {}
    };

    /**
     * @brief IMU sensor with 9-DOF capabilities
     * 
     * Simulates a high-quality IMU with accelerometer, gyroscope, and magnetometer.
     * Provides orientation estimation and sensor fusion capabilities.
     */
    class IMUSensor : public Sensor {
    private:
        IMUData current_data;
        double update_frequency;    // Hz
        double next_update_time;
        concord::Pose robot_pose;   // Current robot pose
        concord::Pose last_pose;    // Previous robot pose for velocity calculation
        
        // Sensor parameters
        double accel_noise_std;     // Accelerometer noise (m/s²)
        double gyro_noise_std;      // Gyroscope noise (rad/s)
        double mag_noise_std;       // Magnetometer noise (μT)
        double accel_bias_std;      // Accelerometer bias drift
        double gyro_bias_std;       // Gyroscope bias drift
        
        // Current sensor biases
        double accel_bias_x, accel_bias_y, accel_bias_z;
        double gyro_bias_x, gyro_bias_y, gyro_bias_z;
        
        // Calibration parameters
        bool auto_calibrate;
        int calibration_samples;
        int max_calibration_samples;
        
        // Earth's magnetic field parameters (for simulation)
        double magnetic_declination;   // Local magnetic declination (radians)
        double magnetic_inclination;   // Local magnetic inclination (radians)
        double magnetic_intensity;     // Local magnetic field intensity (μT)
        
        // Internal state for integration
        double last_update_real_time;
        
    public:
        /**
         * @brief Construct a new IMU Sensor
         * @param frequency Update frequency in Hz (default: 100 Hz)
         * @param accel_noise Accelerometer noise std dev (default: 0.01 m/s²)
         * @param gyro_noise Gyroscope noise std dev (default: 0.001 rad/s)
         * @param mag_noise Magnetometer noise std dev (default: 0.1 μT)
         */
        IMUSensor(double frequency = 100.0, 
                 double accel_noise = 0.01, 
                 double gyro_noise = 0.001,
                 double mag_noise = 0.1);
        
        virtual ~IMUSensor() = default;
        
        // Sensor interface implementation
        void update(double dt) override;
        void set_robot_pose(const concord::Pose& pose) override;
        void* get_data() override;
        std::string get_type() const override;
        bool is_data_valid() const override;
        double get_frequency() const override;
        
        // IMU-specific methods
        
        /**
         * @brief Get the current IMU data
         * @return Current IMU data structure
         */
        const IMUData& get_imu_data() const;
        
        /**
         * @brief Enable or disable auto-calibration
         * @param enable true to enable auto-calibration
         */
        void set_auto_calibration(bool enable);
        
        /**
         * @brief Start manual calibration procedure
         * @param samples Number of samples to collect for calibration
         */
        void start_calibration(int samples = 1000);
        
        /**
         * @brief Check if sensor is currently calibrating
         * @return true if calibration is in progress
         */
        bool is_calibrating() const;
        
        /**
         * @brief Get overall calibration status
         * @return Worst calibration status among all sensors
         */
        IMUData::CalibrationStatus get_calibration_status() const;
        
        /**
         * @brief Set local magnetic field parameters
         * @param declination Magnetic declination in radians
         * @param inclination Magnetic inclination in radians  
         * @param intensity Magnetic field intensity in μT
         */
        void set_magnetic_field(double declination, double inclination, double intensity);
        
        /**
         * @brief Configure noise parameters
         * @param accel_noise Accelerometer noise std dev (m/s²)
         * @param gyro_noise Gyroscope noise std dev (rad/s)
         * @param mag_noise Magnetometer noise std dev (μT)
         */
        void configure_noise(double accel_noise, double gyro_noise, double mag_noise);
        
        /**
         * @brief Reset sensor biases and calibration
         */
        void reset_calibration();

    private:
        /**
         * @brief Calculate accelerations from robot motion
         */
        void calculate_accelerations(double dt);
        
        /**
         * @brief Calculate angular velocities from robot motion
         */
        void calculate_angular_velocities(double dt);
        
        /**
         * @brief Calculate magnetic field readings
         */
        void calculate_magnetic_field();
        
        /**
         * @brief Update orientation from accelerometer and magnetometer
         */
        void update_orientation();
        
        /**
         * @brief Convert quaternion to Euler angles
         */
        void quaternion_to_euler();
        
        /**
         * @brief Add realistic sensor noise
         */
        void add_sensor_noise();
        
        /**
         * @brief Update sensor biases (drift simulation)
         */
        void update_biases(double dt);
        
        /**
         * @brief Update calibration status based on conditions
         */
        void update_calibration_status();
        
        /**
         * @brief Apply sensor calibration corrections
         */
        void apply_calibration();
        
        /**
         * @brief Normalize quaternion
         */
        void normalize_quaternion();
    };
} // namespace mvs
