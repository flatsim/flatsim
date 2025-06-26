#include "flatsim/robot/sensors/imu_sensor.hpp"
#include <random>
#include <cmath>

namespace fs {
    
    IMUSensor::IMUSensor(double frequency, double accel_noise, double gyro_noise, double mag_noise)
        : update_frequency(frequency)
        , next_update_time(0.0)
        , accel_noise_std(accel_noise)
        , gyro_noise_std(gyro_noise)
        , mag_noise_std(mag_noise)
        , accel_bias_std(0.001)  // 1 mg/s bias drift
        , gyro_bias_std(0.0001)  // 0.0001 rad/s bias drift
        , accel_bias_x(0.0), accel_bias_y(0.0), accel_bias_z(0.0)
        , gyro_bias_x(0.0), gyro_bias_y(0.0), gyro_bias_z(0.0)
        , auto_calibrate(true)
        , calibration_samples(0)
        , max_calibration_samples(1000)
        , magnetic_declination(0.0)    // 0° declination
        , magnetic_inclination(1.047)  // ~60° inclination (typical for mid-latitudes)
        , magnetic_intensity(50.0)     // 50 μT (typical Earth field strength)
        , last_update_real_time(0.0)
    {
        // Initialize with gravity pointing down
        current_data.accel_z = -9.81; // Gravity in body frame (assuming Z-up)
        
        // Start with identity quaternion (no rotation)
        current_data.quat_w = 1.0;
        current_data.quat_x = 0.0;
        current_data.quat_y = 0.0;
        current_data.quat_z = 0.0;
        
        // Set reasonable default temperature
        current_data.temperature = 25.0;
    }
    
    void IMUSensor::update(double dt) {
        last_update_time += dt;
        
        // Check if it's time for an update based on frequency
        if (last_update_time >= next_update_time) {
            double actual_dt = last_update_time - last_update_real_time;
            last_update_real_time = last_update_time;
            
            // Calculate sensor readings from robot motion
            calculate_accelerations(actual_dt);
            calculate_angular_velocities(actual_dt);
            calculate_magnetic_field();
            
            // Update sensor biases (simulate drift)
            update_biases(actual_dt);
            
            // Apply calibration if available
            apply_calibration();
            
            // Add sensor noise
            add_sensor_noise();
            
            // Update orientation estimate
            update_orientation();
            
            // Convert quaternion to Euler angles
            quaternion_to_euler();
            
            // Update calibration status
            update_calibration_status();
            
            // Update timestamp
            current_data.timestamp = std::chrono::system_clock::now();
            
            // Mark data as valid
            data_valid = true;
            
            // Schedule next update
            next_update_time = last_update_time + (1.0 / update_frequency);
            
            // Update last pose for next iteration
            last_pose = robot_pose;
        }
    }
    
    void IMUSensor::set_robot_pose(const concord::Pose& pose) {
        robot_pose = pose;
    }
    
    void* IMUSensor::get_data() {
        return &current_data;
    }
    
    std::string IMUSensor::get_type() const {
        return "IMU";
    }
    
    bool IMUSensor::is_data_valid() const {
        return data_valid;
    }
    
    double IMUSensor::get_frequency() const {
        return update_frequency;
    }
    
    const IMUData& IMUSensor::get_imu_data() const {
        return current_data;
    }
    
    void IMUSensor::set_auto_calibration(bool enable) {
        auto_calibrate = enable;
        if (!enable) {
            calibration_samples = 0;
        }
    }
    
    void IMUSensor::start_calibration(int samples) {
        calibration_samples = 0;
        max_calibration_samples = samples;
        auto_calibrate = true;
    }
    
    bool IMUSensor::is_calibrating() const {
        return auto_calibrate && (calibration_samples < max_calibration_samples);
    }
    
    IMUData::CalibrationStatus IMUSensor::get_calibration_status() const {
        // Return the worst calibration status
        IMUData::CalibrationStatus worst = IMUData::CalibrationStatus::FULLY_CALIBRATED;
        
        if (current_data.accel_cal < worst) worst = current_data.accel_cal;
        if (current_data.gyro_cal < worst) worst = current_data.gyro_cal;
        if (current_data.mag_cal < worst) worst = current_data.mag_cal;
        
        return worst;
    }
    
    void IMUSensor::set_magnetic_field(double declination, double inclination, double intensity) {
        magnetic_declination = declination;
        magnetic_inclination = inclination;
        magnetic_intensity = intensity;
    }
    
    void IMUSensor::configure_noise(double accel_noise, double gyro_noise, double mag_noise) {
        accel_noise_std = accel_noise;
        gyro_noise_std = gyro_noise;
        mag_noise_std = mag_noise;
    }
    
    void IMUSensor::reset_calibration() {
        calibration_samples = 0;
        accel_bias_x = accel_bias_y = accel_bias_z = 0.0;
        gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0;
        
        current_data.accel_cal = IMUData::CalibrationStatus::UNCALIBRATED;
        current_data.gyro_cal = IMUData::CalibrationStatus::UNCALIBRATED;
        current_data.mag_cal = IMUData::CalibrationStatus::UNCALIBRATED;
    }
    
    void IMUSensor::calculate_accelerations(double dt) {
        if (dt <= 0.0) {
            // If no time has passed, keep previous acceleration
            return;
        }
        
        // Calculate linear acceleration from position change
        static double last_vel_x = 0.0, last_vel_y = 0.0, last_vel_z = 0.0;
        
        // Estimate velocity from position change
        double vel_x = (robot_pose.point.x - last_pose.point.x) / dt;
        double vel_y = (robot_pose.point.y - last_pose.point.y) / dt;
        double vel_z = (robot_pose.point.z - last_pose.point.z) / dt;
        
        // Calculate acceleration from velocity change
        double accel_x = (vel_x - last_vel_x) / dt;
        double accel_y = (vel_y - last_vel_y) / dt;
        double accel_z = (vel_z - last_vel_z) / dt;
        
        // Transform from world frame to body frame
        // For simplicity, assume robot orientation matches robot_pose angle
        double cos_yaw = std::cos(robot_pose.angle.yaw);
        double sin_yaw = std::sin(robot_pose.angle.yaw);
        double cos_pitch = std::cos(robot_pose.angle.pitch);
        double sin_pitch = std::sin(robot_pose.angle.pitch);
        double cos_roll = std::cos(robot_pose.angle.roll);
        double sin_roll = std::sin(robot_pose.angle.roll);
        
        // Rotation matrix from world to body (simplified)
        current_data.accel_x = accel_x * cos_yaw + accel_y * sin_yaw;
        current_data.accel_y = -accel_x * sin_yaw + accel_y * cos_yaw;
        current_data.accel_z = accel_z + 9.81; // Add gravity (in body frame, gravity points down)
        
        // Update for next iteration
        last_vel_x = vel_x;
        last_vel_y = vel_y;
        last_vel_z = vel_z;
    }
    
    void IMUSensor::calculate_angular_velocities(double dt) {
        if (dt <= 0.0) {
            return;
        }
        
        // Calculate angular velocities from orientation change
        double delta_roll = robot_pose.angle.roll - last_pose.angle.roll;
        double delta_pitch = robot_pose.angle.pitch - last_pose.angle.pitch;
        double delta_yaw = robot_pose.angle.yaw - last_pose.angle.yaw;
        
        // Handle angle wrapping
        while (delta_roll > M_PI) delta_roll -= 2.0 * M_PI;
        while (delta_roll < -M_PI) delta_roll += 2.0 * M_PI;
        while (delta_pitch > M_PI) delta_pitch -= 2.0 * M_PI;
        while (delta_pitch < -M_PI) delta_pitch += 2.0 * M_PI;
        while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
        while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
        
        // Calculate angular velocities in body frame
        current_data.gyro_x = delta_roll / dt;
        current_data.gyro_y = delta_pitch / dt;
        current_data.gyro_z = delta_yaw / dt;
    }
    
    void IMUSensor::calculate_magnetic_field() {
        // Simulate Earth's magnetic field in body frame
        // Earth's magnetic field vector in NED frame
        double mag_north = magnetic_intensity * std::cos(magnetic_inclination);
        double mag_east = mag_north * std::sin(magnetic_declination);
        double mag_north_comp = mag_north * std::cos(magnetic_declination);
        double mag_down = magnetic_intensity * std::sin(magnetic_inclination);
        
        // Transform to body frame using current orientation
        double cos_yaw = std::cos(robot_pose.angle.yaw);
        double sin_yaw = std::sin(robot_pose.angle.yaw);
        double cos_pitch = std::cos(robot_pose.angle.pitch);
        double sin_pitch = std::sin(robot_pose.angle.pitch);
        double cos_roll = std::cos(robot_pose.angle.roll);
        double sin_roll = std::sin(robot_pose.angle.roll);
        
        // Simplified rotation from NED to body frame
        current_data.mag_x = mag_north_comp * cos_yaw + mag_east * sin_yaw;
        current_data.mag_y = -mag_north_comp * sin_yaw + mag_east * cos_yaw;
        current_data.mag_z = mag_down;
    }
    
    void IMUSensor::update_orientation() {
        // Simple orientation update using accelerometer and magnetometer
        // In a real implementation, this would use a more sophisticated sensor fusion algorithm
        
        // Extract gravity vector from accelerometer (assuming low acceleration)
        double ax = current_data.accel_x;
        double ay = current_data.accel_y;
        double az = current_data.accel_z;
        
        // Normalize gravity vector
        double accel_norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (accel_norm > 0.1) { // Avoid division by zero
            ax /= accel_norm;
            ay /= accel_norm;
            az /= accel_norm;
            
            // Calculate roll and pitch from gravity vector
            current_data.roll = std::atan2(ay, az);
            current_data.pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
        }
        
        // Calculate yaw from magnetometer (simplified)
        double mx = current_data.mag_x;
        double my = current_data.mag_y;
        
        // Compensate for roll and pitch
        double mag_x_comp = mx * std::cos(current_data.pitch) + 
                           current_data.mag_z * std::sin(current_data.pitch);
        double mag_y_comp = my * std::cos(current_data.roll) + 
                           current_data.mag_z * std::sin(current_data.roll);
        
        current_data.yaw = std::atan2(-mag_y_comp, mag_x_comp);
        
        // Update quaternion from Euler angles
        double cy = std::cos(current_data.yaw * 0.5);
        double sy = std::sin(current_data.yaw * 0.5);
        double cp = std::cos(current_data.pitch * 0.5);
        double sp = std::sin(current_data.pitch * 0.5);
        double cr = std::cos(current_data.roll * 0.5);
        double sr = std::sin(current_data.roll * 0.5);
        
        current_data.quat_w = cr * cp * cy + sr * sp * sy;
        current_data.quat_x = sr * cp * cy - cr * sp * sy;
        current_data.quat_y = cr * sp * cy + sr * cp * sy;
        current_data.quat_z = cr * cp * sy - sr * sp * cy;
        
        normalize_quaternion();
    }
    
    void IMUSensor::quaternion_to_euler() {
        // Convert quaternion to Euler angles (already done in update_orientation)
        // This method exists for consistency with the interface
    }
    
    void IMUSensor::add_sensor_noise() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        // Add noise to accelerometer
        std::normal_distribution<double> accel_noise(0.0, accel_noise_std);
        current_data.accel_x += accel_noise(gen) + accel_bias_x;
        current_data.accel_y += accel_noise(gen) + accel_bias_y;
        current_data.accel_z += accel_noise(gen) + accel_bias_z;
        
        // Add noise to gyroscope
        std::normal_distribution<double> gyro_noise(0.0, gyro_noise_std);
        current_data.gyro_x += gyro_noise(gen) + gyro_bias_x;
        current_data.gyro_y += gyro_noise(gen) + gyro_bias_y;
        current_data.gyro_z += gyro_noise(gen) + gyro_bias_z;
        
        // Add noise to magnetometer
        std::normal_distribution<double> mag_noise(0.0, mag_noise_std);
        current_data.mag_x += mag_noise(gen);
        current_data.mag_y += mag_noise(gen);
        current_data.mag_z += mag_noise(gen);
        
        // Add temperature variation
        std::normal_distribution<double> temp_noise(0.0, 1.0);
        current_data.temperature = 25.0 + temp_noise(gen);
    }
    
    void IMUSensor::update_biases(double dt) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        // Update biases with random walk
        std::normal_distribution<double> accel_bias_noise(0.0, accel_bias_std * std::sqrt(dt));
        std::normal_distribution<double> gyro_bias_noise(0.0, gyro_bias_std * std::sqrt(dt));
        
        accel_bias_x += accel_bias_noise(gen);
        accel_bias_y += accel_bias_noise(gen);
        accel_bias_z += accel_bias_noise(gen);
        
        gyro_bias_x += gyro_bias_noise(gen);
        gyro_bias_y += gyro_bias_noise(gen);
        gyro_bias_z += gyro_bias_noise(gen);
    }
    
    void IMUSensor::update_calibration_status() {
        if (auto_calibrate && calibration_samples < max_calibration_samples) {
            calibration_samples++;
            
            // Update calibration status based on number of samples
            if (calibration_samples > max_calibration_samples * 0.9) {
                current_data.accel_cal = IMUData::CalibrationStatus::FULLY_CALIBRATED;
                current_data.gyro_cal = IMUData::CalibrationStatus::FULLY_CALIBRATED;
                current_data.mag_cal = IMUData::CalibrationStatus::FULLY_CALIBRATED;
            } else if (calibration_samples > max_calibration_samples * 0.7) {
                current_data.accel_cal = IMUData::CalibrationStatus::MOSTLY_CALIBRATED;
                current_data.gyro_cal = IMUData::CalibrationStatus::MOSTLY_CALIBRATED;
                current_data.mag_cal = IMUData::CalibrationStatus::MOSTLY_CALIBRATED;
            } else if (calibration_samples > max_calibration_samples * 0.3) {
                current_data.accel_cal = IMUData::CalibrationStatus::PARTIALLY_CALIBRATED;
                current_data.gyro_cal = IMUData::CalibrationStatus::PARTIALLY_CALIBRATED;
                current_data.mag_cal = IMUData::CalibrationStatus::PARTIALLY_CALIBRATED;
            }
        }
    }
    
    void IMUSensor::apply_calibration() {
        // Apply bias corrections if calibrated
        if (current_data.accel_cal >= IMUData::CalibrationStatus::PARTIALLY_CALIBRATED) {
            // Bias correction would be applied here in a real implementation
        }
        
        if (current_data.gyro_cal >= IMUData::CalibrationStatus::PARTIALLY_CALIBRATED) {
            // Bias correction would be applied here in a real implementation
        }
        
        if (current_data.mag_cal >= IMUData::CalibrationStatus::PARTIALLY_CALIBRATED) {
            // Hard and soft iron corrections would be applied here in a real implementation
        }
    }
    
    void IMUSensor::normalize_quaternion() {
        double norm = std::sqrt(current_data.quat_w * current_data.quat_w +
                               current_data.quat_x * current_data.quat_x +
                               current_data.quat_y * current_data.quat_y +
                               current_data.quat_z * current_data.quat_z);
        
        if (norm > 0.0) {
            current_data.quat_w /= norm;
            current_data.quat_x /= norm;
            current_data.quat_y /= norm;
            current_data.quat_z /= norm;
        }
    }
    
} // namespace fs
