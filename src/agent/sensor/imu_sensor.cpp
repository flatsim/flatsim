#include "flatsim/agent/sensor/imu_sensor.hpp"
#include "flatsim/utils.hpp"
#include <cmath>
#include <cstdio>
#include <echo/echo.hpp>
#include <random>
#include <unistd.h>

namespace fs {

    IMUSensor::IMUSensor(double frequency, double accel_noise, double gyro_noise, double mag_noise)
        : update_frequency(frequency), next_update_time(0.0), accel_noise_std(accel_noise), gyro_noise_std(gyro_noise),
          mag_noise_std(mag_noise), accel_bias_std(0.001) // 1 mg/s bias drift
          ,
          gyro_bias_std(0.0001) // 0.0001 rad/s bias drift
          ,
          accel_bias_x(0.0), accel_bias_y(0.0), accel_bias_z(0.0), gyro_bias_x(0.0), gyro_bias_y(0.0), gyro_bias_z(0.0),
          auto_calibrate(true), calibration_samples(0), max_calibration_samples(1000),
          magnetic_declination(0.0) // 0° declination
          ,
          magnetic_inclination(1.047) // ~60° inclination (typical for mid-latitudes)
          ,
          magnetic_intensity(50.0) // 50 μT (typical Earth field strength)
          ,
          last_update_real_time(0.0), linear_vel_x(0.0), linear_vel_y(0.0), angular_vel(0.0), last_linear_vel_x(0.0),
          last_linear_vel_y(0.0), last_angular_vel(0.0) {
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

            // If simulator data is available, use it. Otherwise, compute from physics.
            if (simulator_data_available_) {
                // Data already set by update_from_simulator()
                // Just add noise and calculate magnetic field
                calculate_magnetic_field();
                simulator_data_available_ = false; // Reset for next tick
            } else {
                // Fallback: Calculate sensor readings from robot motion
                calculate_accelerations(actual_dt);
                calculate_angular_velocities(actual_dt);
                calculate_magnetic_field();
            }

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

            // Write to shared memory if enabled
            if (shm_enabled) {
                write_to_shm();
            }

            // Write to PTY serial output
            if (pty_) {
                char buf[256];
                int len = snprintf(buf, sizeof(buf),
                                   R"({"ax":%.4f,"ay":%.4f,"az":%.4f,"gx":%.4f,"gy":%.4f,"gz":%.4f,"yaw":%.4f})"
                                   "\n",
                                   current_data.accel_x, current_data.accel_y, current_data.accel_z,
                                   current_data.gyro_x, current_data.gyro_y, current_data.gyro_z, current_data.yaw);
                ::write(pty_->master_fd(), buf, len);
            }

            // Schedule next update
            next_update_time = last_update_time + (1.0 / update_frequency);

            // Update last pose for next iteration
            last_pose = robot_pose;
        }
    }

    void IMUSensor::set_robot_pose(const datapod::Pose &pose) { robot_pose = pose; }

    void IMUSensor::set_physics_data(double vel_x, double vel_y, double ang_vel) {
        linear_vel_x = vel_x;
        linear_vel_y = vel_y;
        angular_vel = ang_vel;
    }

    void IMUSensor::update_from_simulator(const types::SensorData &data) {
        if (!data.has_imu) {
            return;
        }

        // Use IMU data computed by simulator
        current_data.accel_x = static_cast<double>(data.imu.accel_x);
        current_data.accel_y = static_cast<double>(data.imu.accel_y);
        current_data.accel_z = static_cast<double>(data.imu.accel_z);
        current_data.gyro_z = static_cast<double>(data.imu.gyro_z);
        current_data.yaw = static_cast<double>(data.imu.yaw);

        // Mark that we have simulator data (skip self-computation in update())
        simulator_data_available_ = true;
    }

    void *IMUSensor::get_data() { return &current_data; }

    std::string IMUSensor::get_type() const { return "IMU"; }

    bool IMUSensor::is_data_valid() const { return data_valid; }

    double IMUSensor::get_frequency() const { return update_frequency; }

    const IMUData &IMUSensor::get_imu_data() const { return current_data; }

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

    bool IMUSensor::is_calibrating() const { return auto_calibrate && (calibration_samples < max_calibration_samples); }

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

        // Calculate acceleration from velocity change (using physics engine velocities)
        double accel_world_x = (linear_vel_x - last_linear_vel_x) / dt;
        double accel_world_y = (linear_vel_y - last_linear_vel_y) / dt;

        // Transform from world frame to body frame
        double cos_yaw = std::cos(utils::get_yaw(robot_pose));
        double sin_yaw = std::sin(utils::get_yaw(robot_pose));

        // Rotation from world to body frame (2D, around Z axis)
        current_data.accel_x = accel_world_x * cos_yaw + accel_world_y * sin_yaw;
        current_data.accel_y = -accel_world_x * sin_yaw + accel_world_y * cos_yaw;
        current_data.accel_z = 9.81; // Gravity in body frame (IMU measures specific force, not acceleration)

        // Update for next iteration
        last_linear_vel_x = linear_vel_x;
        last_linear_vel_y = linear_vel_y;
    }

    void IMUSensor::calculate_angular_velocities(double dt) {
        if (dt <= 0.0) {
            return;
        }

        // Use angular velocity directly from physics engine
        // In 2D physics (flywheel), we only have Z-axis rotation
        current_data.gyro_x = 0.0;         // No roll in 2D
        current_data.gyro_y = 0.0;         // No pitch in 2D
        current_data.gyro_z = angular_vel; // Yaw rate from physics

        // Update for next iteration
        last_angular_vel = angular_vel;
    }

    void IMUSensor::calculate_magnetic_field() {
        // Simulate Earth's magnetic field in body frame
        // Earth's magnetic field vector in NED frame
        double mag_north = magnetic_intensity * std::cos(magnetic_inclination);
        double mag_east = mag_north * std::sin(magnetic_declination);
        double mag_north_comp = mag_north * std::cos(magnetic_declination);
        double mag_down = magnetic_intensity * std::sin(magnetic_inclination);

        // Transform to body frame using current orientation
        double cos_yaw = std::cos(utils::get_yaw(robot_pose));
        double sin_yaw = std::sin(utils::get_yaw(robot_pose));
        double cos_pitch = std::cos(utils::get_pitch(robot_pose));
        double sin_pitch = std::sin(utils::get_pitch(robot_pose));
        double cos_roll = std::cos(utils::get_roll(robot_pose));
        double sin_roll = std::sin(utils::get_roll(robot_pose));

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
        double accel_norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (accel_norm > 0.1) { // Avoid division by zero
            ax /= accel_norm;
            ay /= accel_norm;
            az /= accel_norm;

            // Calculate roll and pitch from gravity vector
            current_data.roll = std::atan2(ay, az);
            current_data.pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        }

        // Calculate yaw from magnetometer (simplified)
        double mx = current_data.mag_x;
        double my = current_data.mag_y;

        // Compensate for roll and pitch
        double mag_x_comp = mx * std::cos(current_data.pitch) + current_data.mag_z * std::sin(current_data.pitch);
        double mag_y_comp = my * std::cos(current_data.roll) + current_data.mag_z * std::sin(current_data.roll);

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
        double norm = std::sqrt(current_data.quat_w * current_data.quat_w + current_data.quat_x * current_data.quat_x +
                                current_data.quat_y * current_data.quat_y + current_data.quat_z * current_data.quat_z);

        if (norm > 0.0) {
            current_data.quat_w /= norm;
            current_data.quat_x /= norm;
            current_data.quat_y /= norm;
            current_data.quat_z /= norm;
        }
    }

    bool IMUSensor::write_to_shm() {
        if (!is_data_valid() || !is_shm_enabled()) {
            return false;
        }

        // Pack IMU data into binary format (15 doubles = 120 bytes)
        struct {
            double accel_x, accel_y, accel_z;
            double gyro_x, gyro_y, gyro_z;
            double mag_x, mag_y, mag_z;
            double quat_w, quat_x, quat_y, quat_z;
            double roll, pitch, yaw;
        } binary_data;

        binary_data.accel_x = current_data.accel_x;
        binary_data.accel_y = current_data.accel_y;
        binary_data.accel_z = current_data.accel_z;
        binary_data.gyro_x = current_data.gyro_x;
        binary_data.gyro_y = current_data.gyro_y;
        binary_data.gyro_z = current_data.gyro_z;
        binary_data.mag_x = current_data.mag_x;
        binary_data.mag_y = current_data.mag_y;
        binary_data.mag_z = current_data.mag_z;
        binary_data.quat_w = current_data.quat_w;
        binary_data.quat_x = current_data.quat_x;
        binary_data.quat_y = current_data.quat_y;
        binary_data.quat_z = current_data.quat_z;
        binary_data.roll = current_data.roll;
        binary_data.pitch = current_data.pitch;
        binary_data.yaw = current_data.yaw;

        return write_shm_data(&binary_data, sizeof(binary_data));
    }

    std::string IMUSensor::get_metadata() const {
        std::string metadata;
        metadata += "IMU Binary Format Description\n";
        metadata += "==============================\n\n";
        metadata += "Format: Binary packed struct\n";
        metadata += "Total size: 120 bytes (15 doubles)\n\n";
        metadata += "Structure:\n";
        metadata += "----------\n";
        metadata += "Offset | Size | Type   | Field\n";
        metadata += "-------|------|--------|------------------\n";
        metadata += "0      | 8    | double | accel_x (m/s²)\n";
        metadata += "8      | 8    | double | accel_y (m/s²)\n";
        metadata += "16     | 8    | double | accel_z (m/s²)\n";
        metadata += "24     | 8    | double | gyro_x (rad/s)\n";
        metadata += "32     | 8    | double | gyro_y (rad/s)\n";
        metadata += "40     | 8    | double | gyro_z (rad/s)\n";
        metadata += "48     | 8    | double | mag_x (μT)\n";
        metadata += "56     | 8    | double | mag_y (μT)\n";
        metadata += "64     | 8    | double | mag_z (μT)\n";
        metadata += "72     | 8    | double | quat_w\n";
        metadata += "80     | 8    | double | quat_x\n";
        metadata += "88     | 8    | double | quat_y\n";
        metadata += "96     | 8    | double | quat_z\n";
        metadata += "104    | 8    | double | roll (rad)\n";
        metadata += "112    | 8    | double | pitch (rad)\n";
        metadata += "120    | 8    | double | yaw (rad)\n\n";
        metadata += "Example C code:\n";
        metadata += "  struct imu_data { double accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,\n";
        metadata += "                    mag_x, mag_y, mag_z, quat_w, quat_x, quat_y, quat_z,\n";
        metadata += "                    roll, pitch, yaw; };\n";
        metadata += "  struct imu_data imu;\n";
        metadata += "  memcpy(&imu, shm_data_ptr, sizeof(imu));\n";
        return metadata;
    }

    std::string IMUSensor::enable_serial_output(const std::string &uuid) {
        auto res = wirebit::PtyLink::create();
        if (res.is_err()) {
            echo::error("[IMUSensor] Failed to create PTY: ", res.error().message.c_str()).red();
            return "";
        }
        pty_ = std::make_unique<wirebit::PtyLink>(std::move(res.value()));

        std::string path = std::string(pty_->slave_path().c_str());

        // Create symlink if UUID provided
        if (!uuid.empty()) {
            std::string symlink_path = "/tmp/flatsim/" + uuid + "/imu";
            std::string dir = "/tmp/flatsim/" + uuid;

            std::system(("mkdir -p " + dir).c_str());
            ::unlink(symlink_path.c_str());

            if (::symlink(path.c_str(), symlink_path.c_str()) == 0) {
                echo::trace("[IMUSensor] Serial: ", symlink_path, " -> ", path).green();
                path = symlink_path;
            } else {
                echo::trace("[IMUSensor] Serial: ", path).green();
            }
        } else {
            echo::trace("[IMUSensor] Serial: ", path).green();
        }
        return path;
    }

    std::string IMUSensor::get_serial_path() const { return pty_ ? std::string(pty_->slave_path().c_str()) : ""; }

} // namespace fs
