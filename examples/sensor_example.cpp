/**
 * @file sensor_example.cpp
 * @brief Example demonstrating how to use the sensor system
 * 
 * This example shows how to:
 * 1. Create sensors using the factory function
 * 2. Add sensors to a robot
 * 3. Update sensors and retrieve data
 * 4. Work with RTK GPS specifically
 * 5. Demonstrate IMU sensor capabilities
 * 6. Show LIDAR sensor integration with physics
 */

#include "flatsim/robot/sensors.hpp"
#include "flatsim/robot.hpp"
#include "flatsim/world.hpp"  // For muli::World
#include <iostream>
#include <memory>
#include <iomanip>
#include <algorithm>

using namespace fs;

void demonstrate_gps_sensor() {
    std::cout << "=== GPS Sensor Example ===\n";
    
    // Create a GPS sensor with RTK capabilities
    auto gps = sensors::create_sensor<GPSSensor>(
        10.0,    // 10 Hz update rate
        true,    // Enable RTK
        3.0,     // 3m base accuracy
        0.02     // 2cm RTK accuracy
    );
    
    // Cast to GPS sensor for specific functionality
    auto* gps_sensor = static_cast<GPSSensor*>(gps.get());
    
    // Configure the sensor
    gps_sensor->set_satellite_count(12);  // Good satellite visibility
    gps_sensor->configure_noise(0.05, 0.01);  // Low noise levels
    
    // Simulate robot pose
    concord::Pose robot_pose;
    robot_pose.point.x = 100.0;  // 100m east
    robot_pose.point.y = 200.0;  // 200m north
    robot_pose.point.z = 10.0;   // 10m up
    
    // Update sensor
    gps_sensor->set_robot_pose(robot_pose);
    gps_sensor->update(0.1);  // 100ms update
    
    // Get data
    if (gps_sensor->is_data_valid()) {
        const GPSData& data = gps_sensor->get_gps_data();
        
        std::cout << "GPS Data:\n";
        std::cout << "  Latitude: " << std::fixed << std::setprecision(8) << data.latitude << "°\n";
        std::cout << "  Longitude: " << std::fixed << std::setprecision(8) << data.longitude << "°\n";
        std::cout << "  Altitude: " << std::fixed << std::setprecision(2) << data.altitude << " m\n";
        std::cout << "  Horizontal Accuracy: " << data.horizontal_accuracy << " m\n";
        std::cout << "  Vertical Accuracy: " << data.vertical_accuracy << " m\n";
        std::cout << "  Satellites: " << data.num_satellites << "\n";
        
        // RTK Status
        std::string rtk_status;
        switch (data.rtk_status) {
            case GPSData::RTKStatus::NO_FIX: rtk_status = "NO_FIX"; break;
            case GPSData::RTKStatus::SINGLE: rtk_status = "SINGLE"; break;
            case GPSData::RTKStatus::DGPS: rtk_status = "DGPS"; break;
            case GPSData::RTKStatus::RTK_FLOAT: rtk_status = "RTK_FLOAT"; break;
            case GPSData::RTKStatus::RTK_FIXED: rtk_status = "RTK_FIXED"; break;
        }
        std::cout << "  RTK Status: " << rtk_status << "\n";
    }
    
    std::cout << std::endl;
}

void demonstrate_imu_sensor() {
    std::cout << "=== IMU Sensor Example ===\n";
    
    // Create a 9-DOF IMU sensor
    auto imu = sensors::create_sensor<IMUSensor>(
        100.0,   // 100 Hz update rate (typical for IMU)
        0.01,    // 0.01 m/s² accelerometer noise
        0.001,   // 0.001 rad/s gyroscope noise
        0.05     // 0.05 µT magnetometer noise
    );
    
    // Cast to IMU sensor for specific functionality
    auto* imu_sensor = static_cast<IMUSensor*>(imu.get());
    
    // Configure noise parameters (available method)
    imu_sensor->configure_noise(0.01, 0.001, 0.05);  // accel, gyro, mag noise
    
    // Simulate robot motion (rotating and accelerating)
    concord::Pose robot_pose;
    robot_pose.point.x = 0.0;
    robot_pose.point.y = 0.0;
    robot_pose.point.z = 1.0;  // 1m above ground
    robot_pose.angle.roll = 0.0;
    robot_pose.angle.pitch = 0.0;
    robot_pose.angle.yaw = 0.1;  // Small rotation around Z-axis
    
    // Update sensor
    imu_sensor->set_robot_pose(robot_pose);
    imu_sensor->update(0.01);  // 10ms update (100Hz)
    
    // Get data
    if (imu_sensor->is_data_valid()) {
        const IMUData& data = imu_sensor->get_imu_data();
        
        std::cout << "IMU Data:\n";
        std::cout << "  Acceleration (m/s²): [" 
                  << std::fixed << std::setprecision(3)
                  << data.accel_x << ", " 
                  << data.accel_y << ", " 
                  << data.accel_z << "]\n";
        std::cout << "  Angular Velocity (rad/s): [" 
                  << data.gyro_x << ", " 
                  << data.gyro_y << ", " 
                  << data.gyro_z << "]\n";
        std::cout << "  Magnetic Field (µT): [" 
                  << data.mag_x << ", " 
                  << data.mag_y << ", " 
                  << data.mag_z << "]\n";
        std::cout << "  Orientation (quaternion): [" 
                  << data.quat_w << ", " 
                  << data.quat_x << ", " 
                  << data.quat_y << ", " 
                  << data.quat_z << "]\n";
        
        std::cout << "  Temperature: " << data.temperature << " °C\n";
        
        // Calibration status
        std::string cal_status;
        switch (data.accel_cal) {
            case IMUData::CalibrationStatus::UNCALIBRATED: cal_status = "UNCALIBRATED"; break;
            case IMUData::CalibrationStatus::PARTIALLY_CALIBRATED: cal_status = "PARTIALLY_CALIBRATED"; break;
            case IMUData::CalibrationStatus::MOSTLY_CALIBRATED: cal_status = "MOSTLY_CALIBRATED"; break;
            case IMUData::CalibrationStatus::FULLY_CALIBRATED: cal_status = "FULLY_CALIBRATED"; break;
        }
        std::cout << "  Calibration Status (Accel): " << cal_status << "\n";
    }
    
    std::cout << std::endl;
}

void demonstrate_lidar_sensor() {
    std::cout << "=== LIDAR Sensor Example ===\n";
    
    // Create a 3D LIDAR sensor with physics integration
    // Note: In a real application, you would pass a proper physics world
    std::shared_ptr<muli::World> world = nullptr;  // Placeholder for physics world
    
    auto lidar = sensors::create_sensor<LIDARSensor>(
        world,                                    // Physics world (nullptr for demo)
        LIDARSensor::ScanPattern::CIRCULAR_2D,    // Scan pattern
        20.0,                                     // 20 Hz update rate
        0.5,                                      // 0.5m minimum range
        100.0,                                    // 100m maximum range
        360.0,                                    // 360° horizontal field of view
        0.5                                       // 0.5° angular resolution
    );
    
    // Cast to LIDAR sensor for specific functionality
    auto* lidar_sensor = static_cast<LIDARSensor*>(lidar.get());
    
    // Configure LIDAR parameters
    lidar_sensor->configure_range(0.5, 100.0, 0.01);  // 0.5m min, 100m max, 1cm accuracy
    lidar_sensor->configure_angular(360.0, 30.0, 0.5, 1.0);  // 360° H-FOV, 30° V-FOV, 0.5° res
    lidar_sensor->configure_noise(0.01, 0.1, 0.01);  // Low noise levels
    lidar_sensor->enable_weather_simulation(false);  // Disable weather effects
    
    // Note: In a real simulation, you would set the physics world
    // lidar_sensor->set_physics_world(physics_world);
    
    // Simulate robot pose in environment
    concord::Pose robot_pose;
    robot_pose.point.x = 0.0;
    robot_pose.point.y = 0.0;
    robot_pose.point.z = 1.5;  // 1.5m above ground (typical sensor height)
    robot_pose.angle.roll = 0.0;
    robot_pose.angle.pitch = 0.0;
    robot_pose.angle.yaw = 0.0;  // No rotation
    
    // Update sensor
    lidar_sensor->set_robot_pose(robot_pose);
    lidar_sensor->update(0.05);  // 50ms update (20Hz)
    
    // Get data
    if (lidar_sensor->is_data_valid()) {
        const LIDARData& data = lidar_sensor->get_lidar_data();
        
        std::cout << "LIDAR Data:\n";
        std::cout << "  Number of Points: " << data.ranges.size() << "\n";
        std::cout << "  Scan Duration: " << std::fixed << std::setprecision(3) 
                  << data.scan_duration << " s\n";
        std::cout << "  Min Range: " << data.min_range << " m\n";
        std::cout << "  Max Range: " << data.max_range << " m\n";
        
        // Show first few points as example
        if (!data.points_x.empty()) {
            std::cout << "  Sample Points (first 5):\n";
            size_t num_points = std::min(size_t(5), data.points_x.size());
            for (size_t i = 0; i < num_points; ++i) {
                std::cout << "    Point " << i << ": [" 
                          << std::fixed << std::setprecision(2)
                          << data.points_x[i] << ", " << data.points_y[i] << ", " << data.points_z[i] 
                          << "] intensity: " << (i < data.intensities.size() ? data.intensities[i] : 0.0) << "\n";
            }
        }
        
        // Show range data summary
        if (!data.ranges.empty()) {
            auto min_range = *std::min_element(data.ranges.begin(), data.ranges.end());
            auto max_range = *std::max_element(data.ranges.begin(), data.ranges.end());
            std::cout << "  Range Data: " << data.ranges.size() << " measurements\n";
            std::cout << "    Min measured range: " << min_range << " m\n";
            std::cout << "    Max measured range: " << max_range << " m\n";
        }
    }
    
    std::cout << std::endl;
}

// Main function demonstrating all sensor capabilities
int main() {
    std::cout << "Multiverse Sensor System Example\n";
    std::cout << "================================\n\n";
    
    // Demonstrate individual sensor types
    demonstrate_gps_sensor();
    demonstrate_imu_sensor();
    demonstrate_lidar_sensor();
    
    std::cout << "All sensor demonstrations completed successfully!\n";
    return 0;
}
