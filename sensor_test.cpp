/**
 * @file sensor_test.cpp
 * @brief Simple test program to verify sensor system functionality
 */

#include "multiverse/robot/sensors.hpp"
#include <iostream>
#include <vector>
#include <memory>

using namespace mvs;

int main() {
    std::cout << "=== Multiverse Sensor System Test ===\n\n";
    
    try {
        // Create all three sensor types
        std::cout << "Creating sensors...\n";
        
        auto gps = sensors::create_sensor<GPSSensor>(10.0, true, 3.0, 0.02);
        auto imu = sensors::create_sensor<IMUSensor>(100.0, 0.01, 0.05);
        auto lidar = sensors::create_sensor<LIDARSensor>(
            10.0, LIDARSensor::ScanPattern::HORIZONTAL_2D, 100.0, 360.0, 0.25);
        
        std::cout << "✓ GPS Sensor created: " << gps->get_type() 
                  << " @ " << gps->get_frequency() << " Hz\n";
        std::cout << "✓ IMU Sensor created: " << imu->get_type() 
                  << " @ " << imu->get_frequency() << " Hz\n";
        std::cout << "✓ LIDAR Sensor created: " << lidar->get_type() 
                  << " @ " << lidar->get_frequency() << " Hz\n\n";
        
        // Set up robot pose
        concord::Pose robot_pose;
        robot_pose.point.enu.x = 100.0;
        robot_pose.point.enu.y = 200.0;
        robot_pose.point.enu.z = 5.0;
        robot_pose.quaternion.x = 0.0;
        robot_pose.quaternion.y = 0.0;
        robot_pose.quaternion.z = 0.0;
        robot_pose.quaternion.w = 1.0;
        
        std::cout << "Testing sensor updates...\n";
        
        // Test GPS
        gps->set_robot_pose(robot_pose);
        gps->update(0.1);
        std::cout << "✓ GPS sensor updated, data valid: " 
                  << (gps->is_data_valid() ? "YES" : "NO") << "\n";
        
        // Test IMU
        imu->set_robot_pose(robot_pose);
        imu->update(0.01);
        std::cout << "✓ IMU sensor updated, data valid: " 
                  << (imu->is_data_valid() ? "YES" : "NO") << "\n";
        
        // Test LIDAR
        lidar->set_robot_pose(robot_pose);
        lidar->update(0.1);
        std::cout << "✓ LIDAR sensor updated, data valid: " 
                  << (lidar->is_data_valid() ? "YES" : "NO") << "\n";
        
        std::cout << "\n=== Sensor System Test PASSED ===\n";
        
        // Show basic data if available
        if (gps->is_data_valid()) {
            auto* gps_sensor = static_cast<GPSSensor*>(gps.get());
            const auto& gps_data = gps_sensor->get_gps_data();
            std::cout << "GPS Position: " << gps_data.latitude << ", " 
                      << gps_data.longitude << ", " << gps_data.altitude << " m\n";
        }
        
        if (imu->is_data_valid()) {
            auto* imu_sensor = static_cast<IMUSensor*>(imu.get());
            const auto& imu_data = imu_sensor->get_imu_data();
            std::cout << "IMU Orientation: [" << imu_data.orientation.w << ", "
                      << imu_data.orientation.x << ", " << imu_data.orientation.y 
                      << ", " << imu_data.orientation.z << "]\n";
        }
        
        if (lidar->is_data_valid()) {
            auto* lidar_sensor = static_cast<LIDARSensor*>(lidar.get());
            const auto& lidar_data = lidar_sensor->get_lidar_data();
            std::cout << "LIDAR Points: " << lidar_data.points.size() << "\n";
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
