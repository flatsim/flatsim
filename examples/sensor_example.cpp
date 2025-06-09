/**
 * @file sensor_example.cpp
 * @brief Example demonstrating how to use the sensor system
 * 
 * This example shows how to:
 * 1. Create sensors using the factory function
 * 2. Add sensors to a robot
 * 3. Update sensors and retrieve data
 * 4. Work with RTK GPS specifically
 */

#include "multiverse/robot/sensors.hpp"
#include "multiverse/robot.hpp"
#include <iostream>
#include <memory>

using namespace mvs;

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
    robot_pose.point.enu.x = 100.0;  // 100m east
    robot_pose.point.enu.y = 200.0;  // 200m north
    robot_pose.point.enu.z = 10.0;   // 10m up
    
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

void demonstrate_sensor_management() {
    std::cout << "=== Sensor Management Example ===\n";
    
    // This would typically be done within a Robot class
    std::vector<std::unique_ptr<Sensor>> sensors;
    
    // Add multiple GPS sensors (e.g., primary and backup)
    sensors.push_back(sensors::create_sensor<GPSSensor>(10.0, true));   // Primary RTK GPS
    sensors.push_back(sensors::create_sensor<GPSSensor>(5.0, false));   // Backup standard GPS
    
    std::cout << "Created " << sensors.size() << " sensors:\n";
    for (size_t i = 0; i < sensors.size(); ++i) {
        std::cout << "  Sensor " << i << ": " << sensors[i]->get_type() 
                  << " @ " << sensors[i]->get_frequency() << " Hz\n";
    }
    
    // Simulate sensor updates
    concord::Pose robot_pose;
    robot_pose.point.enu.x = 50.0;
    robot_pose.point.enu.y = 75.0;
    robot_pose.point.enu.z = 5.0;
    
    for (auto& sensor : sensors) {
        sensor->set_robot_pose(robot_pose);
        sensor->update(0.1);  // 100ms update
        
        if (sensor->is_data_valid()) {
            std::cout << "  " << sensor->get_type() << " sensor updated successfully\n";
        }
    }
    
    std::cout << std::endl;
}

// Note: This is an example file and would not typically be compiled
// It demonstrates the sensor API usage
#if 0
int main() {
    demonstrate_gps_sensor();
    demonstrate_sensor_management();
    return 0;
}
#endif
