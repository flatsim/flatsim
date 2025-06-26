#include "flatsim/robot/sensors/gps_sensor.hpp"
#include <random>
#include <cmath>

namespace fs {
    
    GPSSensor::GPSSensor(double frequency, bool enable_rtk, double base_acc, double rtk_acc)
        : update_frequency(frequency)
        , next_update_time(0.0)
        , rtk_enabled(enable_rtk)
        , base_accuracy(base_acc)
        , rtk_accuracy(rtk_acc)
        , position_noise_std(0.1)
        , velocity_noise_std(0.05)
    {
        current_data.num_satellites = 8; // Default satellite count
        update_rtk_status();
    }
    
    void GPSSensor::update(double dt) {
        last_update_time += dt;
        
        // Check if it's time for an update based on frequency
        if (last_update_time >= next_update_time) {
            // Convert robot pose to GPS coordinates
            convert_enu_to_wgs84(robot_pose);
            
            // Add measurement noise
            add_measurement_noise();
            
            // Update RTK status
            update_rtk_status();
            
            // Update timestamp
            current_data.timestamp = std::chrono::system_clock::now();
            
            // Mark data as valid
            data_valid = true;
            
            // Schedule next update
            next_update_time = last_update_time + (1.0 / update_frequency);
        }
    }
    
    void GPSSensor::set_robot_pose(const concord::Pose& pose) {
        robot_pose = pose;
    }
    
    void* GPSSensor::get_data() {
        return &current_data;
    }
    
    std::string GPSSensor::get_type() const {
        return "GPS";
    }
    
    bool GPSSensor::is_data_valid() const {
        return data_valid;
    }
    
    double GPSSensor::get_frequency() const {
        return update_frequency;
    }
    
    const GPSData& GPSSensor::get_gps_data() const {
        return current_data;
    }
    
    void GPSSensor::set_rtk_available(bool available) {
        rtk_enabled = available;
        update_rtk_status();
    }
    
    GPSData::RTKStatus GPSSensor::get_rtk_status() const {
        return current_data.rtk_status;
    }
    
    void GPSSensor::set_satellite_count(int count) {
        current_data.num_satellites = count;
        update_rtk_status();
    }
    
    void GPSSensor::configure_noise(double pos_noise, double vel_noise) {
        position_noise_std = pos_noise;
        velocity_noise_std = vel_noise;
    }
    
    void GPSSensor::add_measurement_noise() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        // Determine noise level based on RTK status
        double noise_level = base_accuracy;
        switch (current_data.rtk_status) {
            case GPSData::RTKStatus::RTK_FIXED:
                noise_level = rtk_accuracy;
                break;
            case GPSData::RTKStatus::RTK_FLOAT:
                noise_level = rtk_accuracy * 5.0; // 5x worse than fixed
                break;
            case GPSData::RTKStatus::DGPS:
                noise_level = base_accuracy * 0.3; // 30% of base accuracy
                break;
            case GPSData::RTKStatus::SINGLE:
                noise_level = base_accuracy;
                break;
            case GPSData::RTKStatus::NO_FIX:
                noise_level = base_accuracy * 10.0; // Very poor accuracy
                break;
        }
        
        // Add position noise
        std::normal_distribution<double> pos_noise(0.0, noise_level);
        std::normal_distribution<double> vel_noise(0.0, velocity_noise_std);
        
        // Convert noise from meters to degrees (approximate)
        double lat_noise_deg = pos_noise(gen) / 111000.0; // ~111km per degree
        double lon_noise_deg = pos_noise(gen) / (111000.0 * std::cos(current_data.latitude * M_PI / 180.0));
        double alt_noise_m = pos_noise(gen);
        
        current_data.latitude += lat_noise_deg;
        current_data.longitude += lon_noise_deg;
        current_data.altitude += alt_noise_m;
        
        // Add velocity noise
        current_data.velocity_north += vel_noise(gen);
        current_data.velocity_east += vel_noise(gen);
        current_data.velocity_up += vel_noise(gen);
        
        // Update accuracy estimates
        current_data.horizontal_accuracy = noise_level;
        current_data.vertical_accuracy = noise_level * 1.5; // Vertical typically worse
    }
    
    void GPSSensor::update_rtk_status() {
        // Simulate RTK status based on conditions
        if (!rtk_enabled || current_data.num_satellites < 4) {
            current_data.rtk_status = GPSData::RTKStatus::NO_FIX;
        } else if (current_data.num_satellites < 6) {
            current_data.rtk_status = GPSData::RTKStatus::SINGLE;
        } else if (current_data.num_satellites < 8) {
            current_data.rtk_status = GPSData::RTKStatus::DGPS;
        } else if (rtk_enabled && current_data.num_satellites >= 8) {
            // Simulate RTK convergence time and conditions
            static int rtk_convergence_counter = 0;
            rtk_convergence_counter++;
            
            if (rtk_convergence_counter > 100) { // Simulated convergence time
                current_data.rtk_status = GPSData::RTKStatus::RTK_FIXED;
            } else if (rtk_convergence_counter > 50) {
                current_data.rtk_status = GPSData::RTKStatus::RTK_FLOAT;
            } else {
                current_data.rtk_status = GPSData::RTKStatus::DGPS;
            }
        }
    }
    
    void GPSSensor::convert_enu_to_wgs84(const concord::Pose& robot_pose) {
        // This is a simplified conversion - in a real system, you would need
        // proper geodetic transformations using the datum information
        
        // For simulation purposes, assume a local origin and convert ENU to approximate WGS84
        // This should ideally use the world datum that's already available in the system
        
        // Extract ENU coordinates
        double east = robot_pose.point.x;
        double north = robot_pose.point.y;
        double up = robot_pose.point.z;
        
        // Simple conversion (this should use proper geodetic transformations)
        // Assuming a local origin around latitude 45°N for example
        const double origin_lat = 45.0; // degrees
        const double origin_lon = -93.0; // degrees
        const double origin_alt = 300.0; // meters
        
        // Convert ENU to lat/lon (simplified)
        current_data.latitude = origin_lat + (north / 111000.0); // ~111km per degree
        current_data.longitude = origin_lon + (east / (111000.0 * std::cos(origin_lat * M_PI / 180.0)));
        current_data.altitude = origin_alt + up;
        
        // Estimate velocity from position changes (simplified)
        static concord::Pose last_pose = robot_pose;
        static double last_time = last_update_time;
        
        if (last_update_time > last_time) {
            double dt = last_update_time - last_time;
            if (dt > 0) {
                current_data.velocity_north = (north - last_pose.point.y) / dt;
                current_data.velocity_east = (east - last_pose.point.x) / dt;
                current_data.velocity_up = (up - last_pose.point.z) / dt;
            }
        }
        
        last_pose = robot_pose;
        last_time = last_update_time;
    }
    
} // namespace fs
