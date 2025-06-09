#pragma once

#include "multiverse/robot/sensor.hpp"
#include "multiverse/types.hpp"
#include <chrono>

namespace mvs {
    /**
     * @brief GPS sensor data structure with RTK capabilities
     * 
     * Uses double precision for RTK-grade accuracy
     */
    struct GPSData {
        // Position data (double precision for RTK accuracy)
        double latitude = 0.0;          // Degrees
        double longitude = 0.0;         // Degrees  
        double altitude = 0.0;          // Meters above sea level
        
        // Velocity data
        double velocity_north = 0.0;    // m/s
        double velocity_east = 0.0;     // m/s
        double velocity_up = 0.0;       // m/s
        
        // Accuracy and status
        double horizontal_accuracy = 0.0;  // Meters (standard deviation)
        double vertical_accuracy = 0.0;    // Meters (standard deviation)
        
        // RTK status
        enum class RTKStatus {
            NO_FIX = 0,
            SINGLE = 1,         // Standard GPS
            DGPS = 2,           // Differential GPS
            RTK_FLOAT = 3,      // RTK with float ambiguities
            RTK_FIXED = 4       // RTK with fixed ambiguities (highest accuracy)
        } rtk_status = RTKStatus::NO_FIX;
        
        // Number of satellites
        int num_satellites = 0;
        
        // Time information
        std::chrono::system_clock::time_point timestamp;
        
        // Constructor
        GPSData() : timestamp(std::chrono::system_clock::now()) {}
    };

    /**
     * @brief GPS sensor with RTK capabilities
     * 
     * Simulates a high-precision GPS sensor with RTK correction capabilities.
     * In RTK mode, provides centimeter-level accuracy.
     */
    class GPSSensor : public Sensor {
    private:
        GPSData current_data;
        double update_frequency;    // Hz
        double next_update_time;
        concord::Pose robot_pose;   // Current robot pose
        
        // RTK simulation parameters
        bool rtk_enabled;
        double base_accuracy;       // Base accuracy without RTK (meters)
        double rtk_accuracy;        // RTK accuracy (meters)
        
        // Noise simulation
        double position_noise_std;  // Standard deviation for position noise
        double velocity_noise_std;  // Standard deviation for velocity noise
        
    public:
        /**
         * @brief Construct a new GPS Sensor
         * @param frequency Update frequency in Hz (default: 10 Hz)
         * @param enable_rtk Enable RTK capabilities (default: true)
         * @param base_acc Base GPS accuracy in meters (default: 3.0m)
         * @param rtk_acc RTK accuracy in meters (default: 0.02m)
         */
        GPSSensor(double frequency = 10.0, bool enable_rtk = true, 
                 double base_acc = 3.0, double rtk_acc = 0.02);
        
        virtual ~GPSSensor() = default;
        
        // Sensor interface implementation
        void update(double dt) override;
        void set_robot_pose(const concord::Pose& pose) override;
        void* get_data() override;
        std::string get_type() const override;
        bool is_data_valid() const override;
        double get_frequency() const override;
        
        // GPS-specific methods
        
        /**
         * @brief Get the current GPS data
         * @return Current GPS data structure
         */
        const GPSData& get_gps_data() const;
        
        /**
         * @brief Set RTK base station availability
         * @param available true if RTK base station is available
         */
        void set_rtk_available(bool available);
        
        /**
         * @brief Get current RTK status
         * @return Current RTK status
         */
        GPSData::RTKStatus get_rtk_status() const;
        
        /**
         * @brief Set the number of visible satellites
         * @param count Number of satellites (affects accuracy)
         */
        void set_satellite_count(int count);
        
        /**
         * @brief Configure noise parameters
         * @param pos_noise Position noise standard deviation (meters)
         * @param vel_noise Velocity noise standard deviation (m/s)
         */
        void configure_noise(double pos_noise, double vel_noise);

    private:
        /**
         * @brief Add realistic noise to GPS measurements
         */
        void add_measurement_noise();
        
        /**
         * @brief Update RTK status based on conditions
         */
        void update_rtk_status();
        
        /**
         * @brief Convert ENU coordinates to WGS84
         */
        void convert_enu_to_wgs84(const concord::Pose& robot_pose);
    };
} // namespace mvs
