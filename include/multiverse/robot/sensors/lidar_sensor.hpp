#pragma once

#include "multiverse/robot/sensor.hpp"
#include "multiverse/types.hpp"
#include "muli/world.h"
#include <chrono>
#include <vector>

namespace mvs {
    /**
     * @brief LIDAR sensor data structure
     * 
     * Contains range measurements and point cloud data
     */
    struct LIDARData {
        // Range measurements
        std::vector<double> ranges;          // Distance measurements (meters)
        std::vector<double> angles;          // Beam angles (radians)
        std::vector<bool> valid;             // Validity flags for each measurement
        
        // Point cloud data (in sensor frame)
        std::vector<double> points_x;        // X coordinates of detected points
        std::vector<double> points_y;        // Y coordinates of detected points
        std::vector<double> points_z;        // Z coordinates of detected points (if 3D LIDAR)
        
        // Sensor configuration
        double min_range = 0.0;              // Minimum detection range (meters)
        double max_range = 0.0;              // Maximum detection range (meters)
        double angular_resolution = 0.0;     // Angular resolution (radians)
        int num_beams = 0;                   // Total number of beams
        
        // Scan timing
        double scan_duration = 0.0;          // Time to complete one full scan (seconds)
        double beam_interval = 0.0;          // Time between individual beam measurements
        
        // Quality metrics
        std::vector<double> intensities;     // Return signal intensities (0-1)
        std::vector<int> num_returns;        // Number of returns per beam (multi-echo)
        
        // Sensor status
        enum class ScanStatus {
            IDLE = 0,
            SCANNING = 1,
            COMPLETE = 2,
            ERROR = 3
        } scan_status = ScanStatus::IDLE;
        
        // Environmental conditions
        double ambient_light = 0.5;          // Ambient light level (affects performance)
        double visibility = 1.0;             // Atmospheric visibility (0-1)
        
        // Time information
        std::chrono::system_clock::time_point timestamp;
        
        // Constructor
        LIDARData() : timestamp(std::chrono::system_clock::now()) {}
        
        /**
         * @brief Clear all measurement data
         */
        void clear() {
            ranges.clear();
            angles.clear();
            valid.clear();
            points_x.clear();
            points_y.clear();
            points_z.clear();
            intensities.clear();
            num_returns.clear();
        }
        
        /**
         * @brief Reserve space for measurements
         */
        void reserve(size_t size) {
            ranges.reserve(size);
            angles.reserve(size);
            valid.reserve(size);
            points_x.reserve(size);
            points_y.reserve(size);
            points_z.reserve(size);
            intensities.reserve(size);
            num_returns.reserve(size);
        }
    };

    /**
     * @brief LIDAR sensor with physics-based raycasting
     * 
     * Simulates a 2D or 3D LIDAR using the Muli physics engine for accurate
     * obstacle detection and range measurements.
     */
    class LIDARSensor : public Sensor {
    public:
        enum class ScanPattern {
            CIRCULAR_2D,        // 360° 2D scan (like Velodyne VLP-16 single layer)
            SECTOR_2D,          // Sector 2D scan (like sick laser scanners)
            MULTI_LAYER_3D,     // Multi-layer 3D scan (like Velodyne VLP-16)
            SPINNING_3D,        // Spinning 3D scan (like Velodyne HDL-64E)
            SOLID_STATE_3D      // Solid-state 3D scan (like Livox)
        };
        
    private:
        LIDARData current_data;
        std::shared_ptr<muli::World> physics_world;
        muli::CollisionFilter filter;
        
        // Sensor configuration
        ScanPattern scan_pattern;
        double update_frequency;        // Hz (scan rate)
        double next_update_time;
        concord::Pose robot_pose;       // Current robot pose
        concord::Pose sensor_offset;    // Sensor offset from robot center
        
        // Range parameters
        double min_range;               // Minimum detection range (meters)
        double max_range;               // Maximum detection range (meters)
        double range_accuracy;          // Range measurement accuracy (meters)
        
        // Angular parameters
        double horizontal_fov;          // Horizontal field of view (radians)
        double vertical_fov;            // Vertical field of view (radians)
        double horizontal_resolution;   // Horizontal angular resolution (radians)
        double vertical_resolution;     // Vertical angular resolution (radians)
        int num_horizontal_beams;       // Number of horizontal beams
        int num_vertical_layers;        // Number of vertical layers (for 3D)
        
        // Scanning state
        double current_scan_angle;      // Current scanning angle for rotating sensors
        int current_beam_index;         // Current beam being measured
        bool scan_in_progress;          // Is a scan currently in progress
        double scan_start_time;         // When current scan started
        
        // Noise and error simulation
        double range_noise_std;         // Range measurement noise std dev
        double beam_divergence;         // Beam divergence angle (radians)
        double dropout_probability;     // Probability of measurement dropout
        double false_positive_rate;     // Rate of false positive detections
        
        // Environmental effects
        bool simulate_weather;          // Enable weather simulation
        double rain_intensity;          // Rain intensity (0-1)
        double fog_density;             // Fog density (0-1)
        double dust_level;              // Dust level (0-1)
        
    public:
        /**
         * @brief Construct a new LIDAR Sensor
         * @param world Pointer to the physics world for raycasting
         * @param pattern Scan pattern type
         * @param frequency Update frequency in Hz (default: 10 Hz)
         * @param min_r Minimum range in meters (default: 0.1m)
         * @param max_r Maximum range in meters (default: 30m)
         * @param h_fov Horizontal FOV in degrees (default: 360°)
         * @param h_res Horizontal resolution in degrees (default: 1°)
         */
        LIDARSensor(std::shared_ptr<muli::World> world,
                   ScanPattern pattern = ScanPattern::CIRCULAR_2D,
                   double frequency = 10.0,
                   double min_r = 0.1,
                   double max_r = 30.0,
                   double h_fov = 360.0,
                   double h_res = 1.0);
        
        virtual ~LIDARSensor() = default;
        
        // Sensor interface implementation
        void update(double dt) override;
        void set_robot_pose(const concord::Pose& pose) override;
        void* get_data() override;
        std::string get_type() const override;
        bool is_data_valid() const override;
        double get_frequency() const override;
        
        // LIDAR-specific methods
        
        /**
         * @brief Get the current LIDAR data
         * @return Current LIDAR data structure
         */
        const LIDARData& get_lidar_data() const;
        
        /**
         * @brief Set sensor offset from robot center
         * @param offset Pose offset of sensor relative to robot
         */
        void set_sensor_offset(const concord::Pose& offset);
        
        /**
         * @brief Configure range parameters
         * @param min_r Minimum range (meters)
         * @param max_r Maximum range (meters)
         * @param accuracy Range accuracy (meters)
         */
        void configure_range(double min_r, double max_r, double accuracy = 0.01);
        
        /**
         * @brief Configure angular parameters
         * @param h_fov Horizontal FOV (degrees)
         * @param v_fov Vertical FOV (degrees, for 3D)
         * @param h_res Horizontal resolution (degrees)
         * @param v_res Vertical resolution (degrees, for 3D)
         */
        void configure_angular(double h_fov, double v_fov = 0.0, 
                              double h_res = 1.0, double v_res = 1.0);
        
        /**
         * @brief Configure noise parameters
         * @param range_noise Range noise std dev (meters)
         * @param beam_div Beam divergence (degrees)
         * @param dropout Dropout probability (0-1)
         */
        void configure_noise(double range_noise = 0.01, 
                            double beam_div = 0.1, 
                            double dropout = 0.01);
        
        /**
         * @brief Set environmental conditions
         * @param rain Rain intensity (0-1)
         * @param fog Fog density (0-1)
         * @param dust Dust level (0-1)
         */
        void set_environmental_conditions(double rain = 0.0, 
                                         double fog = 0.0, 
                                         double dust = 0.0);
        
        /**
         * @brief Enable/disable weather simulation
         * @param enable Enable weather effects
         */
        void enable_weather_simulation(bool enable);
        
        /**
         * @brief Set collision filter for raycasting
         * @param filter Collision filter to use
         */
        void set_collision_filter(const muli::CollisionFilter& filter);
        
        /**
         * @brief Force start a new scan
         */
        void start_scan();
        
        /**
         * @brief Check if sensor is currently scanning
         * @return true if scan is in progress
         */
        bool is_scanning() const;
        
        /**
         * @brief Get scan progress (0-1)
         * @return Scan completion percentage
         */
        double get_scan_progress() const;
        
        /**
         * @brief Convert LIDAR data to point cloud
         * @param points Output vector for 3D points
         * @param transform_to_world If true, transform points to world frame
         */
        void get_point_cloud(std::vector<muli::Vec2>& points, bool transform_to_world = false) const;

    private:
        /**
         * @brief Perform raycasting for a single beam
         * @param angle Beam angle (radians)
         * @param layer_height Height for 3D scanning (meters)
         * @return Range measurement (meters), or max_range if no hit
         */
        double raycast_beam(double angle, double layer_height = 0.0);
        
        /**
         * @brief Update 2D circular scan
         */
        void update_circular_2d_scan(double dt);
        
        /**
         * @brief Update 2D sector scan
         */
        void update_sector_2d_scan(double dt);
        
        /**
         * @brief Update 3D multi-layer scan
         */
        void update_multi_layer_3d_scan(double dt);
        
        /**
         * @brief Apply environmental effects to range measurement
         * @param range Raw range measurement
         * @param angle Beam angle
         * @return Modified range measurement
         */
        double apply_environmental_effects(double range, double angle);
        
        /**
         * @brief Add measurement noise
         * @param range Clean range measurement
         * @return Noisy range measurement
         */
        double add_measurement_noise(double range);
        
        /**
         * @brief Check if measurement should be dropped
         * @return true if measurement should be marked invalid
         */
        bool should_drop_measurement();
        
        /**
         * @brief Calculate beam position in world coordinates
         * @param angle Beam angle (radians)
         * @param range Range (meters)
         * @param height Height for 3D (meters)
         * @return World position
         */
        muli::Vec2 calculate_world_position(double angle, double range, double height = 0.0) const;
        
        /**
         * @brief Calculate sensor position in world coordinates
         * @return Sensor world position
         */
        muli::Vec2 get_sensor_world_position() const;
        
        /**
         * @brief Calculate sensor orientation in world frame
         * @return Sensor world orientation (radians)
         */
        double get_sensor_world_orientation() const;
        
        /**
         * @brief Initialize scan parameters based on pattern
         */
        void initialize_scan_parameters();
    };
} // namespace mvs
