#pragma once

#include "concord/concord.hpp"
#include "flatsim/agent/sensor/sensor.hpp"
#include "flatsim/types.hpp"
#include <chrono>
#include <vector>

namespace fs {
    /**
     * @brief LIDAR sensor data structure
     *
     * Contains range measurements and point cloud data
     */
    struct LIDARData {
        // Range measurements
        std::vector<float> ranges; // Distance measurements (meters)
        std::vector<float> angles; // Beam angles (radians)
        std::vector<bool> valid;   // Validity flags for each measurement

        // Sensor configuration
        float min_range = 0.1f;  // Minimum detection range (meters)
        float max_range = 30.0f; // Maximum detection range (meters)
        int num_beams = 0;       // Total number of beams

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
        }

        /**
         * @brief Reserve space for measurements
         */
        void reserve(size_t size) {
            ranges.reserve(size);
            angles.reserve(size);
            valid.reserve(size);
        }
    };

    /**
     * @brief LIDAR sensor that receives data from simulator
     *
     * This sensor does NOT do its own raycasting. It receives LIDAR scan
     * data computed by the simulator's physics engine.
     */
    class LIDARSensor : public Sensor {
      public:
        enum class ScanPattern {
            CIRCULAR_2D, // 360° 2D scan
            SECTOR_2D,   // Sector 2D scan (e.g., 45° FOV)
        };

      private:
        LIDARData current_data;

        // Sensor configuration
        ScanPattern scan_pattern;
        double update_frequency; // Hz (scan rate)
        double next_update_time;
        concord::Pose robot_pose; // Current robot pose

        // Range parameters
        float min_range; // Minimum detection range (meters)
        float max_range; // Maximum detection range (meters)

        // Angular parameters
        float horizontal_fov;        // Horizontal field of view (radians)
        float horizontal_resolution; // Horizontal angular resolution (radians)
        int num_horizontal_beams;    // Number of horizontal beams

        // Flag indicating simulator data is available
        bool simulator_data_available_ = false;

      public:
        /**
         * @brief Construct a new LIDAR Sensor
         * @param pattern Scan pattern type
         * @param frequency Update frequency in Hz (default: 10 Hz)
         * @param min_r Minimum range in meters (default: 0.1m)
         * @param max_r Maximum range in meters (default: 30m)
         * @param h_fov Horizontal FOV in degrees (default: 360°)
         * @param h_res Horizontal resolution in degrees (default: 1°)
         */
        LIDARSensor(ScanPattern pattern = ScanPattern::SECTOR_2D, double frequency = 10.0, float min_r = 0.1f,
                    float max_r = 30.0f, float h_fov = 45.0f, float h_res = 3.0f);

        virtual ~LIDARSensor() = default;

        // Sensor interface implementation
        void update(double dt) override;
        void set_robot_pose(const concord::Pose &pose) override;
        void update_from_simulator(const types::SensorData &data) override;
        void *get_data() override;
        std::string get_type() const override;
        bool is_data_valid() const override;
        double get_frequency() const override;

        // LIDAR-specific methods

        /**
         * @brief Get the current LIDAR data
         * @return Current LIDAR data structure
         */
        const LIDARData &get_lidar_data() const;

        /**
         * @brief Configure range parameters
         * @param min_r Minimum range (meters)
         * @param max_r Maximum range (meters)
         */
        void configure_range(float min_r, float max_r);

        /**
         * @brief Configure angular parameters
         * @param h_fov Horizontal FOV (degrees)
         * @param h_res Horizontal resolution (degrees)
         */
        void configure_angular(float h_fov, float h_res);

        /**
         * @brief Get LIDAR configuration for simulator
         *
         * The simulator needs to know the LIDAR configuration to compute
         * the scan. This returns the configuration in a format the simulator
         * can use.
         */
        float get_fov_deg() const { return horizontal_fov * 180.0f / M_PI; }
        float get_resolution_deg() const { return horizontal_resolution * 180.0f / M_PI; }
        float get_min_range() const { return min_range; }
        float get_max_range() const { return max_range; }

      private:
        /**
         * @brief Initialize scan parameters based on pattern
         */
        void initialize_scan_parameters();

        /**
         * @brief Write LIDAR data to shared memory in binary format
         */
        bool write_to_shm() override;

        /**
         * @brief Get metadata describing LIDAR binary format
         */
        std::string get_metadata() const override;
    };
} // namespace fs
