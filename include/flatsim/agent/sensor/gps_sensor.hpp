#pragma once

#include "flatsim/agent/sensor/sensor.hpp"
#include "flatsim/types.hpp"
#include <chrono>
#include <memory>
#include <wirebit/wirebit.hpp>

namespace fs {
    /**
     * @brief GPS sensor data structure with RTK capabilities
     *
     * Uses double precision for RTK-grade accuracy
     */
    struct GPSData {
        // Position data (double precision for RTK accuracy)
        double latitude = 0.0;  // Degrees
        double longitude = 0.0; // Degrees
        double altitude = 0.0;  // Meters above sea level

        // Velocity data
        double velocity_north = 0.0; // m/s
        double velocity_east = 0.0;  // m/s
        double velocity_up = 0.0;    // m/s

        // Accuracy and status
        double horizontal_accuracy = 0.0; // Meters (standard deviation)
        double vertical_accuracy = 0.0;   // Meters (standard deviation)

        // RTK status
        enum class RTKStatus {
            NO_FIX = 0,
            SINGLE = 1,    // Standard GPS
            DGPS = 2,      // Differential GPS
            RTK_FLOAT = 3, // RTK with float ambiguities
            RTK_FIXED = 4  // RTK with fixed ambiguities (highest accuracy)
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
        double update_frequency; // Hz
        double next_update_time;
        datapod::Pose robot_pose; // Current robot pose

        // RTK simulation parameters
        bool rtk_enabled;
        double base_accuracy; // Base accuracy without RTK (meters)
        double rtk_accuracy;  // RTK accuracy (meters)

        // Noise simulation
        double position_noise_std; // Standard deviation for position noise
        double velocity_noise_std; // Standard deviation for velocity noise

        // NMEA generation
        std::string current_nmea_sentence;
        int nmea_sentence_index = 0; // Cycles through different NMEA types

        // PHTG generation
        bool phtg = false;

        // Flag indicating simulator data is available (skip self-computation)
        bool simulator_data_available_ = false;

        // Datum (reference point for ENU to WGS84 conversion)
        double datum_lat_ = 0.0;
        double datum_lon_ = 0.0;
        double datum_alt_ = 0.0;
        bool datum_set_ = false;

        // Wirebit PTY for serial output (NMEA)
        std::unique_ptr<wirebit::PtyLink> pty_;

        /**
         * @brief Write NMEA string to shared memory
         */
        bool write_to_shm() override;

        /**
         * @brief Get metadata for NMEA format
         */
        std::string get_metadata() const override;

        /**
         * @brief Generate NMEA sentence from current GPS data
         * @param sentence_type Type of NMEA sentence (GGA, RMC, GNS, GST, GSV, etc.)
         * @return Generated NMEA sentence string
         */
        std::string generate_nmea_sentence(const std::string &sentence_type);

        /**
         * @brief Calculate NMEA checksum
         */
        std::string nmea_checksum(const std::string &body) const;

        /**
         * @brief Format latitude for NMEA (ddmm.mmmmmm format)
         */
        std::string format_lat_nmea(double lat_deg, char &hemisphere) const;

        /**
         * @brief Format longitude for NMEA (dddmm.mmmmmm format)
         */
        std::string format_lon_nmea(double lon_deg, char &hemisphere) const;

        /**
         * @brief Get current UTC time string for NMEA
         */
        std::string get_utc_time() const;

        /**
         * @brief Get current UTC date string for NMEA
         */
        std::string get_utc_date() const;

      public:
        /**
         * @brief Construct a new GPS Sensor
         * @param frequency Update frequency in Hz (default: 10 Hz)
         * @param enable_rtk Enable RTK capabilities (default: true)
         * @param base_acc Base GPS accuracy in meters (default: 3.0m)
         * @param rtk_acc RTK accuracy in meters (default: 0.02m)
         */
        GPSSensor(double frequency = 10.0, bool enable_rtk = true, double base_acc = 3.0, double rtk_acc = 0.02);

        virtual ~GPSSensor() = default;

        // Sensor interface implementation
        void update(double dt) override;
        void set_robot_pose(const datapod::Pose &pose) override;
        void update_from_simulator(const types::SensorData &data) override;
        void *get_data() override;
        std::string get_type() const override;
        bool is_data_valid() const override;
        double get_frequency() const override;

        // GPS-specific methods

        /**
         * @brief Get the current GPS data
         * @return Current GPS data structure
         */
        const GPSData &get_gps_data() const;

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

        /**
         * @brief Feed raw NMEA sentence to the sensor
         * @param nmea_sentence Raw NMEA string (e.g. "$GPGGA,...*XX\r\n")
         * @return true if sentence was accepted and written to SHM
         */
        bool feed_nmea(const std::string &nmea_sentence);

        /**
         * @brief Set PHTG status
         */
        void set_phtg_status(bool enable);

        /**
         * @brief Enable serial output via PTY device
         * @param uuid Robot UUID for symlink path (optional)
         * @return PTY path (symlink if uuid provided, else raw /dev/pts/X)
         */
        std::string enable_serial_output(const std::string &uuid = "");

        /**
         * @brief Get PTY path for serial output
         * @return PTY slave path or empty string if not enabled
         */
        std::string get_serial_path() const;

        /**
         * @brief Set datum (reference point) for ENU to WGS84 conversion
         * @param lat Latitude in degrees
         * @param lon Longitude in degrees
         * @param alt Altitude in meters
         */
        void set_datum(double lat, double lon, double alt = 0.0);

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
        void convert_enu_to_wgs84(const datapod::Pose &robot_pose);
    };
} // namespace fs
