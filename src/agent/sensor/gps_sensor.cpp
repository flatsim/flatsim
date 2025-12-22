#include "flatsim/agent/sensor/gps_sensor.hpp"
#include <cmath>
#include <random>

namespace fs {

    GPSSensor::GPSSensor(double frequency, bool enable_rtk, double base_acc, double rtk_acc)
        : update_frequency(frequency), next_update_time(0.0), rtk_enabled(enable_rtk), base_accuracy(base_acc),
          rtk_accuracy(rtk_acc), position_noise_std(0.1), velocity_noise_std(0.05) {
        current_data.num_satellites = 8; // Default satellite count
        update_rtk_status();
    }

    void GPSSensor::update(double dt) {
        last_update_time += dt;

        // If simulator data is available, use it. Otherwise, compute from pose.
        if (simulator_data_available_) {
            // Data already set by update_from_simulator(), just add noise
            add_measurement_noise();
            simulator_data_available_ = false; // Reset for next tick
        } else {
            // Fallback: Convert robot pose to GPS coordinates (self-computation)
            convert_enu_to_wgs84(robot_pose);
            add_measurement_noise();
        }

        // Update RTK status
        update_rtk_status();

        // Update timestamp
        current_data.timestamp = std::chrono::system_clock::now();

        // Mark data as valid
        data_valid = true;

        // Generate ALL NMEA sentences and concatenate them (each has \r\n already)
        const char *sentence_types[] = {"GGA", "RMC", "GNS", "GST", "GSV", "PHTG"};
        current_nmea_sentence.clear();
        for (const char *type : sentence_types) {
            std::string sentence = generate_nmea_sentence(type);
            if (!sentence.empty()) {
                current_nmea_sentence += sentence; // Each sentence already has \r\n
            }
        }

        // Write all sentences at once to shared memory
        if (shm_enabled && !current_nmea_sentence.empty()) {
            write_to_shm();
        }
    }

    bool GPSSensor::write_to_shm() {
        if (!is_data_valid() || !is_shm_enabled()) {
            return false;
        }

        if (current_nmea_sentence.empty()) {
            return false;
        }

        // Write raw NMEA string to shared memory
        return write_shm_data(current_nmea_sentence.c_str(), current_nmea_sentence.size());
    }

    std::string GPSSensor::get_metadata() const {
        std::string metadata;
        metadata += "GPS/GNSS NMEA Format Description\n";
        metadata += "================================\n\n";
        metadata += "Format: Raw NMEA-0183 strings\n";
        metadata += "Encoding: ASCII text\n\n";
        metadata += "Structure:\n";
        metadata += "----------\n";
        metadata += "Each update contains a single NMEA sentence as a null-terminated ASCII string.\n\n";
        metadata += "Common NMEA sentence types:\n";
        metadata += "  - $GPGGA: Global Positioning System Fix Data\n";
        metadata += "  - $GPRMC: Recommended Minimum Specific GNSS Data\n";
        metadata += "  - $GNGNS: GNSS Fix Data (combined constellations)\n";
        metadata += "  - $GPGST: Position Error Statistics\n\n";
        metadata += "Example sentences:\n";
        metadata += "  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n";
        metadata += "  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\n\n";
        metadata += "Reading from shared memory:\n";
        metadata += "---------------------------\n";
        metadata += "The data following the header is a null-terminated ASCII string.\n";
        metadata += "Example C code:\n";
        metadata += "  char nmea[256];\n";
        metadata += "  memcpy(nmea, shm_data_ptr, data_size);\n";
        metadata += "  nmea[data_size] = '\\0';\n";
        metadata += "  printf(\"NMEA: %s\\n\", nmea);\n";
        return metadata;
    }

    std::string GPSSensor::nmea_checksum(const std::string &body) const {
        uint8_t cs = 0;
        for (char ch : body) {
            cs ^= static_cast<uint8_t>(ch);
        }
        char buf[3];
        snprintf(buf, sizeof(buf), "%02X", cs);
        return std::string(buf);
    }

    std::string GPSSensor::format_lat_nmea(double lat_deg, char &hemisphere) const {
        hemisphere = (lat_deg >= 0) ? 'N' : 'S';
        double lat_abs = std::abs(lat_deg);
        int deg = static_cast<int>(lat_abs);
        double minutes = (lat_abs - deg) * 60.0;

        char buf[32];
        snprintf(buf, sizeof(buf), "%02d%011.8f", deg, minutes);
        return std::string(buf);
    }

    std::string GPSSensor::format_lon_nmea(double lon_deg, char &hemisphere) const {
        hemisphere = (lon_deg >= 0) ? 'E' : 'W';
        double lon_abs = std::abs(lon_deg);
        int deg = static_cast<int>(lon_abs);
        double minutes = (lon_abs - deg) * 60.0;

        char buf[32];
        snprintf(buf, sizeof(buf), "%03d%011.8f", deg, minutes);
        return std::string(buf);
    }

    std::string GPSSensor::get_utc_time() const {
        auto now = std::chrono::system_clock::now();
        time_t tt = std::chrono::system_clock::to_time_t(now);
        tm utc_tm;
        gmtime_r(&tt, &utc_tm);

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        char buf[16];
        snprintf(buf, sizeof(buf), "%02d%02d%02d.%03d", utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec,
                 static_cast<int>(ms.count()));
        return std::string(buf);
    }

    std::string GPSSensor::get_utc_date() const {
        auto now = std::chrono::system_clock::now();
        time_t tt = std::chrono::system_clock::to_time_t(now);
        tm utc_tm;
        gmtime_r(&tt, &utc_tm);

        char buf[8];
        snprintf(buf, sizeof(buf), "%02d%02d%02d", utc_tm.tm_mday, utc_tm.tm_mon + 1, utc_tm.tm_year % 100);
        return std::string(buf);
    }

    std::string GPSSensor::generate_nmea_sentence(const std::string &sentence_type) {
        char lat_hemi, lon_hemi;
        std::string lat_str = format_lat_nmea(current_data.latitude, lat_hemi);
        std::string lon_str = format_lon_nmea(current_data.longitude, lon_hemi);
        std::string time_str = get_utc_time();
        std::string date_str = get_utc_date();

        std::string body;

        if (sentence_type == "GGA") {
            // $GPGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,geoidSep,M,,*checksum
            int fix_quality = (current_data.rtk_status == GPSData::RTKStatus::RTK_FIXED)   ? 4
                              : (current_data.rtk_status == GPSData::RTKStatus::RTK_FLOAT) ? 5
                                                                                           : 1;

            char buf[256];
            snprintf(buf, sizeof(buf), "GPGGA,%s,%s,%c,%s,%c,%d,%02d,%.1f,%.2f,M,0.0,M,,", time_str.c_str(),
                     lat_str.c_str(), lat_hemi, lon_str.c_str(), lon_hemi, fix_quality, current_data.num_satellites,
                     0.8, current_data.altitude);
            body = buf;

        } else if (sentence_type == "RMC") {
            // $GPRMC,time,status,lat,N/S,lon,E/W,speed,track,date,magvar,*checksum
            double speed_knots = std::sqrt(current_data.velocity_north * current_data.velocity_north +
                                           current_data.velocity_east * current_data.velocity_east) *
                                 1.94384; // m/s to knots
            double track = std::atan2(current_data.velocity_east, current_data.velocity_north) * 180.0 / M_PI;
            if (track < 0) track += 360.0;

            char buf[256];
            snprintf(buf, sizeof(buf), "GPRMC,%s,A,%s,%c,%s,%c,%.1f,%.1f,%s,,", time_str.c_str(), lat_str.c_str(),
                     lat_hemi, lon_str.c_str(), lon_hemi, speed_knots, track, date_str.c_str());
            body = buf;

        } else if (sentence_type == "GNS") {
            // $GNGNS,time,lat,N/S,lon,E/W,mode,numSV,HDOP,alt,sep,,,*checksum
            char buf[256];
            snprintf(buf, sizeof(buf), "GNGNS,%s,%s,%c,%s,%c,RRNNN,%.1f,%.2f,0.0,,,", time_str.c_str(), lat_str.c_str(),
                     lat_hemi, lon_str.c_str(), lon_hemi, 0.8, current_data.altitude);
            body = buf;

        } else if (sentence_type == "GST") {
            // $GPGST,time,rms,major,minor,orient,lat_err,lon_err,alt_err*checksum
            char buf[256];
            snprintf(
                buf, sizeof(buf), "GPGST,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", time_str.c_str(),
                current_data.horizontal_accuracy, current_data.horizontal_accuracy, current_data.horizontal_accuracy,
                0.0, // orientation
                current_data.horizontal_accuracy, current_data.horizontal_accuracy, current_data.vertical_accuracy);
            body = buf;

        } else if (sentence_type == "GSV") {
            // $GPGSV,numMsgs,msgNum,numSats,satId,elev,azim,snr,...*checksum
            // Simple single message with one satellite
            char buf[256];
            snprintf(buf, sizeof(buf), "GPGSV,1,1,%02d,11,45,120,40", current_data.num_satellites);
            body = buf;

        } else if (sentence_type == "PHTG") {
            // $PHTG,date,time,System,Service,AuthResult,Status*checksum
            // Date: dd:mm:yyyy, Time: hh:mm:ss.ss
            auto now = std::chrono::system_clock::now();
            time_t tt = std::chrono::system_clock::to_time_t(now);
            tm utc_tm;
            gmtime_r(&tt, &utc_tm);

            char date_str[32];
            char time_str[32];
            snprintf(date_str, sizeof(date_str), "%02d:%02d:%04d", utc_tm.tm_mday, utc_tm.tm_mon + 1,
                     utc_tm.tm_year + 1900);
            snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d.00", utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec);

            int pth_status = phtg ? 1 : 0;

            char buf[256];
            snprintf(buf, sizeof(buf), "PHTG,%s,%s,GAL,HAS,%d,0", date_str, time_str, pth_status);
            body = buf;

        } else {
            return ""; // Unknown sentence type
        }

        std::string checksum = nmea_checksum(body);
        return "$" + body + "*" + checksum + "\r\n";
    }

    void GPSSensor::set_robot_pose(const concord::Pose &pose) { robot_pose = pose; }

    void GPSSensor::update_from_simulator(const types::SensorData &data) {
        if (!data.has_gps) {
            return;
        }

        // Use GPS data computed by simulator (already in WGS84)
        current_data.latitude = data.gps.latitude;
        current_data.longitude = data.gps.longitude;
        current_data.altitude = data.gps.altitude;

        // Compute velocity from heading and speed
        double heading_rad = static_cast<double>(data.gps.heading);
        double speed = static_cast<double>(data.gps.speed);
        current_data.velocity_north = speed * std::cos(heading_rad);
        current_data.velocity_east = speed * std::sin(heading_rad);
        current_data.velocity_up = 0.0;

        // Mark that we have simulator data (skip self-computation in update())
        simulator_data_available_ = true;
    }

    void *GPSSensor::get_data() { return &current_data; }

    std::string GPSSensor::get_type() const { return "GPS"; }

    bool GPSSensor::is_data_valid() const { return data_valid; }

    void GPSSensor::set_phtg_status(bool enable) { phtg = enable; }

    double GPSSensor::get_frequency() const { return update_frequency; }

    const GPSData &GPSSensor::get_gps_data() const { return current_data; }

    void GPSSensor::set_rtk_available(bool available) {
        rtk_enabled = available;
        update_rtk_status();
    }

    GPSData::RTKStatus GPSSensor::get_rtk_status() const { return current_data.rtk_status; }

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

    void GPSSensor::convert_enu_to_wgs84(const concord::Pose &robot_pose) {
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
        const double origin_lat = 45.0;  // degrees
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
