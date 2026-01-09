#include "flatsim/agent/sensor/lidar_sensor.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <echo/echo.hpp>
#include <unistd.h>

namespace fs {

    LIDARSensor::LIDARSensor(ScanPattern pattern, double frequency, float min_r, float max_r, float h_fov, float h_res)
        : scan_pattern(pattern), update_frequency(frequency), next_update_time(0.0), min_range(min_r), max_range(max_r),
          horizontal_fov(h_fov * M_PI / 180.0f), horizontal_resolution(h_res * M_PI / 180.0f) {
        initialize_scan_parameters();
    }

    void LIDARSensor::update(double dt) {
        last_update_time += dt;

        // Check if it's time for an update
        if (last_update_time >= next_update_time) {
            // If simulator data is available, it's already been set via update_from_simulator()
            if (simulator_data_available_) {
                simulator_data_available_ = false; // Reset for next tick
            }
            // Otherwise, data remains empty/stale (agent has no physics access)

            // Update timestamp
            current_data.timestamp = std::chrono::system_clock::now();

            // Mark data as valid if we have any beams
            data_valid = !current_data.ranges.empty();

            // Write to SHM if enabled
            if (shm_enabled && data_valid) {
                write_to_shm();
            }

            // Write binary to PTY serial output
            if (pty_ && data_valid && !current_data.ranges.empty()) {
                uint32_t num_points = static_cast<uint32_t>(current_data.ranges.size());
                size_t ranges_size = num_points * sizeof(float);
                size_t angles_size = num_points * sizeof(float);
                size_t total_size = sizeof(uint32_t) + ranges_size + angles_size;

                std::vector<uint8_t> buf(total_size);
                uint8_t *ptr = buf.data();

                std::memcpy(ptr, &num_points, sizeof(uint32_t));
                ptr += sizeof(uint32_t);
                std::memcpy(ptr, current_data.ranges.data(), ranges_size);
                ptr += ranges_size;
                std::memcpy(ptr, current_data.angles.data(), angles_size);

                ::write(pty_->master_fd(), buf.data(), total_size);
            }

            // Schedule next update
            next_update_time = last_update_time + (1.0 / update_frequency);
        }
    }

    void LIDARSensor::set_robot_pose(const datapod::Pose &pose) { robot_pose = pose; }

    void LIDARSensor::update_from_simulator(const types::SensorData &data) {
        if (!data.has_lidar) {
            return;
        }

        // Copy LIDAR data from simulator
        current_data.ranges.clear();
        current_data.angles.clear();
        current_data.valid.clear();

        for (const auto &r : data.lidar.ranges) {
            current_data.ranges.push_back(r);
        }
        for (const auto &a : data.lidar.angles) {
            current_data.angles.push_back(a);
        }
        for (const auto &v : data.lidar.valid) {
            current_data.valid.push_back(v);
        }

        current_data.min_range = data.lidar.min_range;
        current_data.max_range = data.lidar.max_range;
        current_data.num_beams = static_cast<int>(current_data.ranges.size());

        // Mark that we have simulator data
        simulator_data_available_ = true;
    }

    void *LIDARSensor::get_data() { return &current_data; }

    std::string LIDARSensor::get_type() const { return "LIDAR"; }

    bool LIDARSensor::is_data_valid() const { return data_valid && !current_data.ranges.empty(); }

    double LIDARSensor::get_frequency() const { return update_frequency; }

    const LIDARData &LIDARSensor::get_lidar_data() const { return current_data; }

    void LIDARSensor::configure_range(float min_r, float max_r) {
        min_range = min_r;
        max_range = max_r;
        current_data.min_range = min_range;
        current_data.max_range = max_range;
    }

    void LIDARSensor::configure_angular(float h_fov, float h_res) {
        horizontal_fov = h_fov * M_PI / 180.0f;
        horizontal_resolution = h_res * M_PI / 180.0f;
        initialize_scan_parameters();
    }

    void LIDARSensor::initialize_scan_parameters() {
        // Calculate number of beams based on FOV and resolution
        num_horizontal_beams = static_cast<int>(std::ceil(horizontal_fov / horizontal_resolution));
        if (num_horizontal_beams < 1) num_horizontal_beams = 1;

        // Update data structure
        current_data.num_beams = num_horizontal_beams;
        current_data.min_range = min_range;
        current_data.max_range = max_range;
    }

    bool LIDARSensor::write_to_shm() {
        if (!is_data_valid() || !is_shm_enabled()) {
            return false;
        }

        uint32_t num_ranges = static_cast<uint32_t>(current_data.ranges.size());
        if (num_ranges == 0) {
            return false;
        }

        // Binary format: header + data arrays
        size_t header_size = sizeof(uint32_t);
        size_t ranges_size = num_ranges * sizeof(float);
        size_t angles_size = num_ranges * sizeof(float);
        size_t valid_size = num_ranges * sizeof(uint8_t);
        size_t total_size = header_size + ranges_size + angles_size + valid_size;

        // Allocate buffer
        std::vector<uint8_t> buffer(total_size);
        uint8_t *ptr = buffer.data();

        // Write header
        std::memcpy(ptr, &num_ranges, sizeof(uint32_t));
        ptr += sizeof(uint32_t);

        // Write ranges array
        std::memcpy(ptr, current_data.ranges.data(), ranges_size);
        ptr += ranges_size;

        // Write angles array
        std::memcpy(ptr, current_data.angles.data(), angles_size);
        ptr += angles_size;

        // Write valid array (convert bool to uint8_t)
        for (size_t i = 0; i < num_ranges; ++i) {
            uint8_t valid_byte = current_data.valid[i] ? 1 : 0;
            std::memcpy(ptr, &valid_byte, sizeof(uint8_t));
            ptr += sizeof(uint8_t);
        }

        return write_shm_data(buffer.data(), total_size);
    }

    std::string LIDARSensor::get_metadata() const {
        std::string metadata;
        metadata += "LIDAR Binary Format Description\n";
        metadata += "================================\n\n";
        metadata += "Byte order: Little-endian\n";
        metadata += "Floating point: IEEE 754 single precision (4 bytes)\n\n";
        metadata += "Structure:\n";
        metadata += "----------\n";
        metadata += "Header:\n";
        metadata += "  - num_ranges: uint32_t (4 bytes) - Number of range measurements\n\n";
        metadata += "Data Arrays (all of length num_ranges):\n";
        metadata += "  1. ranges: float[] - Distance measurements in meters\n";
        metadata += "     - Offset: 4 bytes\n";
        metadata += "     - Size: num_ranges * 4 bytes\n\n";
        metadata += "  2. angles: float[] - Beam angles in radians (relative to heading)\n";
        metadata += "     - Offset: 4 + (num_ranges * 4) bytes\n";
        metadata += "     - Size: num_ranges * 4 bytes\n\n";
        metadata += "  3. valid: uint8_t[] - Validity flags (0=no hit, 1=valid hit)\n";
        metadata += "     - Offset: 4 + (num_ranges * 8) bytes\n";
        metadata += "     - Size: num_ranges bytes\n\n";
        metadata += "Total size: 4 + (num_ranges * 9) bytes\n";
        return metadata;
    }

    std::string LIDARSensor::enable_serial_output(const std::string &uuid) {
        auto res = wirebit::PtyLink::create();
        if (res.is_err()) {
            echo::error("[LIDARSensor] Failed to create PTY: ", res.error().message.c_str()).red();
            return "";
        }
        pty_ = std::make_unique<wirebit::PtyLink>(std::move(res.value()));

        std::string path = std::string(pty_->slave_path().c_str());

        // Create symlink if UUID provided
        if (!uuid.empty()) {
            std::string symlink_path = "/tmp/flatsim/" + uuid + "/lidar";
            std::string dir = "/tmp/flatsim/" + uuid;

            std::system(("mkdir -p " + dir).c_str());
            ::unlink(symlink_path.c_str());

            if (::symlink(path.c_str(), symlink_path.c_str()) == 0) {
                path = symlink_path;
            }
        }

        echo::trace("[LIDARSensor] Serial output: ", path.c_str()).green();
        return path;
    }

    std::string LIDARSensor::get_serial_path() const { return pty_ ? std::string(pty_->slave_path().c_str()) : ""; }

} // namespace fs
