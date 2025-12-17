#include "flatsim/robot/sensor/sensor.hpp"
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace fs {

    // Shared memory header structure
    struct ShmHeader {
        uint64_t sequence_number; // Increments each write
        uint64_t timestamp_ns;    // Nanoseconds since epoch
        uint32_t data_size;       // Size of data following this header
        uint32_t reserved;        // Padding for alignment
    };

    bool Sensor::enable_shm_output(const std::string &robot_uuid) {
        if (shm_enabled) {
            return true; // Already enabled
        }

        // Create shared memory name: /flatsim_<robot_uuid>_<sensor_type>
        std::string sensor_type = get_type();
        shm_name = "/flatsim_" + robot_uuid + "_" + sensor_type;

        // Create metadata file path in /tmp
        std::string base_dir = "/tmp/flatsim_" + robot_uuid;
        if (mkdir(base_dir.c_str(), 0755) == -1 && errno != EEXIST) {
            return false;
        }
        metadata_path = base_dir + "/" + sensor_type + ".format";

        // Calculate total shared memory size (header + max data)
        shm_size = sizeof(ShmHeader) + MAX_SHM_SIZE;

        // Create or open shared memory object
        shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            return false;
        }

        // Set size
        if (ftruncate(shm_fd, shm_size) == -1) {
            close(shm_fd);
            shm_fd = -1;
            shm_unlink(shm_name.c_str());
            return false;
        }

        // Map shared memory
        shm_ptr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_ptr == MAP_FAILED) {
            close(shm_fd);
            shm_fd = -1;
            shm_unlink(shm_name.c_str());
            shm_ptr = nullptr;
            return false;
        }

        // Initialize header
        ShmHeader *header = static_cast<ShmHeader *>(shm_ptr);
        header->sequence_number = 0;
        header->timestamp_ns = 0;
        header->data_size = 0;
        header->reserved = 0;

        // Create metadata file
        if (!update_metadata_file()) {
            // Non-fatal, continue anyway
        }

        shm_enabled = true;
        sequence_number = 0;
        return true;
    }

    void Sensor::disable_shm_output() {
        if (!shm_enabled) {
            return;
        }

        if (shm_ptr != nullptr && shm_ptr != MAP_FAILED) {
            munmap(shm_ptr, shm_size);
            shm_ptr = nullptr;
        }

        if (shm_fd != -1) {
            close(shm_fd);
            shm_fd = -1;
        }

        if (!shm_name.empty()) {
            shm_unlink(shm_name.c_str());
            shm_name.clear();
        }

        if (!metadata_path.empty()) {
            unlink(metadata_path.c_str());
            metadata_path.clear();
        }

        shm_enabled = false;
    }

    bool Sensor::write_shm_data(const void *data, size_t size) {
        if (!shm_enabled || data == nullptr || size == 0) {
            return false;
        }

        if (shm_ptr == nullptr || shm_ptr == MAP_FAILED) {
            return false;
        }

        // Check size limit
        if (size > MAX_SHM_SIZE) {
            return false;
        }

        // Get current time
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();

        // Update header
        ShmHeader *header = static_cast<ShmHeader *>(shm_ptr);
        header->data_size = static_cast<uint32_t>(size);
        header->timestamp_ns = timestamp_ns;

        // Copy data after header
        void *data_ptr = static_cast<uint8_t *>(shm_ptr) + sizeof(ShmHeader);
        std::memcpy(data_ptr, data, size);

        // Update sequence number last (atomic-ish write indicator)
        header->sequence_number = ++sequence_number;

        return true;
    }

    bool Sensor::update_metadata_file() {
        if (metadata_path.empty()) {
            return false;
        }

        std::string metadata = get_metadata();
        if (metadata.empty()) {
            return false;
        }

        // Add shared memory information to metadata
        std::string full_metadata;
        full_metadata += "Shared Memory Segment\n";
        full_metadata += "=====================\n";
        full_metadata += "Name: " + shm_name + "\n";
        full_metadata += "Location: /dev/shm" + shm_name + "\n";
        full_metadata += "Total size: " + std::to_string(shm_size) + " bytes\n\n";
        full_metadata += "Header Structure (16 bytes):\n";
        full_metadata += "  - sequence_number: uint64_t (8 bytes) - Increments each update\n";
        full_metadata += "  - timestamp_ns: uint64_t (8 bytes) - Nanoseconds since epoch\n";
        full_metadata += "  - data_size: uint32_t (4 bytes) - Size of data following header\n";
        full_metadata += "  - reserved: uint32_t (4 bytes) - Padding\n\n";
        full_metadata += metadata;

        std::ofstream file(metadata_path);
        if (!file.is_open()) {
            return false;
        }

        file << full_metadata;
        file.close();
        return true;
    }

} // namespace fs
