#pragma once

#include "flatsim/agent/network/interface.hpp"
#include <string>
#include <vector>

namespace fs::network {

    /**
     * @brief WiFi interface for long-range communication
     *
     * TODO: Implement WiFi protocol integration
     * - WiFi Direct/Ad-hoc or Infrastructure mode
     * - UDP multicast for discovery
     * - TCP for reliable messaging
     * - Up to 100m range (line of sight)
     * - High bandwidth (video, sensor data)
     *
     * Use case: Long-range communication for dispersed robot fleets
     */
    class WiFiInterface : public Interface {
      private:
        std::string robot_uuid_;
        bool initialized_ = false;
        std::vector<std::string> connected_peers_;

      public:
        WiFiInterface() = default;
        ~WiFiInterface() override = default;

        bool init(const std::string &robot_uuid) override;
        void cleanup() override;
        void tick(float dt) override;
        bool is_ready() const override;
        std::string get_type() const override;
        bool connect_to_peer(const std::string &peer_uuid) override;
        void disconnect_from_peer(const std::string &peer_uuid) override;
        std::vector<std::string> get_connected_peers() const override;
        void send_bytes(const std::vector<uint8_t> &data) override;
        void send_bytes_to_peer(const std::string &peer_uuid, const std::vector<uint8_t> &data) override;
        std::vector<std::vector<uint8_t>> receive_bytes() override;
    };

} // namespace fs::network
