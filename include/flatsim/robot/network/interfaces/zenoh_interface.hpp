#pragma once

#include "flatsim/communication/messages.hpp"
#include "flatsim/robot/network/interface.hpp"
#include <string>
#include <vector>

namespace fs::network {

    /**
     * @brief Zenoh interface for high-performance P2P communication
     *
     * TODO: Implement Zenoh protocol integration
     * - Zenoh pub/sub with key expressions
     * - Automatic peer discovery via Zenoh scouting
     * - Content-based routing with spatial/temporal filters
     * - Sub-millisecond latency, high throughput
     *
     * Use case: High-performance, scalable P2P communication for field robots
     */
    class ZenohInterface : public Interface {
      private:
        std::string robot_uuid_;
        bool initialized_ = false;
        std::vector<std::string> connected_peers_;

      public:
        ZenohInterface() = default;
        ~ZenohInterface() override = default;

        bool init(const std::string &robot_uuid) override;
        void cleanup() override;
        void tick(float dt) override;
        bool is_ready() const override;
        std::string get_type() const override;
        bool connect_to_peer(const std::string &peer_uuid) override;
        void disconnect_from_peer(const std::string &peer_uuid) override;
        std::vector<std::string> get_connected_peers() const override;
        void send_position(const messages::PositionMessage &msg) override;
        void send_control_command(const std::string &target_uuid, const messages::ControlCommand &msg) override;
        std::vector<messages::PositionMessage> receive_positions() override;
        std::vector<messages::ControlCommand> receive_control_commands() override;
        void send_bytes(const std::vector<uint8_t> &data) override;
        void send_bytes_to_peer(const std::string &peer_uuid, const std::vector<uint8_t> &data) override;
        std::vector<std::vector<uint8_t>> receive_bytes() override;
    };

} // namespace fs::network
