#pragma once

#include "flatsim/robot/network/interface.hpp"
#include <string>
#include <vector>

namespace fs::network {

    /**
     * @brief CAN-bus interface for low-latency, reliable communication
     *
     * TODO: Implement CAN-bus protocol integration
     * - CAN-bus hardware interface
     * - Linear bus topology (master-slave chains)
     * - ~100μs deterministic latency
     * - High reliability (error detection, automatic retransmission)
     *
     * Use case: Physically connected devices (SLAVE robots), high reliability
     */
    class CANBusInterface : public Interface {
      private:
        std::string robot_uuid_;
        bool initialized_ = false;
        std::vector<std::string> connected_peers_;

      public:
        CANBusInterface() = default;
        ~CANBusInterface() override = default;

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
