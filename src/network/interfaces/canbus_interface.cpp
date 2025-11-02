#include "flatsim/network/interfaces/canbus_interface.hpp"

namespace fs::network {

    bool CANBusInterface::init(const std::string &robot_uuid) {
        // TODO: Initialize CAN-bus hardware interface
        // - Open CAN interface (e.g., "can0")
        // - Set bitrate (e.g., 500kbps)
        robot_uuid_ = robot_uuid;
        initialized_ = true;
        return true;
    }

    void CANBusInterface::cleanup() {
        // TODO: Close CAN-bus hardware interface
        initialized_ = false;
        connected_peers_.clear();
    }

    void CANBusInterface::tick(float dt) {
        // TODO: Process CAN-bus frames (non-blocking)
        (void)dt;
    }

    bool CANBusInterface::is_ready() const { return initialized_; }

    std::string CANBusInterface::get_type() const { return "canbus"; }

    bool CANBusInterface::connect_to_peer(const std::string &peer_uuid) {
        // TODO: Register peer CAN ID mapping
        if (std::find(connected_peers_.begin(), connected_peers_.end(), peer_uuid) == connected_peers_.end()) {
            connected_peers_.push_back(peer_uuid);
        }
        return true;
    }

    void CANBusInterface::disconnect_from_peer(const std::string &peer_uuid) {
        // TODO: Remove peer CAN ID mapping
        connected_peers_.erase(std::remove(connected_peers_.begin(), connected_peers_.end(), peer_uuid),
                               connected_peers_.end());
    }

    std::vector<std::string> CANBusInterface::get_connected_peers() const { return connected_peers_; }

    void CANBusInterface::send_position(const messages::PositionMessage &msg) {
        // TODO: Serialize and send position via CAN-bus frame
        (void)msg;
    }

    void CANBusInterface::send_control_command(const std::string &target_uuid, const messages::ControlCommand &msg) {
        // TODO: Serialize and send control command via CAN-bus frame
        (void)target_uuid;
        (void)msg;
    }

    std::vector<messages::PositionMessage> CANBusInterface::receive_positions() {
        // TODO: Receive and deserialize CAN-bus frames into position messages
        return {};
    }

    std::vector<messages::ControlCommand> CANBusInterface::receive_control_commands() {
        // TODO: Receive and deserialize CAN-bus frames into control commands
        return {};
    }

    void CANBusInterface::send_bytes(const std::vector<uint8_t> &data) {
        // TODO: Send raw bytes via CAN-bus (broadcast)
        (void)data;
    }

    void CANBusInterface::send_bytes_to_peer(const std::string &peer_uuid, const std::vector<uint8_t> &data) {
        // TODO: Send raw bytes to specific peer via CAN-bus
        (void)peer_uuid;
        (void)data;
    }

    std::vector<std::vector<uint8_t>> CANBusInterface::receive_bytes() {
        // TODO: Receive raw bytes from CAN-bus
        return {};
    }

} // namespace fs::network
