#include "flatsim/agent/network/zenoh_interface.hpp"

#include <algorithm>

namespace fs::network {

    bool ZenohInterface::init(const std::string &robot_uuid) {
        // TODO: Initialize Zenoh session and publishers/subscribers
        robot_uuid_ = robot_uuid;
        initialized_ = true;
        return true;
    }

    void ZenohInterface::cleanup() {
        // TODO: Close Zenoh session and cleanup resources
        initialized_ = false;
        connected_peers_.clear();
    }

    void ZenohInterface::tick(float dt) {
        // TODO: Process Zenoh events (non-blocking)
        (void)dt; // Suppress unused parameter warning
    }

    bool ZenohInterface::is_ready() const { return initialized_; }

    std::string ZenohInterface::get_type() const { return "zenoh"; }

    bool ZenohInterface::connect_to_peer(const std::string &peer_uuid) {
        // TODO: Setup Zenoh subscriber for this peer's key expressions
        if (std::find(connected_peers_.begin(), connected_peers_.end(), peer_uuid) == connected_peers_.end()) {
            connected_peers_.push_back(peer_uuid);
        }
        return true;
    }

    void ZenohInterface::disconnect_from_peer(const std::string &peer_uuid) {
        // TODO: Remove Zenoh subscriber for this peer
        connected_peers_.erase(std::remove(connected_peers_.begin(), connected_peers_.end(), peer_uuid),
                               connected_peers_.end());
    }

    std::vector<std::string> ZenohInterface::get_connected_peers() const { return connected_peers_; }

    void ZenohInterface::send_bytes(const std::vector<uint8_t> &data) {
        // TODO: Send raw bytes via Zenoh (broadcast)
        (void)data;
    }

    void ZenohInterface::send_bytes_to_peer(const std::string &peer_uuid, const std::vector<uint8_t> &data) {
        // TODO: Send raw bytes to specific peer via Zenoh
        (void)peer_uuid;
        (void)data;
    }

    std::vector<std::vector<uint8_t>> ZenohInterface::receive_bytes() {
        // TODO: Receive raw bytes from Zenoh
        return {};
    }

} // namespace fs::network
