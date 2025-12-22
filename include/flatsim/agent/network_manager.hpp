#pragma once

#include "flatsim/agent/network/interface.hpp"
#include <memory>
#include <string>
#include <vector>

namespace fs {

    /**
     * @brief Network manager for robot communication interfaces
     *
     * NetworkManager manages multiple communication interfaces for a robot.
     * Each robot can have 1, 2, or many interfaces simultaneously.
     * This class coordinates between interfaces and provides unified API.
     */
    class Network {
      private:
        std::vector<std::unique_ptr<network::Interface>> interfaces;
        std::string robot_uuid;
        bool initialized;

      public:
        Network();
        ~Network();

        // Interface Management
        void add_interface(std::unique_ptr<network::Interface> interface);
        void remove_interface(const std::string &interface_type);
        network::Interface *get_interface(const std::string &interface_type);

        // Lifecycle
        bool init(const std::string &robot_uuid);
        void tick(float dt);
        void cleanup();

        // Status
        bool is_ready() const;
        std::vector<std::string> get_interface_types() const;
        size_t get_interface_count() const;

        // Connection Management (aggregates from all interfaces)
        bool connect_to_peer(const std::string &peer_uuid);
        void disconnect_from_peer(const std::string &peer_uuid);
        std::vector<std::string> get_connected_peers() const;

        // Communication - Generic byte-level methods
        void send_all(const std::vector<uint8_t> &data);
        void send_via(const std::string &interface_type, const std::vector<uint8_t> &data);
        void send_to_peer(const std::string &peer_uuid, const std::vector<uint8_t> &data);
        std::vector<std::vector<uint8_t>> receive();
    };

} // namespace fs
