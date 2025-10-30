#pragma once

#include "flatsim/robot/systems/messages.hpp"
#include <string>
#include <vector>

namespace fs::network {

    /**
     * @brief Abstract base class for communication interfaces
     *
     * Interface represents a specific communication protocol/medium
     * (Zenoh, CAN-bus, WiFi, etc.) that robots can use to communicate.
     * Each interface handles its own connection management.
     */
    class Interface {
      public:
        virtual ~Interface() = default;

        /**
         * @brief Initialize interface with robot identification
         * @param robot_uuid Unique identifier for this robot
         * @return true if initialization successful
         */
        virtual bool init(const std::string &robot_uuid) = 0;

        /**
         * @brief Cleanup interface resources
         */
        virtual void cleanup() = 0;

        /**
         * @brief Process interface-specific updates (non-blocking)
         * @param dt Time delta since last update
         */
        virtual void tick(float dt) = 0;

        /**
         * @brief Check if interface is ready for communication
         * @return true if interface is operational
         */
        virtual bool is_ready() const = 0;

        /**
         * @brief Get interface type/name for identification
         * @return String identifier for this interface type
         */
        virtual std::string get_type() const = 0;

        /**
         * @brief Connect to a specific peer robot
         * @param peer_uuid UUID of robot to connect to
         * @return true if connection successful
         */
        virtual bool connect_to_peer(const std::string &peer_uuid) = 0;

        /**
         * @brief Disconnect from a specific peer robot
         * @param peer_uuid UUID of robot to disconnect from
         */
        virtual void disconnect_from_peer(const std::string &peer_uuid) = 0;

        /**
         * @brief Get list of connected peers
         * @return Vector of connected peer UUIDs
         */
        virtual std::vector<std::string> get_connected_peers() const = 0;

        /**
         * @brief Send position message via this interface
         * @param msg Position message to send
         */
        virtual void send_position(const messages::PositionMessage &msg) = 0;

        /**
         * @brief Send control command to specific peer
         * @param target_uuid UUID of target robot
         * @param msg Control command to send
         */
        virtual void send_control_command(const std::string &target_uuid, const messages::ControlCommand &msg) = 0;

        /**
         * @brief Receive position messages from this interface
         * @return Vector of received position messages
         */
        virtual std::vector<messages::PositionMessage> receive_positions() = 0;

        /**
         * @brief Receive control commands from this interface
         * @return Vector of received control commands
         */
        virtual std::vector<messages::ControlCommand> receive_control_commands() = 0;

        /**
         * @brief Send raw bytes via this interface (broadcast to all peers)
         * @param data Raw byte data to send
         */
        virtual void send_bytes(const std::vector<uint8_t> &data) = 0;

        /**
         * @brief Send raw bytes to specific peer via this interface
         * @param peer_uuid UUID of target peer
         * @param data Raw byte data to send
         */
        virtual void send_bytes_to_peer(const std::string &peer_uuid, const std::vector<uint8_t> &data) = 0;

        /**
         * @brief Receive raw bytes from this interface
         * @return Vector of received byte arrays
         */
        virtual std::vector<std::vector<uint8_t>> receive_bytes() = 0;
    };

} // namespace fs::network