#pragma once

#include "flatsim/robot/systems/messages.hpp"
#include "flatsim/types.hpp"
#include <memory>
#include <string>
#include <zmq.hpp>

namespace fs {

    /**
     * @brief Client handles ZMQ communication from robot process to simulator
     *
     * The Client runs in robot processes and manages:
     * - Spawning robot in simulator
     * - Sending control commands to simulator
     * - Receiving physics states from simulator
     */
    class Client {
      private:
        zmq::context_t context;
        std::unique_ptr<zmq::socket_t> spawn_socket;   // REQ socket for spawn requests
        std::unique_ptr<zmq::socket_t> command_socket; // PUSH socket for control commands
        std::unique_ptr<zmq::socket_t> state_socket;   // SUB socket for physics states

        std::string robot_uuid;
        bool initialized = false;
        bool spawned = false;

      public:
        Client();
        ~Client();

        /**
         * @brief Initialize ZMQ sockets and context
         * @return true if initialization successful
         */
        bool init();

        /**
         * @brief Spawn robot in simulator
         * @param robot_info Robot configuration to spawn
         * @return true if spawn successful
         */
        bool spawn_robot(const RobotInfo &robot_info);

        /**
         * @brief Send control command to simulator
         * @param cmd Control command to send
         */
        void send_control_command(const messages::ControlCommand &cmd);

        /**
         * @brief Receive physics state from simulator (non-blocking)
         * @return Physics state if available, empty optional otherwise
         */
        std::optional<messages::PhysicsState> receive_physics_state();

        /**
         * @brief Cleanup ZMQ resources
         */
        void cleanup();

        /**
         * @brief Check if client is ready
         */
        bool is_ready() const { return initialized; }

        /**
         * @brief Check if robot has been spawned
         */
        bool is_spawned() const { return spawned; }

        /**
         * @brief Get robot UUID
         */
        const std::string &get_uuid() const { return robot_uuid; }
    };

} // namespace fs
