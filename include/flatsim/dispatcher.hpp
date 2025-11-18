#pragma once

#include "flatsim/communication/messages.hpp"
#include "flatsim/robot.hpp"
#include "flatsim/types.hpp"
#include <map>
#include <memory>
#include <string>
#include <zmq.hpp>

namespace fs {

    /**
     * @brief Dispatcher handles ZMQ communication between simulator and robot processes
     *
     * The Dispatcher runs in the simulator process and manages:
     * - Robot spawn requests from robot processes
     * - Control commands from robot processes
     * - Physics state broadcasts to robot processes
     */
    class Dispatcher {
      private:
        zmq::context_t context;
        std::unique_ptr<zmq::socket_t> spawn_socket;   // REP socket for spawn requests
        std::unique_ptr<zmq::socket_t> command_socket; // PULL socket for control commands (DEPRECATED)
        std::unique_ptr<zmq::socket_t> state_socket;   // PUB socket for physics states (DEPRECATED)

        // Per-robot sockets
        std::map<std::string, std::unique_ptr<zmq::socket_t>> robot_command_sockets; // PULL per robot
        std::map<std::string, std::unique_ptr<zmq::socket_t>> robot_state_sockets;   // PUB per robot

        // Robot registry: UUID -> Robot pointer
        std::map<std::string, Robot *> robots;

        // Robot heartbeat tracking: UUID -> last heartbeat timestamp
        std::map<std::string, double> robot_last_heartbeat;
        double heartbeat_timeout = 5.0; // seconds

        // Simulator reference for creating robots
        class Simulator *simulator;

        bool initialized = false;
        bool use_tcp = false;
        int next_tcp_port = 6000;
        std::string server_host = "0.0.0.0"; // Host address for TCP connections (0.0.0.0 = use client's IP)

      public:
        Dispatcher();
        ~Dispatcher();

        /**
         * @brief Initialize ZMQ sockets and context
         * @param sim Pointer to simulator for robot spawning
         * @param use_tcp If true, use TCP transport instead of IPC
         * @param server_host Host address to advertise (default: 0.0.0.0 = use client's IP)
         * @return true if initialization successful
         */
        bool init(Simulator *sim, bool use_tcp = false, const std::string &server_host = "0.0.0.0");

        /**
         * @brief Process incoming spawn requests from robot processes
         */
        void process_spawn_requests();

        /**
         * @brief Receive control commands from robot processes
         * @return Vector of received control commands
         */
        std::vector<messages::ControlCommand> receive_commands();

        /**
         * @brief Send physics state to all robot processes
         * @param uuid Robot UUID
         * @param state Physics state to send
         */
        void send_state_to_robot(const std::string &uuid, const messages::PhysicsState &state);

        /**
         * @brief Send physics states to all robots
         */
        void send_states();

        /**
         * @brief Process heartbeat messages from robot processes
         * @param current_time Current simulation time for timeout checking
         */
        void process_heartbeats(double current_time);

        /**
         * @brief Check if a robot is online (based on heartbeat)
         * @param uuid Robot UUID
         * @return true if robot is online
         */
        bool is_robot_online(const std::string &uuid) const;

        /**
         * @brief Get last heartbeat time for a robot
         * @param uuid Robot UUID
         * @return Last heartbeat timestamp, or 0.0 if not found
         */
        double get_last_heartbeat(const std::string &uuid) const;

        /**
         * @brief Set heartbeat timeout
         * @param timeout Timeout in seconds
         */
        void set_heartbeat_timeout(double timeout) { heartbeat_timeout = timeout; }

        /**
         * @brief Get heartbeat timeout
         * @return Timeout in seconds
         */
        double get_heartbeat_timeout() const { return heartbeat_timeout; }

        /**
         * @brief Cleanup ZMQ resources
         */
        void cleanup();

        /**
         * @brief Check if dispatcher is ready
         */
        bool is_ready() const { return initialized; }

        /**
         * @brief Get number of registered robots
         */
        size_t get_robot_count() const { return robots.size(); }
    };

} // namespace fs
