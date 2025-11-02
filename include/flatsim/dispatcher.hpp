#pragma once

#include "flatsim/robot.hpp"
#include "flatsim/robot/systems/messages.hpp"
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
        std::unique_ptr<zmq::socket_t> command_socket; // PULL socket for control commands
        std::unique_ptr<zmq::socket_t> state_socket;   // PUB socket for physics states

        // Robot registry: UUID -> Robot pointer
        std::map<std::string, Robot *> robots;

        // Simulator reference for creating robots
        class Simulator *simulator;

        bool initialized = false;

      public:
        Dispatcher();
        ~Dispatcher();

        /**
         * @brief Initialize ZMQ sockets and context
         * @param sim Pointer to simulator for robot spawning
         * @return true if initialization successful
         */
        bool init(Simulator *sim);

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
