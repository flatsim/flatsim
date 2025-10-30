#include "flatsim/robot/systems/client.hpp"
#include <chrono>
#include <iostream>

namespace fs {

    Client::Client() : context(1) {}

    Client::~Client() { cleanup(); }

    bool Client::init() {
        if (initialized) {
            std::cerr << "[Client] Already initialized" << std::endl;
            return false;
        }

        try {
            // Create REQ socket for spawn requests
            spawn_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::req);
            spawn_socket->connect("ipc:///tmp/flatsim_spawn");

            // Create PUSH socket for control commands
            command_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::push);
            command_socket->connect("ipc:///tmp/flatsim_commands");

            // Create SUB socket for physics states
            state_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::sub);
            state_socket->connect("ipc:///tmp/flatsim_state");
            state_socket->set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
            state_socket->set(zmq::sockopt::rcvtimeo, 0);   // Non-blocking

            initialized = true;
            std::cout << "[Client] Initialized successfully" << std::endl;
            return true;

        } catch (const zmq::error_t &e) {
            std::cerr << "[Client] Failed to initialize: " << e.what() << std::endl;
            return false;
        }
    }

    bool Client::spawn_robot(const RobotInfo &robot_info) {
        if (!initialized) {
            std::cerr << "[Client] Not initialized" << std::endl;
            return false;
        }

        if (spawned) {
            std::cerr << "[Client] Robot already spawned" << std::endl;
            return false;
        }

        try {
            // Create spawn request
            messages::RobotInfoMessage info_msg(robot_info);
            messages::SpawnRobotRequest request(info_msg, 0.0); // TODO: Add timestamp

            // Serialize and send request
            std::string request_str = request.serialize();
            zmq::message_t req_msg(request_str.size());
            memcpy(req_msg.data(), request_str.c_str(), request_str.size());
            spawn_socket->send(req_msg, zmq::send_flags::none);

            std::cout << "[Client] Sent spawn request for robot: " << robot_info.uuid << std::endl;

            // Wait for reply (blocking with timeout)
            zmq::message_t reply;
            spawn_socket->set(zmq::sockopt::rcvtimeo, 5000); // 5 second timeout
            auto result = spawn_socket->recv(reply, zmq::recv_flags::none);

            if (!result) {
                std::cerr << "[Client] No reply received from simulator" << std::endl;
                return false;
            }

            // Deserialize reply
            std::string reply_str(static_cast<char *>(reply.data()), reply.size());
            auto spawn_reply = messages::SpawnRobotReply::deserialize(reply_str);

            if (spawn_reply.success) {
                robot_uuid = spawn_reply.robot_uuid;
                spawned = true;
                std::cout << "[Client] Robot spawned successfully: " << robot_uuid << std::endl;
                return true;
            } else {
                std::cerr << "[Client] Spawn failed: " << spawn_reply.error_message << std::endl;
                return false;
            }

        } catch (const zmq::error_t &e) {
            std::cerr << "[Client] ZMQ error during spawn: " << e.what() << std::endl;
            return false;
        } catch (const std::exception &e) {
            std::cerr << "[Client] Error during spawn: " << e.what() << std::endl;
            return false;
        }
    }

    void Client::send_control_command(const messages::ControlCommand &cmd) {
        if (!initialized || !spawned) {
            return;
        }

        try {
            // Serialize and send control command
            std::string cmd_str = cmd.serialize();
            zmq::message_t message(cmd_str.size());
            memcpy(message.data(), cmd_str.c_str(), cmd_str.size());
            command_socket->send(message, zmq::send_flags::none);

        } catch (const zmq::error_t &e) {
            std::cerr << "[Client] Error sending control command: " << e.what() << std::endl;
        }
    }

    std::optional<messages::PhysicsState> Client::receive_physics_state() {
        if (!initialized || !spawned) {
            return std::nullopt;
        }

        try {
            zmq::message_t message;
            auto result = state_socket->recv(message, zmq::recv_flags::dontwait);

            if (!result) {
                // No message available
                return std::nullopt;
            }

            // Deserialize physics state
            std::string state_str(static_cast<char *>(message.data()), message.size());
            auto state = messages::PhysicsState::deserialize(state_str);

            // Only return state for our robot
            if (state.robot_uuid == robot_uuid) {
                return state;
            }

            return std::nullopt;

        } catch (const zmq::error_t &e) {
            std::cerr << "[Client] Error receiving physics state: " << e.what() << std::endl;
            return std::nullopt;
        } catch (const std::exception &e) {
            std::cerr << "[Client] Error deserializing physics state: " << e.what() << std::endl;
            return std::nullopt;
        }
    }

    void Client::cleanup() {
        if (!initialized) {
            return;
        }

        try {
            spawn_socket->close();
            command_socket->close();
            state_socket->close();
            context.close();
            initialized = false;
            spawned = false;
            std::cout << "[Client] Cleaned up" << std::endl;

        } catch (const zmq::error_t &e) {
            std::cerr << "[Client] Error cleaning up: " << e.what() << std::endl;
        }
    }

} // namespace fs
