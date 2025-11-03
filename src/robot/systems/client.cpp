#include "flatsim/robot/systems/client.hpp"
#include <chrono>
#include <iostream>

namespace fs {

    Client::Client() : context(1) {}

    Client::~Client() { cleanup(); }

    bool Client::init(bool use_tcp, const std::string &host) {
        if (initialized) {
            std::cerr << "[Client] Already initialized" << std::endl;
            return false;
        }

        this->use_tcp = use_tcp;
        this->tcp_host = host;

        try {
            // Create REQ socket for spawn requests (only connect to spawn endpoint)
            spawn_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::req);
            if (use_tcp) {
                spawn_socket->connect("tcp://" + host + ":5555");
            } else {
                spawn_socket->connect("ipc:///tmp/flatsim_spawn");
            }

            // Create sockets but don't connect yet - will connect after spawn
            command_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::push);
            state_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::sub);
            state_socket->set(zmq::sockopt::subscribe, "");
            state_socket->set(zmq::sockopt::rcvtimeo, 0);

            initialized = true;
            std::cout << "[Client] Initialized successfully (" << (use_tcp ? "TCP" : "IPC") << ")" << std::endl;
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
            messages::SpawnRobotRequest request(info_msg, 0.0, use_tcp); // Pass transport preference

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
                spawned = true;
                robot_uuid = spawn_reply.robot_uuid;

                // Connect to assigned endpoints
                std::cout << "[Client] Connecting to assigned endpoints..." << std::endl;
                std::cout << "[Client]   Commands: " << spawn_reply.command_endpoint << std::endl;
                std::cout << "[Client]   State: " << spawn_reply.state_endpoint << std::endl;

                command_socket->connect(spawn_reply.command_endpoint);
                state_socket->connect(spawn_reply.state_endpoint);

                std::cout << "[Client] Robot spawned successfully: " << robot_uuid << std::endl;
                return true;
            } else {
                std::cerr << "[Client] Failed to spawn robot: " << spawn_reply.error_message << std::endl;
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

            // Since we have dedicated socket, this is always our state
            return state;

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
