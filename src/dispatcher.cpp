#include "flatsim/dispatcher.hpp"
#include "flatsim/simulator.hpp"
#include <iostream>

namespace fs {

    Dispatcher::Dispatcher() : context(1) {}

    Dispatcher::~Dispatcher() { cleanup(); }

    bool Dispatcher::init(Simulator *sim) {
        if (initialized) {
            std::cerr << "[Dispatcher] Already initialized" << std::endl;
            return false;
        }

        simulator = sim;

        try {
            // Create REP socket for spawn requests
            spawn_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::rep);
            spawn_socket->bind("ipc:///tmp/flatsim_spawn");
            spawn_socket->set(zmq::sockopt::rcvtimeo, 0); // Non-blocking

            // Create PULL socket for control commands
            command_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::pull);
            command_socket->bind("ipc:///tmp/flatsim_commands");
            command_socket->set(zmq::sockopt::rcvtimeo, 0); // Non-blocking

            // Create PUB socket for physics states
            state_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::pub);
            state_socket->bind("ipc:///tmp/flatsim_state");

            initialized = true;
            std::cout << "[Dispatcher] Initialized successfully" << std::endl;
            return true;

        } catch (const zmq::error_t &e) {
            std::cerr << "[Dispatcher] Failed to initialize: " << e.what() << std::endl;
            return false;
        }
    }

    void Dispatcher::process_spawn_requests() {
        if (!initialized) {
            return;
        }

        try {
            zmq::message_t request;
            auto result = spawn_socket->recv(request, zmq::recv_flags::dontwait);

            if (!result) {
                // No message available
                return;
            }

            // Deserialize spawn request
            std::string request_str(static_cast<char *>(request.data()), request.size());
            auto spawn_req = messages::SpawnRobotRequest::deserialize(request_str);

            std::cout << "[Dispatcher] Received spawn request for robot: " << spawn_req.robot_info.uuid << std::endl;

            // Create response
            messages::SpawnRobotReply reply;
            reply.robot_uuid = spawn_req.robot_info.uuid;

            try {
                // Convert RobotInfoMessage to RobotInfo
                RobotInfo robot_info = spawn_req.robot_info.to_robot_info();

                // Generate unique seqid: type + "_" + index
                std::string seqid = robot_info.type + "_" + std::to_string(robots.size());
                robot_info.seqid = seqid;

                std::cout << "[Dispatcher] Spawning robot with seqid: " << seqid << std::endl;

                // Spawn the robot in simulator
                if (simulator) {
                    simulator->add_robot(robot_info);

                    // Register robot in our registry
                    // Find the robot that was just added
                    Robot *spawned_robot = nullptr;
                    for (auto &robot : simulator->robots) {
                        if (robot && robot->info.uuid == robot_info.uuid) {
                            spawned_robot = robot.get();
                            break;
                        }
                    }

                    if (spawned_robot) {
                        robots[robot_info.uuid] = spawned_robot;
                        reply.success = true;
                        reply.error_message = "";
                        std::cout << "[Dispatcher] Robot spawned successfully: " << robot_info.uuid << std::endl;
                    } else {
                        reply.success = false;
                        reply.error_message = "Robot spawned but not found in simulator";
                        std::cerr << "[Dispatcher] " << reply.error_message << std::endl;
                    }
                } else {
                    reply.success = false;
                    reply.error_message = "Simulator not initialized";
                    std::cerr << "[Dispatcher] " << reply.error_message << std::endl;
                }

            } catch (const std::exception &e) {
                reply.success = false;
                reply.error_message = std::string("Failed to spawn robot: ") + e.what();
                std::cerr << "[Dispatcher] " << reply.error_message << std::endl;
            }

            // Send reply
            std::string reply_str = reply.serialize();
            zmq::message_t response(reply_str.size());
            memcpy(response.data(), reply_str.c_str(), reply_str.size());
            spawn_socket->send(response, zmq::send_flags::none);

            std::cout << "[Dispatcher] Sent spawn reply for robot: " << reply.robot_uuid << std::endl;

        } catch (const zmq::error_t &e) {
            std::cerr << "[Dispatcher] Error processing spawn request: " << e.what() << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[Dispatcher] Error deserializing spawn request: " << e.what() << std::endl;
        }
    }

    std::vector<messages::ControlCommand> Dispatcher::receive_commands() {
        std::vector<messages::ControlCommand> commands;

        if (!initialized) {
            return commands;
        }

        try {
            // Receive all available control commands
            while (true) {
                zmq::message_t message;
                auto result = command_socket->recv(message, zmq::recv_flags::dontwait);

                if (!result) {
                    // No more messages
                    break;
                }

                // Deserialize control command
                std::string msg_str(static_cast<char *>(message.data()), message.size());
                auto cmd = messages::ControlCommand::deserialize(msg_str);
                commands.push_back(cmd);
            }

        } catch (const zmq::error_t &e) {
            std::cerr << "[Dispatcher] Error receiving control commands: " << e.what() << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[Dispatcher] Error deserializing control command: " << e.what() << std::endl;
        }

        return commands;
    }

    void Dispatcher::send_state_to_robot(const std::string &uuid, const messages::PhysicsState &state) {
        if (!initialized) {
            return;
        }

        try {
            // Serialize and send physics state
            std::string state_str = state.serialize();
            zmq::message_t message(state_str.size());
            memcpy(message.data(), state_str.c_str(), state_str.size());
            state_socket->send(message, zmq::send_flags::none);

        } catch (const zmq::error_t &e) {
            std::cerr << "[Dispatcher] Error sending physics state for robot " << uuid << ": " << e.what() << std::endl;
        }
    }

    void Dispatcher::send_states() {
        if (!initialized) {
            return;
        }

        // Send physics state for each registered robot
        for (const auto &[uuid, robot] : robots) {
            if (robot) {
                messages::PhysicsState state;
                state.robot_uuid = uuid;
                state.timestamp = 0.0; // TODO: Get actual timestamp
                state.pose = robot->get_position();
                state.velocity.linear = 0.0f;  // TODO: Get actual velocity
                state.velocity.angular = 0.0f; // TODO: Get actual velocity

                send_state_to_robot(uuid, state);
            }
        }
    }

    void Dispatcher::cleanup() {
        if (!initialized) {
            return;
        }

        try {
            spawn_socket->close();
            command_socket->close();
            state_socket->close();
            context.close();
            initialized = false;
            robots.clear();
            std::cout << "[Dispatcher] Cleaned up" << std::endl;

        } catch (const zmq::error_t &e) {
            std::cerr << "[Dispatcher] Error cleaning up: " << e.what() << std::endl;
        }
    }

} // namespace fs
