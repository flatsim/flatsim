#include "flatsim/dispatcher.hpp"
#include "flatsim/simulator.hpp"
#include <iostream>

namespace fs {

    Dispatcher::Dispatcher() : context(1) {}

    Dispatcher::~Dispatcher() { cleanup(); }

    bool Dispatcher::init(Simulator *sim, bool use_tcp, const std::string &server_host) {
        if (initialized) {
            std::cerr << "[Dispatcher] Already initialized" << std::endl;
            return false;
        }

        simulator = sim;
        this->use_tcp = use_tcp;
        this->server_host = server_host;

        try {
            // Create REP socket for spawn requests (shared endpoint)
            spawn_socket = std::make_unique<zmq::socket_t>(context, zmq::socket_type::rep);
            spawn_socket->bind("ipc:///tmp/flatsim_spawn");
            spawn_socket->bind("tcp://*:5555");
            spawn_socket->set(zmq::sockopt::rcvtimeo, 0); // Non-blocking

            initialized = true;
            std::cout << "[Dispatcher] Initialized successfully (" << (use_tcp ? "TCP" : "IPC") << ")" << std::endl;
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

            // Create dedicated endpoints for this robot if spawn succeeded
            if (reply.success) {
                std::string uuid = reply.robot_uuid;
                std::string cmd_endpoint, state_endpoint;

                try {
                    // Check if client wants TCP (use_tcp is now a string: empty=IPC, non-empty=TCP with IP)
                    bool client_wants_tcp = !spawn_req.use_tcp.empty();

                    if (client_wants_tcp) {
                        int robot_base_port = next_tcp_port;

                        // Determine the advertised host
                        std::string advertised_host;

                        if (server_host == "0.0.0.0") {
                            // Server allows any interface - use the IP the client connected to
                            advertised_host = spawn_req.use_tcp;
                            std::cout << "[Dispatcher] Using client's connection IP: " << advertised_host << std::endl;
                        } else {
                            // Server has a specific IP configured - use that
                            advertised_host = server_host;
                            std::cout << "[Dispatcher] Using configured server IP: " << advertised_host << std::endl;
                        }

                        cmd_endpoint = "tcp://*:" + std::to_string(robot_base_port);
                        reply.command_endpoint = "tcp://" + advertised_host + ":" + std::to_string(robot_base_port);

                        state_endpoint = "tcp://*:" + std::to_string(robot_base_port + 1);
                        reply.state_endpoint = "tcp://" + advertised_host + ":" + std::to_string(robot_base_port + 1);

                        next_tcp_port += 10;
                    } else {
                        cmd_endpoint = "ipc:///tmp/flatsim_cmd_" + uuid;
                        reply.command_endpoint = cmd_endpoint;

                        state_endpoint = "ipc:///tmp/flatsim_state_" + uuid;
                        reply.state_endpoint = state_endpoint;
                    }

                    // Create PULL socket for this robot's commands
                    auto cmd_sock = std::make_unique<zmq::socket_t>(context, zmq::socket_type::pull);
                    cmd_sock->bind(cmd_endpoint);
                    cmd_sock->set(zmq::sockopt::rcvtimeo, 0);
                    robot_command_sockets[uuid] = std::move(cmd_sock);

                    // Create PUB socket for this robot's state
                    auto state_sock = std::make_unique<zmq::socket_t>(context, zmq::socket_type::pub);
                    state_sock->bind(state_endpoint);
                    robot_state_sockets[uuid] = std::move(state_sock);

                    std::cout << "[Dispatcher] Created endpoints for " << uuid << std::endl;
                    std::cout << "[Dispatcher]   Commands: " << reply.command_endpoint << std::endl;
                    std::cout << "[Dispatcher]   State: " << reply.state_endpoint << std::endl;

                } catch (const zmq::error_t &e) {
                    std::cerr << "[Dispatcher] Failed to create robot endpoints: " << e.what() << std::endl;
                    reply.success = false;
                    reply.error_message = "Failed to create endpoints: " + std::string(e.what());
                }
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
            // Receive commands from all robot-specific sockets
            for (auto &[uuid, socket] : robot_command_sockets) {
                while (true) {
                    zmq::message_t message;
                    auto result = socket->recv(message, zmq::recv_flags::dontwait);

                    if (!result) {
                        break;
                    }

                    std::string msg_str(static_cast<char *>(message.data()), message.size());
                    auto cmd = messages::ControlCommand::deserialize(msg_str);
                    commands.push_back(cmd);
                }
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
            // Find robot-specific state socket
            auto it = robot_state_sockets.find(uuid);
            if (it == robot_state_sockets.end()) {
                return;
            }

            // Serialize and send physics state
            std::string state_str = state.serialize();
            zmq::message_t message(state_str.size());
            memcpy(message.data(), state_str.c_str(), state_str.size());
            it->second->send(message, zmq::send_flags::none);

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

            for (auto &[uuid, socket] : robot_command_sockets) {
                socket->close();
            }
            robot_command_sockets.clear();

            for (auto &[uuid, socket] : robot_state_sockets) {
                socket->close();
            }
            robot_state_sockets.clear();

            context.close();
            initialized = false;
            robots.clear();
            std::cout << "[Dispatcher] Cleaned up" << std::endl;

        } catch (const zmq::error_t &e) {
            std::cerr << "[Dispatcher] Error cleaning up: " << e.what() << std::endl;
        }
    }

} // namespace fs
