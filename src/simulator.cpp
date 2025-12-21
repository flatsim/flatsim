#include "flatsim/simulator.hpp"
#include <chrono>
#include <cista/serialization.h>
#include <iostream>
#include <vector>

namespace simulator {

    Simulator::Simulator(Conn conn, const std::string &address, const SimulatorSettings &settings,
                         std::shared_ptr<rerun::RecordingStream> rec)
        : ctx_(1), conn_(conn), address_(address), sim_settings_(settings), rec_(rec) {

        // Setup ZMQ spawn socket (REP)
        spawn_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::rep);

        // Setup heartbeat socket (PULL)
        heartbeat_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pull);

        if (conn_ == Conn::IPC) {
            spawn_socket_->bind("ipc:///tmp/flatsim_spawn");
            heartbeat_socket_->bind("ipc:///tmp/flatsim_heartbeat");
            std::cout << "[Simulator] Spawn socket listening on ipc:///tmp/flatsim_spawn" << std::endl;
            std::cout << "[Simulator] Heartbeat socket listening on ipc:///tmp/flatsim_heartbeat" << std::endl;
        } else {
            spawn_socket_->bind("tcp://*:5555");
            heartbeat_socket_->bind("tcp://*:5556");
            std::cout << "[Simulator] Spawn socket listening on tcp://*:5555" << std::endl;
            std::cout << "[Simulator] Heartbeat socket listening on tcp://*:5556" << std::endl;
        }
        spawn_socket_->set(zmq::sockopt::rcvtimeo, 0);
        heartbeat_socket_->set(zmq::sockopt::rcvtimeo, 0);

        // Setup physics world with World wrapper
        world_ = std::make_unique<World>(rec_);
        world_->init(concord::Datum(), concord::Size(sim_settings_.width, sim_settings_.height, 0.0));
        std::cout << "[Simulator] Initialized with world size: " << sim_settings_.width << "x" << sim_settings_.height
                  << std::endl;
    }

    Simulator::~Simulator() {
        spawn_socket_->close();
        heartbeat_socket_->close();
        for (auto &[uuid, sock] : control_sockets_) {
            sock->close();
        }
        for (auto &[uuid, sock] : state_sockets_) {
            sock->close();
        }
        ctx_.close();
    }

    void Simulator::create_machine(const types::Machine &machine) {
        uint32_t group = machine.group > 0 ? machine.group : next_group_++;

        // Create machine directly in the map to avoid pointer invalidation
        auto [it, inserted] = machines_.try_emplace(machine.uuid, rec_, world_->physics_ptr(), machine, group);
        if (inserted) {
            it->second.create();
        }
    }

    void Simulator::apply_control(const types::WheelControl &control, float dt) {
        auto it = machines_.find(control.uuid);
        if (it == machines_.end()) {
            return;
        }
        it->second.apply_control(control, dt);
    }

    bool Simulator::destroy_machine(const std::string &uuid) {
        auto it = machines_.find(uuid);
        if (it == machines_.end()) {
            return false;
        }

        it->second.destroy();
        machines_.erase(it);
        return true;
    }

    types::ser::WorldState Simulator::get_world_state() const {
        types::ser::WorldState state;

        for (const auto &[uuid, machine] : machines_) {
            state.machines.push_back(machine.get_state());
        }

        return state;
    }

    void Simulator::tick(float dt) {
        static int tick_num = 0;
        tick_num++;

        // Tick physics world FIRST (like old code)
        world_->tick(dt);

        // Then tick all machines
        for (auto &[uuid, machine] : machines_) {
            machine.tick(dt);
        }

        if (tick_num % 60 == 0) {
            std::cout << "[Simulator::tick] Tick #" << tick_num << " - " << machines_.size() << " machines"
                      << std::endl;
        }

        // Process spawn/despawn requests (REP socket)
        zmq::message_t spawn_request;
        auto spawn_result = spawn_socket_->recv(spawn_request, zmq::recv_flags::dontwait);
        if (spawn_result) {
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(spawn_request.data()),
                                        static_cast<uint8_t *>(spawn_request.data()) + spawn_request.size());

            auto *req = cista::deserialize<types::ser::Request>(buffer);
            types::ser::Response resp;

            if (!req) {
                resp.success = false;
            } else if (req->type == types::ser::MsgType::SPAWN) {
                auto machine = req->machine.to_machine();
                std::string uuid = machine.uuid;

                // Create machine
                create_machine(machine);

                // Create dedicated sockets for this machine
                auto ctrl_sock = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pull);
                auto state_sock = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pub);

                if (conn_ == Conn::IPC) {
                    std::string ctrl_addr = "ipc:///tmp/flatsim_ctrl_" + uuid;
                    std::string state_addr = "ipc:///tmp/flatsim_state_" + uuid;
                    ctrl_sock->bind(ctrl_addr);
                    state_sock->bind(state_addr);
                    std::cout << "[Simulator] Created sockets for " << uuid << std::endl;
                    std::cout << "[Simulator]   Control: " << ctrl_addr << std::endl;
                    std::cout << "[Simulator]   State: " << state_addr << std::endl;
                } else {
                    int base_port = next_tcp_port_;
                    next_tcp_port_ += 10;
                    ctrl_sock->bind("tcp://*:" + std::to_string(base_port));
                    state_sock->bind("tcp://*:" + std::to_string(base_port + 1));
                    std::cout << "[Simulator] Created TCP sockets for " << uuid << " on ports " << base_port << " and "
                              << (base_port + 1) << std::endl;
                }

                ctrl_sock->set(zmq::sockopt::rcvtimeo, 0);
                control_sockets_[uuid] = std::move(ctrl_sock);
                state_sockets_[uuid] = std::move(state_sock);

                // Initialize heartbeat timestamp
                last_heartbeat_[uuid] = std::chrono::steady_clock::now();

                std::cout << "[Simulator] Getting world state..." << std::endl;
                resp.success = true;
                resp.state = get_world_state();
                std::cout << "[Simulator] Got world state, sending response..." << std::endl;
                std::cout << "[Simulator] Spawned machine: " << uuid << std::endl;
            } else if (req->type == types::ser::MsgType::DESPAWN) {
                std::string uuid_str(req->uuid.view());
                std::cout << "[Simulator] Despawning machine: " << uuid_str << std::endl;

                try {
                    // Close and remove sockets first
                    if (control_sockets_.count(uuid_str)) {
                        std::cout << "[Simulator] Closing control socket..." << std::endl;
                        control_sockets_[uuid_str]->close();
                        control_sockets_.erase(uuid_str);
                    }
                    if (state_sockets_.count(uuid_str)) {
                        std::cout << "[Simulator] Closing state socket..." << std::endl;
                        state_sockets_[uuid_str]->close();
                        state_sockets_.erase(uuid_str);
                    }

                    // Then destroy machine
                    std::cout << "[Simulator] Destroying machine..." << std::endl;
                    resp.success = destroy_machine(uuid_str);
                    last_heartbeat_.erase(uuid_str);
                    std::cout << "[Simulator] Despawned machine: " << uuid_str << std::endl;
                } catch (const std::exception &e) {
                    std::cerr << "[Simulator] Exception during despawn: " << e.what() << std::endl;
                    resp.success = false;
                }
            } else {
                resp.success = false;
            }

            auto data = cista::serialize(resp);
            spawn_socket_->send(zmq::buffer(data), zmq::send_flags::none);
        }

        // Process control commands from all machines (PULL sockets)
        static int ctrl_tick = 0;
        for (auto &[uuid, socket] : control_sockets_) {
            if (!socket) continue; // Skip null sockets

            try {
                zmq::message_t ctrl_msg;
                auto ctrl_result = socket->recv(ctrl_msg, zmq::recv_flags::dontwait);
                if (ctrl_result) {
                    std::vector<uint8_t> buffer(static_cast<uint8_t *>(ctrl_msg.data()),
                                                static_cast<uint8_t *>(ctrl_msg.data()) + ctrl_msg.size());
                    auto *ctrl_req = cista::deserialize<types::ser::WheelControl>(buffer);
                    if (ctrl_req) {
                        auto control = ctrl_req->to_control();

                        // Debug: Print control every 60 ticks (~1 second)
                        if (++ctrl_tick % 60 == 0) {
                            std::cout << "[Simulator] Received control for " << uuid << ": throttle["
                                      << control.throttle.size() << "] = ";
                            for (size_t i = 0; i < std::min(control.throttle.size(), size_t(4)); ++i) {
                                std::cout << control.throttle[i] << " ";
                            }
                            std::cout << " steering[" << control.steering.size() << "]" << std::endl;
                        }

                        apply_control(control, dt);
                    }
                }
            } catch (const zmq::error_t &e) {
                // Socket might be closed, ignore
            }
        }

        // Process heartbeat messages (PULL socket, non-blocking)
        // Limit to 100 messages per tick to prevent blocking
        for (int i = 0; i < 100; ++i) {
            zmq::message_t hb_msg;
            auto hb_result = heartbeat_socket_->recv(hb_msg, zmq::recv_flags::dontwait);
            if (!hb_result) break;

            std::vector<uint8_t> buffer(static_cast<uint8_t *>(hb_msg.data()),
                                        static_cast<uint8_t *>(hb_msg.data()) + hb_msg.size());
            auto *hb_req = cista::deserialize<types::ser::Request>(buffer);
            if (hb_req && hb_req->type == types::ser::MsgType::HEARTBEAT) {
                std::string uuid_str(hb_req->uuid.view());
                last_heartbeat_[uuid_str] = std::chrono::steady_clock::now();
            }
        }

        // Check for stale heartbeats (>5 seconds) and remove dead machines
        // Only check every 60 ticks (~1 second) to avoid overhead
        static int cleanup_tick = 0;
        if (++cleanup_tick % 60 == 0) {
            auto now = std::chrono::steady_clock::now();
            std::vector<std::string> to_remove;

            for (const auto &[uuid, last_hb] : last_heartbeat_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_hb).count();
                if (elapsed > 5) {
                    std::cout << "[Simulator] Machine " << uuid << " heartbeat timeout (" << elapsed
                              << "s), removing..." << std::endl;
                    to_remove.push_back(uuid);
                }
            }

            for (const auto &uuid : to_remove) {
                std::cout << "[Simulator] Heartbeat timeout for machine: " << uuid << std::endl;
                std::cout << "[Simulator]   Cleaning up sockets and tracking..." << std::endl;

                // Close and remove sockets
                if (control_sockets_.count(uuid)) {
                    control_sockets_[uuid]->close();
                    control_sockets_.erase(uuid);
                }
                if (state_sockets_.count(uuid)) {
                    state_sockets_[uuid]->close();
                    state_sockets_.erase(uuid);
                }

                // Remove from heartbeat tracking
                last_heartbeat_.erase(uuid);

                // NOTE: We intentionally DON'T call destroy_machine() here because:
                // 1. It causes segfaults (physics body/chassis may be in use)
                // 2. The machine will stop receiving controls anyway (socket closed)
                // 3. Proper cleanup should happen via explicit DESPAWN message
                // TODO: Mark machine as "disconnected" or "inactive" instead of destroying

                std::cout << "[Simulator] Cleaned up resources for: " << uuid << std::endl;
            }
        }

        // Publish state to all machines (PUB sockets)
        static int state_tick = 0;
        if (!state_sockets_.empty()) {
            auto world_state = get_world_state();
            for (auto &[uuid, socket] : state_sockets_) {
                if (!socket) continue; // Skip null sockets

                // Find this machine's state
                for (const auto &ms : world_state.machines) {
                    if (std::string(ms.uuid.view()) == uuid) {
                        try {
                            // Debug: Print state every 60 ticks (~1 second)
                            if (++state_tick % 60 == 0) {
                                std::cout << "[Simulator] Publishing state for " << uuid << ": pose=("
                                          << ms.pose.position.x << ", " << ms.pose.position.y << ", " << ms.pose.angle
                                          << ")" << std::endl;
                            }

                            auto data = cista::serialize(ms);
                            socket->send(zmq::buffer(data), zmq::send_flags::dontwait);
                        } catch (const zmq::error_t &e) {
                            // Socket might be closed, ignore
                        }
                        break;
                    }
                }
            }
        }
    }

    void Simulator::tock() {
        // Visualization for all machines
        for (auto &[uuid, machine] : machines_) {
            machine.tock();
        }

        // Visualization for world (boundaries, obstacles, etc.)
        world_->tock();
    }

} // namespace simulator
