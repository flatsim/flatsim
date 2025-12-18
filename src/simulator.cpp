#include "flatsim/simulator.hpp"
#include <cista/serialization.h>
#include <iostream>
#include <vector>

namespace simulator {

    Simulator::Simulator(Conn conn, const std::string &address, const WorldSettings &settings,
                         std::shared_ptr<rerun::RecordingStream> rec)
        : ctx_(1), conn_(conn), address_(address), world_settings_(settings), rec_(rec) {

        // Setup ZMQ spawn socket (REP)
        spawn_socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::rep);

        if (conn_ == Conn::IPC) {
            spawn_socket_->bind("ipc:///tmp/flatsim_spawn");
            std::cout << "[Simulator] Spawn socket listening on ipc:///tmp/flatsim_spawn" << std::endl;
        } else {
            spawn_socket_->bind("tcp://*:5555");
            std::cout << "[Simulator] Spawn socket listening on tcp://*:5555" << std::endl;
        }
        spawn_socket_->set(zmq::sockopt::rcvtimeo, 0);

        // Setup physics world with World wrapper
        types::WorldSettings ws;
        ws.size = concord::Size(world_settings_.width, world_settings_.height, 0.0);
        world_ = std::make_unique<World>(ws, rec_);
    }

    Simulator::~Simulator() {
        spawn_socket_->close();
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

        Machine m(rec_, world_->physics_ptr(), machine, group);
        m.create();
        machines_[machine.uuid] = std::move(m);
    }

    void Simulator::apply_control(const types::MachineControl &control, float dt) {
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
        // Tick all machines
        for (auto &[uuid, machine] : machines_) {
            machine.tick(dt);
        }

        // Tick physics world
        world_->tick(dt);

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

                resp.success = true;
                resp.state = get_world_state();
                std::cout << "[Simulator] Spawned machine: " << uuid << std::endl;
            } else if (req->type == types::ser::MsgType::DESPAWN) {
                std::string uuid_str(req->uuid.view());
                resp.success = destroy_machine(uuid_str);
                if (resp.success) {
                    control_sockets_.erase(uuid_str);
                    state_sockets_.erase(uuid_str);
                }
            } else {
                resp.success = false;
            }

            auto data = cista::serialize(resp);
            spawn_socket_->send(zmq::buffer(data), zmq::send_flags::none);
        }

        // Process control commands from all machines (PULL sockets)
        for (auto &[uuid, socket] : control_sockets_) {
            zmq::message_t ctrl_msg;
            auto ctrl_result = socket->recv(ctrl_msg, zmq::recv_flags::dontwait);
            if (ctrl_result) {
                std::vector<uint8_t> buffer(static_cast<uint8_t *>(ctrl_msg.data()),
                                            static_cast<uint8_t *>(ctrl_msg.data()) + ctrl_msg.size());
                auto *ctrl_req = cista::deserialize<types::ser::MachineControl>(buffer);
                if (ctrl_req) {
                    auto control = ctrl_req->to_control();
                    apply_control(control, dt);
                }
            }
        }

        // Publish state to all machines (PUB sockets)
        auto world_state = get_world_state();
        for (auto &[uuid, socket] : state_sockets_) {
            // Find this machine's state
            for (const auto &ms : world_state.machines) {
                if (std::string(ms.uuid.view()) == uuid) {
                    auto data = cista::serialize(ms);
                    socket->send(zmq::buffer(data), zmq::send_flags::dontwait);
                    break;
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
