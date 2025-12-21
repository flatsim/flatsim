#pragma once

#include <map>
#include <memory>
#include <rerun.hpp>
#include <vector>
#include <zmq.hpp>

#include "flatsim/simulator/machine.hpp"
#include "flatsim/simulator/world.hpp"
#include "flatsim/types.hpp"

namespace simulator {

    enum class Conn { TCP, IPC };

    // Simple settings for Simulator constructor (not to be confused with simulator::WorldSettings)
    struct SimulatorSettings {
        float width = 100.0f;
        float height = 100.0f;
    };

    class Simulator {
      private:
        // ZMQ
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> spawn_socket_;                           // REP - for spawn/despawn requests
        std::unique_ptr<zmq::socket_t> heartbeat_socket_;                       // PULL - for heartbeat messages
        std::map<std::string, std::unique_ptr<zmq::socket_t>> control_sockets_; // PULL per-robot
        std::map<std::string, std::unique_ptr<zmq::socket_t>> state_sockets_;   // PUB per-robot
        Conn conn_;
        std::string address_;
        int next_tcp_port_ = 5600;

        // Physics world with obstacle management
        std::unique_ptr<World> world_;
        SimulatorSettings sim_settings_;

        // Machines: uuid -> Machine
        std::map<std::string, Machine> machines_;

        // Heartbeat tracking: uuid -> last heartbeat time
        std::map<std::string, std::chrono::steady_clock::time_point> last_heartbeat_;

        // Next collision group
        uint32_t next_group_ = 1;

        // Rerun visualization
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        Simulator(Conn conn, const std::string &address = "", const SimulatorSettings &settings = {},
                  std::shared_ptr<rerun::RecordingStream> rec = nullptr);
        ~Simulator();

        void tick(float dt);
        void tock();

        // Create machine with wheels, karosseries, etc.
        void create_machine(const types::Machine &machine);

        // Apply control to a machine
        void apply_control(const types::WheelControl &control, float dt);

        // Destroy a machine
        bool destroy_machine(const std::string &uuid);

        // Get world state for feedback
        types::ser::WorldState get_world_state() const;

        // Access to physics world and world wrapper
        muli::World &get_world() { return world_->physics(); }
        World &world() { return *world_; }
    };

} // namespace simulator
