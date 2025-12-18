#pragma once

#include <map>
#include <memory>
#include <vector>
#include <zmq.hpp>

#include "flatsim/simulator/machine.hpp"
#include "flatsim/simulator/world.hpp"
#include "flatsim/types.hpp"

namespace simulator {

    enum class Conn { TCP, IPC };

    struct WorldSettings {
        float width = 100.0f;
        float height = 100.0f;
    };

    class Simulator {
      private:
        // ZMQ
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        Conn conn_;
        std::string address_;

        // Physics world with obstacle management
        std::unique_ptr<World> world_;
        WorldSettings world_settings_;

        // Machines: uuid -> Machine
        std::map<std::string, Machine> machines_;

        // Next collision group
        uint32_t next_group_ = 1;

      public:
        Simulator(Conn conn, const std::string &address = "", const WorldSettings &settings = {});
        ~Simulator();

        void tick(float dt);
        void tock();

        // Create machine with wheels, karosseries, etc.
        void create_machine(const types::Machine &machine);

        // Apply control to a machine
        void apply_control(const types::MachineControl &control, float dt);

        // Destroy a machine
        bool destroy_machine(const std::string &uuid);

        // Get world state for feedback
        types::ser::WorldState get_world_state() const;

        // Access to physics world and world wrapper
        muli::World &get_world() { return world_->physics(); }
        World &world() { return *world_; }
    };

} // namespace simulator
