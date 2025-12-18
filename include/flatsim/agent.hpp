#pragma once

#include <memory>
#include <zmq.hpp>

#include "flatsim/types.hpp"

namespace agent {

    class Agent {
      private:
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        std::string address_;
        types::Machine machine_;

      public:
        Agent(const std::string &address = "");
        ~Agent();

        void set_machine(const types::Machine &machine);
        const types::Machine &machine() const { return machine_; }

        bool spawn();
        bool despawn();
        bool control(const types::MachineControl &ctrl);

        void tick(float dt);
        void tock();

        // Update machine state from simulator feedback
        void update_state(const types::ser::MachineState &state);
    };

} // namespace agent
