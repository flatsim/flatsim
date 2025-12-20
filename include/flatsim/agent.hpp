#pragma once

#include <memory>
#include <rerun.hpp>
#include <zmq.hpp>

#include "flatsim/agent/control_manager.hpp"
#include "flatsim/agent/machine.hpp"
#include "flatsim/types.hpp"

namespace agent {

    class Agent {
      private:
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> spawn_socket_;     // REQ - for spawn/despawn
        std::unique_ptr<zmq::socket_t> control_socket_;   // PUSH - for sending controls
        std::unique_ptr<zmq::socket_t> state_socket_;     // SUB - for receiving state
        std::unique_ptr<zmq::socket_t> heartbeat_socket_; // PUSH - for heartbeats
        std::string address_;
        Machine machine_;
        ControlManager control_manager_;
        bool spawned_ = false;

        // Rerun visualization
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        Agent(const std::string &address = "", std::shared_ptr<rerun::RecordingStream> rec = nullptr);
        ~Agent();

        void set_machine(const types::Machine &config);
        Machine &machine() { return machine_; }
        const Machine &machine() const { return machine_; }

        bool spawn();
        bool despawn();

        // Control interface
        void set_linear(float linear);
        void set_angular(float angular);
        void set_velocity(float linear, float angular);

        // Tick/tock pattern - tick() blocks until state message received from simulator
        void tick(float dt, int timeout_ms = 100);
        void tock();
    };

} // namespace agent
