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
        // Connection mode
        bool local_mode_ = false;

        // ZMQ sockets (only used in networked mode)
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> spawn_socket_;     // REQ - for spawn/despawn
        std::unique_ptr<zmq::socket_t> control_socket_;   // PUSH - for sending controls
        std::unique_ptr<zmq::socket_t> state_socket_;     // SUB - for receiving state
        std::unique_ptr<zmq::socket_t> heartbeat_socket_; // PUSH - for heartbeats
        std::string address_;

        // Core components
        Machine machine_;
        ControlManager control_manager_;
        bool spawned_ = false;

        // Rerun visualization
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        // Constructor for networked mode (IPC/TCP)
        Agent(const std::string &address = "");

        // Constructor for local mode (owned by Simulator)
        Agent(const types::Machine &config, std::shared_ptr<rerun::RecordingStream> rec);

        ~Agent();

        // Machine configuration
        void set_machine(const types::Machine &config);
        Machine &machine() { return machine_; }
        const Machine &machine() const { return machine_; }

        // Networked mode: spawn/despawn via IPC
        bool spawn();
        bool despawn();

        // Control interface
        void set_linear(float linear);
        void set_angular(float angular);
        void set_velocity(float linear, float angular);

        // Control manager access
        ControlManager &controls() { return control_manager_; }
        const ControlManager &controls() const { return control_manager_; }

        // Tick/tock pattern
        // Networked mode: tick() blocks until state message received from simulator
        // Local mode: tick() just runs navigation (state updated by Simulator)
        void tick(float dt, int timeout_ms = 100);
        void tock();

        // Transport abstraction - called by Simulator
        // LOCAL mode: direct state update
        // IPC/TCP mode: state already received via recv_state()
        void update_from_physics(const types::ser::MachineState &state);

        // Get current wheel control (used by Simulator in LOCAL mode)
        types::WheelControl get_wheel_control() const;

        // Check if in local mode
        bool is_local() const { return local_mode_; }

      private:
        // Transport abstraction - handles LOCAL vs IPC/TCP internally
        bool recv_state(int timeout_ms);
        void send_control(const types::WheelControl &ctrl);
        void send_heartbeat();
    };

} // namespace agent
