#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <rerun.hpp>
#include <zmq.hpp>

#include "flatsim/agent/control_manager.hpp"
#include "flatsim/agent/machine.hpp"
#include "flatsim/types.hpp"

namespace agent {

    // Teleport callback type (set by Simulator in LOCAL mode)
    using TeleportCallback = std::function<void(const std::string &uuid, const concord::Pose &pose)>;

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

        // Speed control
        float speed_scale_ = 1.0f;

        // Teleport callback (LOCAL mode only - set by Simulator)
        TeleportCallback teleport_callback_;

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

        // ============================================================================
        // Convenience API (shortcuts to avoid deep nesting)
        // ============================================================================

        // Direct access to drivekit::Tracker (shortcut for controls().tracker().tracker())
        drivekit::Tracker *tracker() { return control_manager_.tracker().tracker(); }
        const drivekit::Tracker *tracker() const { return control_manager_.tracker().tracker(); }

        // Position/pose (alias for machine().world_pose())
        const concord::Pose &get_position() const { return machine_.world_pose(); }

        // Velocity access
        float get_linear_velocity() const { return machine_.linear_velocity(); }
        float get_angular_velocity() const { return machine_.angular_velocity(); }
        void get_velocity(float &linear, float &angular) const {
            linear = machine_.linear_velocity();
            angular = machine_.angular_velocity();
        }

        // Speed control (scales velocity commands, 0.0 to 1.0)
        void set_speed(float scale) { speed_scale_ = std::clamp(scale, 0.0f, 1.0f); }
        float get_speed() const { return speed_scale_; }
        void speed_up(float delta = 0.1f) { set_speed(speed_scale_ + delta); }
        void slow_down(float delta = 0.1f) { set_speed(speed_scale_ - delta); }

        // Braking
        void brake();

        // Teleport to a new pose (LOCAL mode: immediate, IPC/TCP: sends request)
        void teleport(const concord::Pose &pose);

        // Set teleport callback (called by Simulator in LOCAL mode)
        void set_teleport_callback(TeleportCallback cb) { teleport_callback_ = std::move(cb); }

        // Navigation enable/disable (shortcut for controls().set_navigation_enabled())
        void set_navigation_enabled(bool enabled) { control_manager_.set_navigation_enabled(enabled); }
        bool is_navigation_enabled() const { return control_manager_.is_navigation_enabled(); }

        // UUID access
        const std::string &uuid() const { return machine_.uuid(); }

        // Name access
        const std::string &name() const { return machine_.config().name; }

      private:
        // Transport abstraction - handles LOCAL vs IPC/TCP internally
        bool recv_state(int timeout_ms);
        void send_control(const types::WheelControl &ctrl);
        void send_heartbeat();
    };

} // namespace agent
