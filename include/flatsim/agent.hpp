#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <rerun.hpp>
#include <utility>

#include "flatsim/types.hpp"
#include <agent47.hpp>
#include <datapod/robot.hpp>
#include <drivekit/tracker.hpp>

namespace agent {

    class Agent {
      public:
        std::shared_ptr<agent47::Agent> agent47_;

      private:
        bool local_mode_ = false;
        std::optional<std::string> agent47_endpoint_;
        bool spawned_ = false;
        std::shared_ptr<rerun::RecordingStream> rec_;

        // Machine state (updated from simulator)
        types::Machine config_;
        datapod::Pose world_pose_;
        float linear_velocity_ = 0.0f;
        float angular_velocity_ = 0.0f;
        types::SensorData sensor_data_;

        // Control state
        float cmd_linear_ = 0.0f;
        float cmd_angular_ = 0.0f;
        float speed_scale_ = 1.0f;

        // Path following (uses agent47's drivekit)
        std::unique_ptr<drivekit::Tracker> tracker_;
        bool tracker_enabled_ = false;

      public:
        Agent(const types::Machine &config, std::shared_ptr<rerun::RecordingStream> rec);
        Agent(dp::String urdf_path, dp::robot::Identity identity, agent47::Bridge *bridge,
              std::shared_ptr<rerun::RecordingStream> rec);

        // Networked mode via agent47 PipeBridge
        Agent(const types::Machine &config, const std::string &agent47_endpoint,
              std::shared_ptr<rerun::RecordingStream> rec);

        ~Agent();

        // Networked mode: spawn/despawn via IPC
        bool spawn();
        bool despawn();

        // Control interface
        void set_linear(float linear) { cmd_linear_ = linear; }
        void set_angular(float angular) { cmd_angular_ = angular; }
        void set_velocity(float linear, float angular) {
            cmd_linear_ = linear;
            cmd_angular_ = angular;
        }

        // Tick/tock pattern
        void tick(float dt, int timeout_ms = 100);
        void tock();

        // Transport abstraction - called by Simulator
        void update_from_physics(const types::ser::MachineState &state);
        void update_from_sensors(const types::ser::SensorState &state);

        // Get current twist command (used by Simulator to convert to wheel control)
        std::pair<float, float> get_twist() const { return {cmd_linear_ * speed_scale_, cmd_angular_}; }

        // Get sensor data
        const types::SensorData &get_sensor_data() const { return sensor_data_; }

        // Check mode
        bool is_local() const { return local_mode_; }

        // ============================================================================
        // State accessors
        // ============================================================================

        const types::Machine &config() const { return config_; }
        const datapod::Pose &get_position() const { return world_pose_; }
        float get_linear_velocity() const { return linear_velocity_; }
        float get_angular_velocity() const { return angular_velocity_; }
        void get_velocity(float &linear, float &angular) const {
            linear = linear_velocity_;
            angular = angular_velocity_;
        }

        // ============================================================================
        // Tracker / Navigation
        // ============================================================================

        drivekit::Tracker *tracker() { return tracker_.get(); }
        const drivekit::Tracker *tracker() const { return tracker_.get(); }
        void set_tracker_enabled(bool enabled) { tracker_enabled_ = enabled; }
        bool is_tracker_enabled() const { return tracker_enabled_; }

        // ============================================================================
        // Speed / Braking
        // ============================================================================

        void set_speed(float scale) { speed_scale_ = std::clamp(scale, 0.0f, 1.0f); }
        float get_speed() const { return speed_scale_; }
        void speed_up(float delta = 0.1f) { set_speed(speed_scale_ + delta); }
        void slow_down(float delta = 0.1f) { set_speed(speed_scale_ - delta); }
        void brake() {
            cmd_linear_ = 0.0f;
            cmd_angular_ = 0.0f;
        }

        // ============================================================================
        // Identity
        // ============================================================================

        const std::string &uuid() const { return config_.uuid; }
        const std::string &name() const { return config_.name; }

        // ============================================================================
        // URDF
        // ============================================================================

        static datapod::robot::Model load_model_from_urdf(const std::filesystem::path &urdf_path);
    };

} // namespace agent
