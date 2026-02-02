#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "flatsim/simulator/data.hpp"
#include "flatsim/simulator/machine/chassis.hpp"
#include "flatsim/types.hpp"
#include "flywheel/flywheel.hpp"
#include <datapod/robot.hpp>
#include <optional>
#include <rerun.hpp>

namespace simulator {

    /// Validate that a parsed URDF model contains the flatsim-required extensions.
    /// Throws `std::runtime_error` with a human-readable message on failure.
    void validate_model_for_flatsim(const datapod::robot::Model &model);

    /// Build a legacy `types::Machine` config from a parsed URDF model.
    ///
    /// Note: this is a transitional adapter until the simulation stack consumes
    /// `datapod::robot::Model` directly.
    types::Machine machine_from_model(const datapod::robot::Model &model, const datapod::Pose &spawn_pose,
                                      std::optional<pigment::RGB> color);

    /// Convert twist command (linear/angular velocity) to per-wheel control.
    /// Handles both Ackermann and differential drive configurations.
    /// @param config Machine configuration with wheel layout and control limits
    /// @param linear Linear velocity command [-1, 1]
    /// @param angular Angular velocity command [-1, 1]
    /// @return WheelControl with per-wheel steering and throttle values
    types::WheelControl twist_to_wheel_control(const types::Machine &config, float linear, float angular);

    // Forward declare
    class Data;

    class Machine {
      private:
        std::shared_ptr<rerun::RecordingStream> rec_;
        std::shared_ptr<flywheel::World> world_;
        flywheel::CollisionFilter filter_;

        types::Machine config_;
        types::State state_;
        types::SensorData sensor_data_;

        // For IMU acceleration calculation
        float prev_linear_vel_ = 0.0f;

        std::unique_ptr<Chassis> chassis_;

      public:
        Machine() = default;
        Machine(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<flywheel::World> world,
                const types::Machine &config, uint32_t group);

        // Lifecycle
        void create();
        void destroy();

        // Apply control inputs
        void apply_control(const types::WheelControl &control, float dt);

        // Tick/tock pattern
        void tick(float dt);
        void tock(datapod::Geo datum);

        // Get state for feedback
        types::ser::MachineState get_state() const;

        // Get sensor data (filled by update_sensors)
        const types::SensorData &get_sensor_data() const { return sensor_data_; }

        // Update sensor data using Data helper
        void update_sensors(Data &data, const datapod::Geo &datum, float dt);

        // Find hitch by name
        Hitch *find_hitch(const std::string &name);

        // Accessors
        flywheel::RigidBody *body() const { return chassis_ ? chassis_->body : nullptr; }
        const types::Machine &config() const { return config_; }
        types::Machine &config_mut() { return config_; }
        const types::State &state() const { return state_; }
        types::State &state_mut() { return state_; }
        const std::string &uuid() const { return config_.uuid; }
        Chassis *chassis() { return chassis_.get(); }
        const Chassis *chassis() const { return chassis_.get(); }
        const flywheel::CollisionFilter &get_filter() const { return filter_; }

        // Teleport machine to new pose
        void teleport(const datapod::Pose &pose);

        // Apply braking
        void brake(float brake_force);

        // Update color
        void update_color(const pigment::RGB &new_color);
    };

} // namespace simulator
