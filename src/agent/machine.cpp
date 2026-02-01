#include "flatsim/agent/machine.hpp"
#include "flatsim/utils.hpp"
#include <cmath>
#include <iostream>

namespace agent {

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, const types::Machine &config)
        : rec_(rec), config_(config) {
        world_pose_ = config_.bound.pose;
    }

    void Machine::init() {
        // Initialize sensor manager with robot UUID (auto-enables SHM for sensors added later)
        sensors.set_robot_uuid(config_.uuid);

        // Initialize control manager
        controls.init(&config_, rec_);

        // network/power/container managers removed
    }

    void Machine::update_state(const types::ser::MachineState &state) {
        world_pose_ = state.pose.to_datapod();
        // Compute forward velocity along heading from 2D velocity vector
        float yaw = utils::get_yaw(world_pose_);
        linear_velocity_ = state.velocity.x * std::cos(yaw) + state.velocity.y * std::sin(yaw);
        angular_velocity_ = state.angular_vel;
    }

    void Machine::tick(float dt) {
        // Update sensors with current pose and physics data (fallback when no simulator data)
        float yaw = utils::get_yaw(world_pose_);
        float vel_x = linear_velocity_ * std::cos(yaw);
        float vel_y = linear_velocity_ * std::sin(yaw);
        sensors.update_all_with_physics(world_pose_, vel_x, vel_y, angular_velocity_, dt);

        // network/power/container managers removed

        // Update control manager (navigation, path following)
        if (navigation_enabled_) {
            controls.tick(world_pose_, linear_velocity_, angular_velocity_, dt);
        }
    }

    void Machine::tick(float dt, const types::SensorData &sensor_data) {
        // Update sensors with data from simulator (preferred path in LOCAL mode)
        sensors.update_from_simulator(sensor_data, dt);

        // network/power/container managers removed

        // Update control manager (navigation, path following)
        if (navigation_enabled_) {
            controls.tick(world_pose_, linear_velocity_, angular_velocity_, dt);
        }
    }

    void Machine::tock() {
        // Visualize tracker/path
        controls.tock(rec_);
    }

} // namespace agent
