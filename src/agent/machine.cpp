#include "flatsim/agent/machine.hpp"
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

        // Initialize network manager
        network.init(config_.uuid);

        // Initialize power manager if power config exists
        if (config_.power_source.has_value()) {
            power.init(config_.power_source.value());
        }

        // Initialize container manager if tank/container config exists
        if (config_.tank.has_value()) {
            container.init(config_.tank.value(), config_.color, config_.name);
        }

        std::cout << "[Machine] Initialized all managers for: " << config_.name << " (" << config_.uuid << ")"
                  << std::endl;
    }

    void Machine::update_state(const types::ser::MachineState &state) {
        world_pose_ = state.pose.to_concord();
        // Compute forward velocity along heading from 2D velocity vector
        float yaw = world_pose_.angle.yaw;
        linear_velocity_ = state.velocity.x * std::cos(yaw) + state.velocity.y * std::sin(yaw);
        angular_velocity_ = state.angular_vel;
    }

    void Machine::tick(float dt) {
        // Update sensors with current pose and physics data
        float yaw = world_pose_.angle.yaw;
        float vel_x = linear_velocity_ * std::cos(yaw);
        float vel_y = linear_velocity_ * std::sin(yaw);
        sensors.update_all_with_physics(world_pose_, vel_x, vel_y, angular_velocity_, dt);

        // Update network manager
        network.tick(dt);

        // Update power manager (consume power based on velocity)
        float consumption_mult = std::abs(linear_velocity_) / 10.0f + 0.1f; // Base consumption + velocity-based
        power.update(dt, consumption_mult);

        // Update container manager
        container.tick(dt, world_pose_);

        // Update control manager (navigation, path following)
        if (navigation_enabled_) {
            controls.tick(world_pose_, linear_velocity_, angular_velocity_, dt);
        }
    }

    void Machine::tock() {
        // Visualize container if present
        container.tock(rec_);

        // Visualize tracker/path
        controls.tock(rec_);
    }

} // namespace agent
