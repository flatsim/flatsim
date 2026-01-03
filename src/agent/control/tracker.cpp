#include "flatsim/agent/control/tracker.hpp"
#include "flatsim/agent/machine.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace agent {

    void Tracker::init(types::Machine *config, drivekit::TrackerType type,
                       std::shared_ptr<rerun::RecordingStream> rec) {
        machine_ = config;
        rec_ = rec;
        tracker_ = std::make_unique<drivekit::Tracker>(type);

        // Build robot constraints from machine config
        drivekit::RobotConstraints constraints;

        // Determine steering type - check if differential drive
        bool is_differential_drive = true;
        for (const auto &wheel : config->wheels) {
            if (std::abs(wheel.steering_max) > 1e-6f) {
                is_differential_drive = false;
                break;
            }
        }
        constraints.steering_type =
            is_differential_drive ? drivekit::SteeringType::DIFFERENTIAL : drivekit::SteeringType::ACKERMANN;

        // Derive geometry from wheel positions
        if (!config->wheels.empty()) {
            double max_y = -std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double min_x = std::numeric_limits<double>::infinity();

            for (const auto &w : config->wheels) {
                max_y = std::max(max_y, static_cast<double>(w.bound.pose.point.y));
                min_y = std::min(min_y, static_cast<double>(w.bound.pose.point.y));
                max_x = std::max(max_x, static_cast<double>(w.bound.pose.point.x));
                min_x = std::min(min_x, static_cast<double>(w.bound.pose.point.x));
            }

            double wheelbase = std::abs(max_y - min_y);
            double track_width = std::abs(max_x - min_x);

            constraints.wheelbase = wheelbase > 0.1 ? wheelbase : 1.0;
            constraints.track_width = track_width > 0.1 ? track_width : 1.0;
        } else {
            constraints.wheelbase = 1.5;
            constraints.track_width = 1.5;
        }

        // Use normalized velocity units ([-1,1] maps to full throttle)
        constraints.max_linear_velocity = 1.0;
        constraints.min_linear_velocity = -1.0;
        constraints.max_linear_acceleration = 1.0;
        constraints.max_angular_velocity = 1.0;

        // Steering limits from wheels
        double max_steer = 0.0;
        for (const auto &wheel : config->wheels) {
            max_steer = std::max(max_steer, static_cast<double>(std::abs(wheel.steering_max)));
        }
        if (max_steer <= 0.0) {
            max_steer = 30.0 * M_PI / 180.0;
        }
        constraints.max_steering_angle = max_steer;
        constraints.max_steering_rate = 1.0;

        constraints.min_turning_radius = config->turning_radius;
        constraints.robot_length = config->bound.size.y;
        constraints.robot_width = config->bound.size.x;

        // Initialize tracker with constraints and rerun
        tracker_->init(constraints, rec_, config->uuid);

        // Configure controller settings
        drivekit::ControllerConfig ctrl_config;
        ctrl_config.allow_reverse = false;
        tracker_->get_controller()->set_config(ctrl_config);
    }

    void Tracker::set_controller_type(drivekit::TrackerType type) {
        if (tracker_) {
            tracker_->set_controller_type(type);
        }
    }

    std::pair<float, float> Tracker::update(const datapod::Pose &current_pose, float linear_vel, float angular_vel,
                                            float dt) {
        if (!enabled_ || !tracker_) {
            return {0.0f, 0.0f};
        }

        // Convert current pose to drivekit RobotState
        drivekit::RobotState state;
        state.pose = current_pose;
        state.velocity.linear = linear_vel;
        state.velocity.angular = angular_vel;
        state.timestamp = 0.0;

        // Update tracker
        auto cmd = tracker_->tick(state, dt);

        // Debug output every 60 calls
        static int debug_count = 0;
        if (debug_count++ % 60 == 0) {
        }

        // Only return command if valid
        if (!cmd.valid) {
            return {0.0f, 0.0f};
        }

        // Return normalized velocity command
        return {static_cast<float>(cmd.linear_velocity), static_cast<float>(cmd.angular_velocity)};
    }

} // namespace agent
