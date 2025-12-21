#include "flatsim/agent/machine.hpp"
#include <cmath>

namespace agent {

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, const types::Machine &config)
        : rec_(rec), config_(config) {
        world_pose_ = config_.bound.pose;
    }

    void Machine::update_state(const types::ser::MachineState &state) { world_pose_ = state.pose.to_concord(); }

    void Machine::tick(float dt) {
        (void)dt;
        // Tick is called after receiving state update from simulator
        // Local agent-side processing can go here (e.g., sensor fusion, prediction)
    }

    void Machine::tock() {
        if (!rec_) return;

        // Visualize robot chassis as a box
        auto x = float(world_pose_.point.x);
        auto y = float(world_pose_.point.y);
        auto th = float(world_pose_.angle.yaw);
        auto w = float(config_.bound.size.x);
        auto h = float(config_.bound.size.y);

        rec_->log_static(
            config_.uuid + "/chassis",
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{w, h, 0.0f}})
                .with_radii({{0.02f}})
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors({rerun::Color(config_.color.r, config_.color.g, config_.color.b)}));

        // Visualize wheels if available
        for (size_t i = 0; i < config_.wheels.size(); ++i) {
            const auto &wheel = config_.wheels[i];

            // Transform wheel position from local to world coordinates
            float cos_th = std::cos(th);
            float sin_th = std::sin(th);
            float wheel_local_x = float(wheel.bound.pose.point.x);
            float wheel_local_y = float(wheel.bound.pose.point.y);

            float wheel_world_x = x + wheel_local_x * cos_th - wheel_local_y * sin_th;
            float wheel_world_y = y + wheel_local_x * sin_th + wheel_local_y * cos_th;
            float wheel_th = th + float(wheel.bound.pose.angle.yaw);

            float wheel_w = float(wheel.bound.size.x);
            float wheel_h = float(wheel.bound.size.y);

            rec_->log_static(config_.uuid + "/chassis/wheel/" + wheel.name,
                             rerun::Boxes3D::from_centers_and_sizes({{wheel_world_x, wheel_world_y, 0.1f}},
                                                                    {{wheel_w, wheel_h, 0.0f}})
                                 .with_radii({{0.02f}})
                                 .with_fill_mode(rerun::FillMode::Solid)
                                 .with_rotation_axis_angles(
                                     {rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(wheel_th))})
                                 .with_colors({rerun::Color(config_.color.r, config_.color.g, config_.color.b)}));
        }
    }

} // namespace agent
