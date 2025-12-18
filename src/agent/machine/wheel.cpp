#include "flatsim/agent/machine/wheel.hpp"

namespace agent {

    Wheel::Wheel(const types::Wheel &config, const types::Machine &machine_config,
                 std::shared_ptr<rerun::RecordingStream> rec)
        : config_(config), machine_config_(machine_config), rec_(rec) {}

    void Wheel::update_state(const types::ser::WheelState &state) { world_pose_ = state.pose.to_concord(); }

    void Wheel::tick(float dt) {
        // Future: Process sensor data, compute odometry, etc.
    }

    void Wheel::tock() {
        if (!rec_) return;

        float x = static_cast<float>(world_pose_.point.x);
        float y = static_cast<float>(world_pose_.point.y);
        float th = static_cast<float>(world_pose_.angle.yaw);
        float w = static_cast<float>(config_.size.x);
        float h = static_cast<float>(config_.size.y);

        // Use wheel color if set, otherwise default to machine color
        pigment::RGB color = (config_.color.r == 0 && config_.color.g == 0 && config_.color.b == 0)
                                 ? machine_config_.color
                                 : config_.color;

        std::string entity_path = machine_config_.uuid + "/chassis/wheel/" + config_.name;
        rec_->log_static(entity_path, rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{w, h, 0.0f}})
                                          .with_radii({{0.02f}})
                                          .with_fill_mode(rerun::FillMode::Solid)
                                          .with_rotation_axis_angles(
                                              {rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                                          .with_colors({rerun::Color(color.r, color.g, color.b)}));
    }

} // namespace agent
