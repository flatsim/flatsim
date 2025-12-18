#include "flatsim/agent/machine/hitch.hpp"

namespace agent {

    Hitch::Hitch(const types::Hitch &config, const types::Machine &machine_config,
                 std::shared_ptr<rerun::RecordingStream> rec)
        : config_(config), machine_config_(machine_config), rec_(rec) {}

    void Hitch::tick(float dt) {
        // Future: Monitor connection status, etc.
    }

    void Hitch::tock() {
        if (!rec_) return;

        // Visualize hitch
        float x = static_cast<float>(config_.pose.point.x);
        float y = static_cast<float>(config_.pose.point.y);
        float th = static_cast<float>(config_.pose.angle.yaw);
        float w = static_cast<float>(config_.size.x);
        float h = static_cast<float>(config_.size.y);

        std::string entity_path = machine_config_.uuid + "/chassis/hitch/" + config_.name;
        bool is_hooked = is_connected();
        rec_->log_static(
            entity_path,
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{w, h, 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(is_hooked ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors({rerun::Color(config_.color.r, config_.color.g, config_.color.b)}));
    }

} // namespace agent
