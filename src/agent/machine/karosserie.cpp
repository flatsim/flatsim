#include "flatsim/agent/machine/karosserie.hpp"

namespace agent {

    Karosserie::Karosserie(const types::Karosserie &config, const types::Machine &machine_config,
                           std::shared_ptr<rerun::RecordingStream> rec)
        : config_(config), machine_config_(machine_config), rec_(rec) {}

    void Karosserie::tick(float dt) {
        // Future: Process sensor data, etc.
    }

    void Karosserie::tock() {
        if (!rec_) return;

        // Visualize karosserie body
        float x = static_cast<float>(config_.pose.point.x);
        float y = static_cast<float>(config_.pose.point.y);
        float th = static_cast<float>(config_.pose.angle.yaw);
        float w = static_cast<float>(config_.size.x);
        float h = static_cast<float>(config_.size.y);

        // Use karosserie color if set, otherwise default to machine color
        pigment::RGB color = (config_.color.r == 0 && config_.color.g == 0 && config_.color.b == 0)
                                 ? machine_config_.color
                                 : config_.color;

        std::string entity_path = machine_config_.uuid + "/chassis/karosserie/" + config_.name;
        // TODO: Add 'working' state field to types::Karosserie, for now default to wireframe
        bool working = false;
        rec_->log_static(
            entity_path,
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{w, h, 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(working ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors({rerun::Color(color.r, color.g, color.b)}));
    }

} // namespace agent
