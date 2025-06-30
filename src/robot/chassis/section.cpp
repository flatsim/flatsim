#include "flatsim/robot/chassis/section.hpp"
#include "flatsim/utils.hpp"

namespace fs {
    Section::Section(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}

    void Section::init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                       concord::Bound section_bound, int id) {
        this->name = name;
        this->parent_name = parent_name;
        this->color = color;
        this->bound = section_bound;
        this->section_id = id;
        this->pose = section_bound.pose;
    }

    void Section::tick(float dt, concord::Pose trans_pose) {
        auto new_pose = utils::move(bound.pose, trans_pose);

        pose.point.x = new_pose.point.x;
        pose.point.y = new_pose.point.y;
        pose.angle.yaw = new_pose.angle.yaw;
    }

    void Section::teleport(concord::Pose trans_pose) { pose = trans_pose; }

    void Section::tock() {
        auto s_x = pose.point.x;
        auto s_y = pose.point.y;
        auto s_th = pose.angle.yaw;
        auto s_w = float(bound.size.x);
        auto s_h = float(bound.size.y);

        std::vector<rerun::Color> colors_a;
        colors_a.push_back(rerun::Color(color.r, color.g, color.b));

        std::vector<rerun::components::PoseTranslation3D> centers = {
            rerun::components::PoseTranslation3D(float(s_x), float(s_y), 0.1f)};
        std::vector<rerun::datatypes::Vec3D> sizes = {rerun::datatypes::Vec3D(float(s_w), float(s_h), 0.0f)};

        rec->log_static(
            this->parent_name + "/chassis/karosserie/" + name + "/section_" + std::to_string(section_id),
            rerun::Boxes3D::from_centers_and_sizes(centers, sizes)
                .with_radii({{0.02f}})
                .with_fill_mode(this->working ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(s_th))})
                .with_colors(colors_a));
    }
} // namespace fs
