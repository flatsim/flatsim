#include "flatsim/simulator/machine/section.hpp"
#include "flatsim/utils.hpp"

namespace simulator {
    Section::Section(std::shared_ptr<rerun::RecordingStream> rec, types::Machine *robot_info, types::State *robot_state)
        : rec(rec), robot_info(robot_info), robot_state(robot_state) {}

    void Section::init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                       datapod::Box section_bound, int id) {
        this->name = name;
        this->parent_name = parent_name;
        this->color = color;
        this->bound = section_bound;
        this->section_id = id;
        this->pose = section_bound.pose;
    }

    void Section::tick(float dt, datapod::Pose trans_pose) {
        auto new_pose = utils::move(bound.pose, trans_pose);

        pose.point.x = new_pose.point.x;
        pose.point.y = new_pose.point.y;
        utils::set_yaw(pose, utils::get_yaw(new_pose));
    }

    void Section::teleport(datapod::Pose trans_pose) { pose = trans_pose; }

    void Section::tock() {
        if (!robot_state->online) return;
        if (!rec) return;

        auto s_x = pose.point.x;
        auto s_y = pose.point.y;
        auto s_th = utils::get_yaw(pose);
        auto s_w = float(bound.size.x);
        auto s_h = float(bound.size.y);

        rec->log_static(
            robot_info->uuid + "/chassis/karosserie/" + name + "/section_" + std::to_string(section_id),
            rerun::Boxes3D::from_centers_and_sizes({{float(s_x), float(s_y), 0.1f}}, {{float(s_w), float(s_h), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(this->working ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(s_th))})
                .with_colors({rerun::Color(color.r(), color.g(), color.b())}));
    }
} // namespace simulator
