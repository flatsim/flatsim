#include "flatsim/simulator/machine/hitch.hpp"
#include "flatsim/utils.hpp"

namespace simulator {
    Hitch::Hitch(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<flywheel::World> world,
                 types::Machine *robot_info, types::State *robot_state)
        : rec(rec), world(world), robot_info(robot_info), robot_state(robot_state) {}

    void Hitch::init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                     datapod::Box parent_bound, datapod::Box bound, flywheel::CollisionFilter filter, bool is_master) {
        this->name = name;
        this->parent_name = parent_name;
        this->color = color;
        this->bound = bound;
        this->is_master = is_master;

        pose = utils::shift(parent_bound.pose, bound.pose);
    }

    void Hitch::tick(float dt, datapod::Pose trans_pose) {
        auto new_pose = utils::move(bound.pose, trans_pose);
        pose.point.x = new_pose.point.x;
        pose.point.y = new_pose.point.y;
        utils::set_yaw(pose, utils::get_yaw(new_pose));
    }

    void Hitch::teleport(datapod::Pose trans_pose) { pose = trans_pose; }

    void Hitch::tock() {
        if (!robot_state->online) return;
        if (!rec) return;

        auto k_x = pose.point.x;
        auto k_y = pose.point.y;
        auto k_th = utils::get_yaw(pose);
        auto k_w = float(bound.size.x);
        auto k_h = float(bound.size.y);
        rec->log_static(
            robot_info->uuid + "/chassis/hitch/" + name,
            rerun::Boxes3D::from_centers_and_sizes({{float(k_x), float(k_y), 0.1f}}, {{float(k_w), float(k_h), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(this->hooked ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(k_th))})
                .with_colors({rerun::Color(color.r(), color.g(), color.b())}));
    }
} // namespace simulator
