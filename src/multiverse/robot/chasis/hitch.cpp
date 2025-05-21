#include "multiverse/robot/chasis/hitch.hpp"

namespace mvs {
    Hitch::Hitch(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}

    void Hitch::init(const pigment::RGB &color, std::string parent_name, std::string name, concord::Bound parent_bound,
                     concord::Bound bound, muli::CollisionFilter filter) {
        this->name = name;
        this->parent_name = parent_name;
        this->color = color;
        this->bound = bound;

        pose = utils::shift(parent_bound.pose, bound.pose);
    }

    void Hitch::tick(float dt, concord::Pose trans_pose) {
        auto new_pose = utils::move(bound.pose, trans_pose);

        pose.point.enu.x = new_pose.point.enu.x;
        pose.point.enu.y = new_pose.point.enu.y;
        pose.angle.yaw = new_pose.angle.yaw;

        visualize();
    }

    void Hitch::teleport(concord::Pose trans_pose) { pose = trans_pose; }

    void Hitch::visualize() {
        auto k_x = pose.point.enu.x;
        auto k_y = pose.point.enu.y;
        auto k_th = pose.angle.yaw;
        auto k_w = float(bound.size.x);
        auto k_h = float(bound.size.y);
        std::vector<rerun::Color> colors_a;
        colors_a.push_back(rerun::Color(color.r, color.g, color.b));
        rec->log_static(
            this->parent_name + "/chasis/hitch/" + name,
            rerun::Boxes3D::from_centers_and_sizes({{float(k_x), float(k_y), 0.1f}}, {{float(k_w), float(k_h), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(this->hooked ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(k_th))})
                .with_colors(colors_a));
    }
} // namespace mvs
