#include "multiverse/robot/chasis/karosserie.hpp"

namespace mvs {
    Karosserie::Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}

    void Karosserie::init(const pigment::RGB &color, std::string parent_name, std::string name,
                          concord::Bound parent_bound, concord::Bound bound, muli::CollisionFilter filter) {
        this->name = name;
        this->parent_name = parent_name;
        this->color = color;
        this->bound = bound;

        auto shifted = utils::shift(parent_bound.pose, bound.pose);
        auto karosseriePosition = utils::pose_to_transform(shifted);
        karosserie = world->CreateBox(bound.size.x, bound.size.y, karosseriePosition);
        karosserie->SetCollisionFilter(filter);
    }

    void Karosserie::tick(float dt, muli::Transform t) {
        // Transform the local offset to world coordinates
        muli::Vec2 rotatedOffset;
        rotatedOffset.x = bound.pose.point.enu.x * t.rotation.c - bound.pose.point.enu.y * t.rotation.s;
        rotatedOffset.y = bound.pose.point.enu.x * t.rotation.s + bound.pose.point.enu.y * t.rotation.c;
        // Add the rotated offset to the car's position
        muli::Vec2 karosseriePosition;
        karosseriePosition.x = t.position.x + rotatedOffset.x;
        karosseriePosition.y = t.position.y + rotatedOffset.y;
        karosserie->SetTransform(muli::Transform{karosseriePosition, t.rotation});

        pose.point.enu.x = karosseriePosition.x;
        pose.point.enu.y = karosseriePosition.y;
        pose.angle.yaw = t.rotation.GetAngle();

        visualize();
    }

    void Karosserie::tick(float dt, concord::Pose trans_pose) {
        concord::Pose rotated_offset;
        rotated_offset.point.enu.x = bound.pose.point.enu.x * std::cos(trans_pose.angle.yaw) -
                                     bound.pose.point.enu.y * std::sin(trans_pose.angle.yaw);
        rotated_offset.point.enu.y = bound.pose.point.enu.x * std::sin(trans_pose.angle.yaw) +
                                     bound.pose.point.enu.y * std::cos(trans_pose.angle.yaw);

        concord::Pose new_pose;
        new_pose.point.enu.x = trans_pose.point.enu.x + rotated_offset.point.enu.x;
        new_pose.point.enu.y = trans_pose.point.enu.y + rotated_offset.point.enu.y;
        new_pose.angle.yaw = trans_pose.angle.yaw;

        karosserie->SetTransform(utils::pose_to_transform(new_pose));

        pose.point.enu.x = new_pose.point.enu.x;
        pose.point.enu.y = new_pose.point.enu.y;
        pose.angle.yaw = new_pose.angle.yaw;

        visualize();
    }

    void Karosserie::teleport(concord::Pose trans_pose) {
        concord::Pose t;
        t.point.enu.x = trans_pose.point.enu.x;
        t.point.enu.y = trans_pose.point.enu.y;
        t.angle.yaw = trans_pose.angle.yaw;

        concord::Pose rotated_offset;
        rotated_offset.point.enu.x =
            bound.pose.point.enu.x * std::cos(t.angle.yaw) - bound.pose.point.enu.y * std::sin(t.angle.yaw);
        rotated_offset.point.enu.y =
            bound.pose.point.enu.x * std::sin(t.angle.yaw) + bound.pose.point.enu.y * std::cos(t.angle.yaw);

        concord::Pose new_pose;
        new_pose.point.enu.x = trans_pose.point.enu.x + rotated_offset.point.enu.x;
        new_pose.point.enu.y = trans_pose.point.enu.y + rotated_offset.point.enu.y;
        new_pose.angle.yaw = t.angle.yaw;

        karosserie->SetTransform(utils::pose_to_transform(new_pose));
        karosserie->SetSleeping(true);
    }

    muli::Transform Karosserie::get_transform() const { return karosserie->GetTransform(); }
    muli::RigidBody *Karosserie::get_body() const { return karosserie; }

    void Karosserie::visualize() {
        auto k_x = karosserie->GetPosition().x;
        auto k_y = karosserie->GetPosition().y;
        auto k_th = karosserie->GetRotation().GetAngle();
        auto k_w = float(bound.size.x);
        auto k_h = float(bound.size.y);
        std::vector<rerun::Color> colors_a;
        colors_a.push_back(rerun::Color(color.r, color.g, color.b));
        rec->log_static(
            this->parent_name + "/chasis/karosserie/" + name,
            rerun::Boxes3D::from_centers_and_sizes({{k_x, k_y, 0.1f}}, {{k_w, k_h, 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(this->working ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(k_th))})
                .with_colors(colors_a));
    }
} // namespace mvs
