#include "multiverse/robot/chasis/karosserie.hpp"

namespace mvs {
    Karosserie::Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}

    void Karosserie::init(concord::Bound parent, concord::Bound bound, muli::CollisionFilter filter, pigment::RGB color,
                          std::string name) {
        this->name = name;
        this->color = color;
        this->bound = bound;

        auto karosseriePosition = shift(parent, bound);
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
    }

    muli::Transform Karosserie::shift(concord::Bound parent, concord::Bound child) {
        muli::Rotation p_rotation(parent.pose.angle.yaw);
        muli::Vec2 rotatedOffset;
        rotatedOffset.x = bound.pose.point.enu.x * p_rotation.c - bound.pose.point.enu.y * p_rotation.s;
        rotatedOffset.y = bound.pose.point.enu.x * p_rotation.s + bound.pose.point.enu.y * p_rotation.c;
        // Add the rotated offset to the car's position
        muli::Vec2 wheelPosition;
        wheelPosition.x = parent.pose.point.enu.x + rotatedOffset.x;
        wheelPosition.y = parent.pose.point.enu.y + rotatedOffset.y;
        concord::Pose wheel_pose;
        wheel_pose.point.enu.x = wheelPosition.x;
        wheel_pose.point.enu.y = wheelPosition.y;
        wheel_pose.angle.yaw = 0.0f; // TODO: fix thi
        muli::Rotation rotation(wheel_pose.angle.yaw);
        muli::Transform wheelTf{wheelPosition, rotation};
        return wheelTf;
    }

    void Karosserie::teleport(concord::Pose pose) {
        muli::Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw;
        muli::Vec2 rotatedOffset;
        rotatedOffset.x = bound.pose.point.enu.x * t.rotation.c - bound.pose.point.enu.y * t.rotation.s;
        rotatedOffset.y = bound.pose.point.enu.x * t.rotation.s + bound.pose.point.enu.y * t.rotation.c;
        // Add the rotated offset to the car's position
        muli::Vec2 wheelPosition;
        wheelPosition.x = pose.point.enu.x + rotatedOffset.x;
        wheelPosition.y = pose.point.enu.y + rotatedOffset.y;
        karosserie->SetTransform(muli::Transform{wheelPosition, t.rotation});
        karosserie->SetSleeping(true);
    }

    void Karosserie::visualize() {
        auto k_x = karosserie->GetPosition().x;
        auto k_y = karosserie->GetPosition().y;
        auto k_th = karosserie->GetRotation().GetAngle();
        auto k_w = float(bound.size.x * 1.3);
        auto k_h = float(bound.size.y * 1.3);
        std::vector<rerun::Color> colors_a;
        colors_a.push_back(rerun::Color(color.r, color.g, color.b));
        rec->log_static(
            this->name + "/karosserie",
            rerun::Boxes3D::from_centers_and_half_sizes({{k_x, k_y, 0}}, {{k_w / 2, k_h / 2, 0.0f}})
                .with_radii({{0.02f}})
                // .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(k_th))})
                .with_colors(colors_a));
    }
} // namespace mvs
