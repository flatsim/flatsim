#include "multiverse/robot/chasis/karosserie.hpp"

namespace mvs {
    Karosserie::Karosserie() {}

    Karosserie::Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}

    void Karosserie::init(concord::Bound bound, muli::CollisionFilter filter, pigment::RGB color, std::string name) {
        this->name = name;
        this->color = color;
        this->bound = bound;
        muli::Transform t;
        t.position.x = bound.pose.point.enu.x;
        t.position.y = bound.pose.point.enu.y;
        t.rotation = bound.pose.angle.yaw;
        karosserie = world->CreateBox(bound.size.x, bound.size.y, t);
        karosserie->SetCollisionFilter(filter);
    }

    void Karosserie::tick(float dt, muli::Transform t) {

        // Transform the local offset to world coordinates
        muli::Vec2 localOffset = {float(bound.pose.point.enu.x), float(bound.pose.point.enu.y)};
        // Rotate the offset according to car's rotation
        muli::Vec2 rotatedOffset;
        rotatedOffset.x = localOffset.x * t.rotation.c - localOffset.y * t.rotation.s;
        rotatedOffset.y = localOffset.x * t.rotation.s + localOffset.y * t.rotation.c;
        // Add the rotated offset to the car's position
        muli::Vec2 ks_pose;
        ks_pose.x = t.position.x + rotatedOffset.x;
        ks_pose.y = t.position.y + rotatedOffset.y;
        muli::Transform wheelTf{ks_pose, t.rotation};
        karosserie->SetTransform(wheelTf);
    }

    void Karosserie::visualize() {
        auto k_x = karosserie->GetPosition().x;
        auto k_y = karosserie->GetPosition().y;
        auto k_th = karosserie->GetRotation().GetAngle();
        auto k_w = float(bound.size.x * 1.3);
        auto k_h = float(bound.size.y * 1.3);
        std::vector<rerun::Color> colors_a;
        colors_a.push_back(rerun::Color(color.r, color.g, color.b, 40));
        rec->log_static(
            this->name + "/karosserie",
            rerun::Boxes3D::from_centers_and_half_sizes({{k_x, k_y, 0}}, {{k_w / 2, k_h / 2, 0.0f}})
                .with_radii({{0.02f}})
                // .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(k_th))})
                .with_colors(colors_a));
    }
} // namespace mvs
