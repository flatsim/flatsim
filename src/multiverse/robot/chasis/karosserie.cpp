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
