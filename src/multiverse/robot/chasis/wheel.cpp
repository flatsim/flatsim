#include "multiverse/robot/chasis/wheel.hpp"
#include "pigment/types_hsv.hpp"

namespace mvs {

    Wheel::Wheel(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                 CollisionFilter filter)
        : world(world), rec(rec), filter(filter) {}

    void Wheel::init(const pigment::RGB &color, std::string parent_name, std::string name, concord::Bound parent_bound,
                     concord::Bound bound, float _force, float _friction, float _maxImpulse, float _brake, float _drag,
                     float throttle_max, float steering_max) {
        this->bound = bound;
        this->color = color;
        this->name = name;
        this->parent_name = parent_name;
        this->steering_max = steering_max;
        this->throttle_max = throttle_max;

        pose = utils::shift(parent_bound.pose, bound.pose);
        auto wheelTf = utils::pose_to_transform(pose);

        wheel = world->CreateCapsule(bound.size.y, bound.size.x, false, wheelTf);
        wheel->SetCollisionFilter(filter);
        force = _force;
        friction = _friction;
        maxImpulse = _maxImpulse;
        brake = _brake;
        drag = _drag;
    }

    void Wheel::tick(float dt) {
        const Vec2 up(0, 1);
        const Vec2 right(1, 0);

        forward = Mul(wheel->GetRotation(), up);
        normal = Mul(wheel->GetRotation(), right);

        Vec2 v = wheel->GetLinearVelocity();
        float vf = Dot(v, forward);
        float vn = Dot(v, normal);

        if (Abs(vn) > epsilon) {
            Vec2 j = -wheel->GetMass() * friction * vn * normal;
            if (Length(j) > maxImpulse) {
                j = Normalize(j) * maxImpulse;
            }
            wheel->ApplyLinearImpulse(wheel->GetPosition(), j, true);
        }

        float av = wheel->GetAngularVelocity();
        if (Abs(av) > epsilon) {
            wheel->ApplyAngularImpulse(0.1f * wheel->GetInertia() * -av, true);
        }

        if (Abs(vf) > epsilon) {
            float dragForceMagnitude = -drag * vf;
            wheel->ApplyForce(wheel->GetPosition(), dragForceMagnitude * forward, true);
        }

        visualize();
    }

    void Wheel::update(float steering, float throttle, MotorJoint *joint) {
        throttle_val = throttle;
        steering_val = steering;
        if (steering == 0.0f) {
            joint->SetAngularOffset(steering);
        } else {
            joint->SetAngularOffset(0.0f);
            Vec2 f2 = forward * (throttle * force);
            wheel->ApplyForce(wheel->GetPosition(), f2, true);
        }
    }

    void Wheel::teleport(concord::Pose trans_pose) {
        wheel->SetTransform(utils::pose_to_transform(trans_pose));
        wheel->SetSleeping(true);
    }

    void Wheel::visualize() {
        auto x = wheel->GetPosition().x;
        auto y = wheel->GetPosition().y;
        auto th = wheel->GetRotation().GetAngle();

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(color.r, color.g, color.b));

        rec->log_static(
            this->parent_name + "/chasis/wheel/" + this->name,
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{float(bound.size.x), float(bound.size.y), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));
    }

} // namespace mvs
