#include "multiverse/robot/chasis/wheel.hpp"
#include "pigment/types_hsv.hpp"

namespace mvs {

    Wheel::Wheel(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                 CollisionFilter filter)
        : world(world), rec(rec), filter(filter) {}

    void Wheel::init(World *world, std::shared_ptr<rerun::RecordingStream> rec, const pigment::RGB &color,
                     std::string name, concord::Bound parent, concord::Bound bound, CollisionFilter filter,
                     float linearDamping, float angularDamping, float _force, float _friction, float _maxImpulse,
                     float _brake, float _drag) {
        this->bound = bound;
        this->color = color;
        this->name = name;
        this->rec = rec;

        auto wheelTf = shift(parent, bound);

        wheel = world->CreateCapsule(bound.size.x, bound.size.y, false, wheelTf);
        wheel->SetCollisionFilter(filter);
        wheel->SetLinearDamping(linearDamping);
        wheel->SetAngularDamping(angularDamping);
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
        constexpr float MAX_STEER_DEG = 45.0f;
        steering = std::clamp(steering, -MAX_STEER_DEG, MAX_STEER_DEG);
        float angle = DegToRad(steering);

        if (angle == 0.0f) {
            joint->SetAngularOffset(angle);
        } else {
            joint->SetAngularOffset(0.0f);
            Vec2 f2 = forward * (throttle * force);
            wheel->ApplyForce(wheel->GetPosition(), f2, true);
        }
    }

    muli::Transform Wheel::shift(concord::Bound parent, concord::Bound child) {
        muli::Rotation p_rotation(parent.pose.angle.yaw);
        Vec2 rotatedOffset;
        rotatedOffset.x = bound.pose.point.enu.x * p_rotation.c - bound.pose.point.enu.y * p_rotation.s;
        rotatedOffset.y = bound.pose.point.enu.x * p_rotation.s + bound.pose.point.enu.y * p_rotation.c;
        // Add the rotated offset to the car's position
        Vec2 wheelPosition;
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

    void Wheel::teleport(concord::Pose pose) {
        Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw;
        Vec2 rotatedOffset;
        rotatedOffset.x = bound.pose.point.enu.x * t.rotation.c - bound.pose.point.enu.y * t.rotation.s;
        rotatedOffset.y = bound.pose.point.enu.x * t.rotation.s + bound.pose.point.enu.y * t.rotation.c;
        // Add the rotated offset to the car's position
        Vec2 wheelPosition;
        wheelPosition.x = pose.point.enu.x + rotatedOffset.x;
        wheelPosition.y = pose.point.enu.y + rotatedOffset.y;
        wheel->SetTransform(muli::Transform{wheelPosition, t.rotation});
        wheel->SetSleeping(true);
    }

    void Wheel::visualize() {
        auto x = wheel->GetPosition().x;
        auto y = wheel->GetPosition().y;
        auto th = wheel->GetRotation().GetAngle();

        pigment::HSV h = pigment::HSV::fromRGB(color);
        h.adjustBrightness(0.7f);
        auto c = h.toRGB();

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(c.r, c.g, c.b));

        rec->log_static(
            this->name + "/wheel",
            rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{float(bound.size.x), float(bound.size.y), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));
    }

} // namespace mvs
