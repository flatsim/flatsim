// vehicle.cpp
#include "multiverse/robot/driver.hpp"
#include <algorithm>
#include <iostream>

namespace mvs {

    std::vector<float> quaterion_from_angle(float angle) {
        std::vector<float> quat;
        quat.push_back(cos(angle / 2));
        quat.push_back(0);
        quat.push_back(0);
        quat.push_back(sin(angle / 2));
        return quat;
    }

    void Wheel::init(World *world, std::shared_ptr<rerun::RecordingStream> rec, std::string name, float scale,
                     Transform tf, CollisionFilter filter, float linearDamping, float angularDamping, float _force,
                     float _friction, float _maxImpulse, float _brake, float _drag) {
        this->name = name;
        this->rec = rec;
        wheel = world->CreateCapsule(scale, scale, false, tf);
        wheel->SetCollisionFilter(filter);
        wheel->SetLinearDamping(linearDamping);
        wheel->SetAngularDamping(angularDamping);
        force = _force;
        friction = _friction;
        maxImpulse = _maxImpulse;
        brake = _brake;
        drag = _drag;
    }
    void Wheel::step(float dt) {
        const Vec2 up(0, 1), right(1, 0);
        forward = Mul(wheel->GetRotation(), up);
        normal = Mul(wheel->GetRotation(), right);

        // current velocity in wheel-space
        Vec2 v = wheel->GetLinearVelocity();
        float vf = Dot(v, forward);
        float vn = Dot(v, normal);

        // --- lateral (sideways) friction impulse ---
        if (Abs(vn) > epsilon) {
            // Compute frictional *force*  F = µ * m * vn
            Vec2 Ff = -friction * wheel->GetMass() * vn * normal;
            // Convert to impulse over this timestep: J = F * dt
            Vec2 J = Ff * dt;

            // clamp to maxImpulse
            if (Length(J) > maxImpulse) {
                J = Normalize(J) * maxImpulse;
            }
            wheel->ApplyLinearImpulse(wheel->GetPosition(), J, true);
        }

        // --- angular damping impulse ---
        float av = wheel->GetAngularVelocity();
        if (Abs(av) > epsilon) {
            // torque to oppose spin: τ = -c * I * av
            float Tau = -0.1f * wheel->GetInertia() * av;
            float Jrot = Tau * dt; // angular impulse = torque * dt
            wheel->ApplyAngularImpulse(Jrot, true);
        }

        // --- longitudinal drag force (continual) ---
        if (Abs(vf) > epsilon) {
            // drag force along forward vector
            float Fd = -drag * vf;
            wheel->ApplyForce(wheel->GetPosition(), Fd * forward, true);
        }
        visualize();
    }

    void Wheel::visualize() {
        auto x = wheel->GetPosition().x;
        auto y = wheel->GetPosition().y;
        auto t = quaterion_from_angle(wheel->GetRotation().GetAngle());
        rec->log_static(
            this->name + "/wheel",
            rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{0.1f, 0.2f, 0.0f}})
                .with_quaternions({rerun::Quaternion::IDENTITY, rerun::Quaternion::from_xyzw(t[0], t[1], t[2], t[3])})
                .with_radii({{0.02f}}));
    }

    Vehicle::Vehicle(World *world, std::shared_ptr<rerun::RecordingStream> rec, const concord::Pose &pose,
                     const concord::Size &size, std::string name)
        : world(world), rec(rec), name(name), size({float(size.x), float(size.y)}) {
        CollisionFilter filter;
        filter.bit = 1 << 1;
        filter.mask = ~(1 << 1);

        float w = size.x;
        float h = size.y;

        body = world->CreateBox(w, h);
        body->SetCollisionFilter(filter);

        Vec2 p;
        p.x = pose.point.enu.x;
        p.y = pose.point.enu.y;
        body->SetPosition(p);

        body->SetLinearDamping(linearDamping);
        body->SetAngularDamping(angularDamping);

        float s = 0.2f;

        // Front wheels
        wheels[0].init(world, rec, name + "fr", s, Transform(Vec2(p.x + w / 2, p.y + h / 2)), filter, linearDamping,
                       angularDamping, force, friction, maxImpulse, brake, drag);
        wheels[1].init(world, rec, name + "fl", s, Transform(Vec2(p.x - w / 2, p.y + h / 2)), filter, linearDamping,
                       angularDamping, force, friction, maxImpulse, brake, drag);
        // Rear wheels
        wheels[2].init(world, rec, name + "rr", s, Transform(Vec2(p.x + w / 2, p.y - h / 2)), filter, linearDamping,
                       angularDamping, force, friction, maxImpulse, brake, drag);
        wheels[3].init(world, rec, name + "rl", s, Transform(Vec2(p.x - w / 2, p.y - h / 2)), filter, linearDamping,
                       angularDamping, force, friction, maxImpulse, brake, drag);

        float mf = -1;
        float fr = -1;
        float dr = 0.1f;
        float jm = body->GetMass();

        for (int i = 0; i < 4; ++i) {
            joints[i] =
                world->CreateMotorJoint(body, wheels[i].wheel, wheels[i].wheel->GetPosition(), mf, torque, fr, dr, jm);
        }
    }

    void Vehicle::tick(float dt) {
        for (int i = 0; i < 4; ++i) {
            wheels[i].step(dt);
        }
    }

    std::vector<float> Vehicle::get_position() const { return {body->GetPosition().x, body->GetPosition().y}; }

    void Vehicle::visualize() {
        auto x = get_position()[0];
        auto y = get_position()[1];
        auto t = quaterion_from_angle(body->GetRotation().GetAngle());
        rec->log_static(
            this->name + "/chassis",
            rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{size[0] / 2, size[1] / 2, 0.0f}})
                .with_quaternions({rerun::Quaternion::IDENTITY, rerun::Quaternion::from_xyzw(t[0], t[1], t[2], t[3])})
                .with_radii({{0.02f}}));
    }

    void Vehicle::update(float steering, float throttle) {
        constexpr float MAX_STEER_DEG = 45.0f;
        steering = std::clamp(steering, -MAX_STEER_DEG, MAX_STEER_DEG);
        float angle = DegToRad(steering);
        joints[0]->SetAngularOffset(angle);
        joints[1]->SetAngularOffset(angle);
        joints[2]->SetAngularOffset(0.0f);
        joints[3]->SetAngularOffset(0.0f);
        for (int i = 2; i < 4; ++i) {
            Vec2 f = wheels[i].forward * (throttle * wheels[i].force);
            wheels[i].wheel->ApplyForce(wheels[i].wheel->GetPosition(), f, true);
        }
    }
} // namespace mvs
