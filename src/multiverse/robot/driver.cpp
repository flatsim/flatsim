// vehicle.cpp
#include "multiverse/robot/driver.hpp"
#include <iostream> // for debug output

namespace mvs {
    void Wheel::init(World *world, float scale, Transform tf, CollisionFilter filter, float linearDamping,
                     float angularDamping, float _force, float _friction, float _maxImpulse, float _brake,
                     float _drag) {
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

    void Wheel::step(float dt, std::shared_ptr<rerun::RecordingStream> rec) {
        const Vec2 up(0, 1), right(1, 0);
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
    }

    Vehicle::Vehicle(World *world, std::shared_ptr<rerun::RecordingStream> rec, const concord::Pose &pose)
        : world(world), rec(rec) {
        CollisionFilter filter;
        filter.bit = 1 << 1;
        filter.mask = ~(1 << 1);

        float w = 0.8f;
        float h = 1.4f;

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
        wheels[0].init(world, s, Transform(Vec2(p.x + w / 2, p.y + h / 2)), filter, linearDamping, angularDamping,
                       force, friction, maxImpulse, brake, drag);
        wheels[1].init(world, s, Transform(Vec2(p.x - w / 2, p.y + h / 2)), filter, linearDamping, angularDamping,
                       force, friction, maxImpulse, brake, drag);

        // Rear wheels
        wheels[2].init(world, s, Transform(Vec2(p.x + w / 2, p.y - h / 2)), filter, linearDamping, angularDamping,
                       force, friction, maxImpulse, brake, drag);
        wheels[3].init(world, s, Transform(Vec2(p.x - w / 2, p.y - h / 2)), filter, linearDamping, angularDamping,
                       force, friction, maxImpulse, brake, drag);

        for (int i = 0; i < 4; ++i) {
            std::cout << "wheel raw pointer: " << wheels[i].wheel << std::endl;
        }

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
            wheels[i].step(dt, rec);
        }
    }

    void Vehicle::update(float steering, float throttle) {
        float angle = DegToRad(steering);
        joints[0]->SetAngularOffset(angle);
        joints[1]->SetAngularOffset(angle);
        joints[2]->SetAngularOffset(0.0f);
        joints[3]->SetAngularOffset(0.0f);

        // rear-wheel torque (if you re-enable it)
        // for (int i = 2; i < 4; ++i) {
        //     Vec2 f = wheels[i].forward * (throttle * 0.1f);
        //     wheels[i].wheel->ApplyForce(wheels[i].wheel->GetPosition(), f, true);
        // }
    }

    void Vehicle::visualize_once() {}

    void Vehicle::visualize() {}
} // namespace mvs
