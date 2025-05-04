// vehicle.cpp
#include "multiverse/robot/driver.hpp"
#include <algorithm>
#include <iostream>

namespace mvs {
    void Wheel::init(World *world, std::shared_ptr<rerun::RecordingStream> rec, const pigment::RGB &color,
                     std::string name, float scale, Transform tf, CollisionFilter filter, float linearDamping,
                     float angularDamping, float _force, float _friction, float _maxImpulse, float _brake,
                     float _drag) {
        this->color = color;
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
        auto t = wheel->GetRotation().GetAngle();

        pigment::HSV h = pigment::HSV::fromRGB(color);
        h.adjustBrightness(0.7f);
        auto c = h.toRGB();

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(c.r, c.g, c.b));

        rec->log_static(this->name + "/wheel",
                        rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{0.1f, 0.2f, 0.0f}})
                            .with_radii({{0.02f}})
                            // .with_fill_mode(rerun::FillMode::Solid)
                            .with_rotation_axis_angles({rerun::RotationAxisAngle(
                                {0.0f, 0.0f, 1.0f}, rerun::Angle::radians(static_cast<float>(t) * 2.0f))})
                            .with_colors(colors));
    }

    Vehicle::Vehicle(World *world, std::shared_ptr<rerun::RecordingStream> rec, const concord::Pose &pose,
                     const concord::Size &size, const pigment::RGB &color, std::string name)
        : world(world), rec(rec), name(name), size({float(size.x), float(size.y)}), color(color) {
        CollisionFilter filter;
        filter.bit = 1 << 1;
        filter.mask = ~(1 << 1);

        float w = size.x;
        float h = size.y;

        Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw;
        body = world->CreateBox(w, h, t);
        body->SetCollisionFilter(filter);
        body->SetLinearDamping(linearDamping);
        body->SetAngularDamping(angularDamping);

        float s = 0.2f;
        std::array<std::pair<Vec2, std::string>, 4> wheelOffsets = {{
            {Vec2(+w / 2, +h / 2), "fr"}, // front-right
            {Vec2(-w / 2, +h / 2), "fl"}, // front-left
            {Vec2(+w / 2, -h / 2), "rr"}, // rear-right
            {Vec2(-w / 2, -h / 2), "rl"}, // rear-left
        }};

        // Motor joints things
        float mf = -1;
        float fr = -1;
        float dr = 0.1f;
        float jm = body->GetMass();

        for (int i = 0; i < 4; ++i) {
            Vec2 lo = wheelOffsets[i].first;
            Vec2 worldOffset{lo.x * t.rotation.c - lo.y * t.rotation.s, lo.x * t.rotation.s + lo.y * t.rotation.c};
            Vec2 wheelPos{float(pose.point.enu.x) + worldOffset.x, float(pose.point.enu.y) + worldOffset.y};

            Transform wheelTf{wheelPos, t.rotation};

            wheels[i].init(world, rec, color, name + wheelOffsets[i].second, s, wheelTf, filter, linearDamping,
                           angularDamping, force, friction, maxImpulse, brake, drag);
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
        auto t = body->GetRotation().GetAngle();
        std::cout << "yaw of robot: " << name << " is " << body->GetRotation().GetAngle() << "\n";
        // exit(0);

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(color.r, color.g, color.b));

        rec->log_static(this->name + "/chassis",
                        rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{size[0] / 2, size[1] / 2, 0.0f}})
                            .with_radii({{0.02f}})
                            .with_labels({this->name})
                            .with_fill_mode(rerun::FillMode::Solid)
                            .with_rotation_axis_angles({rerun::RotationAxisAngle(
                                {0.0f, 0.0f, 1.0f}, rerun::Angle::radians(static_cast<float>(t) * 2.0f))})
                            .with_colors(colors));
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
