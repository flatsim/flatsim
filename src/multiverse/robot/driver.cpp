// vehicle.cpp
#include "multiverse/robot/driver.hpp"
#include <algorithm>
#include <cmath>
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

    void Wheel::teleport(Transform t) { wheel->SetTransform(t); }

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
            rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{0.1f, 0.2f, 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));
    }

    Vehicle::Vehicle(World *world, std::shared_ptr<rerun::RecordingStream> rec, const concord::Pose &pose,
                     const concord::Size &size, const pigment::RGB &color, std::string name, uint32_t cid)
        : world(world), rec(rec), name(name), size(size), color(color), collision_id(cid) {
        CollisionFilter filter;
        filter.bit = 1 << collision_id;
        filter.mask = ~(1 << collision_id);
        // filter.bit = 1 << 1;
        // filter.mask = ~(1 << 1);

        float w = size.x; // usually 0.5
        float h = size.y; // usually 2 * w

        Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw; // in radians
        body = world->CreateBox(w, h, t);
        body->SetCollisionFilter(filter);

        body->SetLinearDamping(linearDamping);
        body->SetAngularDamping(angularDamping);

        float s = 0.2f;

        std::array<std::pair<Vec2, std::string>, 4> wheelOffsets = {{
            {Vec2(w / 2, h / 2), "fr"},   // front-right
            {Vec2(-w / 2, h / 2), "fl"},  // front-left
            {Vec2(w / 2, -h / 2), "rr"},  // rear-right
            {Vec2(-w / 2, -h / 2), "rl"}, // rear-left
        }};

        for (int i = 0; i < 4; ++i) {
            // Transform the local offset to world coordinates
            Vec2 localOffset = wheelOffsets[i].first;
            // Rotate the offset according to car's rotation
            Vec2 rotatedOffset;
            rotatedOffset.x = localOffset.x * t.rotation.c - localOffset.y * t.rotation.s;
            rotatedOffset.y = localOffset.x * t.rotation.s + localOffset.y * t.rotation.c;
            // Add the rotated offset to the car's position
            Vec2 wheelPosition;
            wheelPosition.x = t.position.x + rotatedOffset.x;
            wheelPosition.y = t.position.y + rotatedOffset.y;
            // Create the wheel transform
            Transform wheelTf{wheelPosition, t.rotation};
            wheels[i].init(world, rec, color, name + wheelOffsets[i].second, s, wheelTf, filter, linearDamping,
                           angularDamping, force, friction, maxImpulse, brake, drag);
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
            wheels[i].step(dt);
        }
    }

    std::vector<float> Vehicle::get_position() const { return {body->GetPosition().x, body->GetPosition().y}; }

    void Vehicle::visualize() {
        auto x = get_position()[0];
        auto y = get_position()[1];
        auto th = body->GetRotation().GetAngle();
        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(color.r, color.g, color.b));
        auto w = float(size.x);
        auto h = float(size.y);
        rec->log_static(
            this->name + "/chassis",
            rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{w / 2, h / 2, 0.0f}})
                .with_radii({{0.02f}})
                .with_labels({this->name})
                // .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));

        // Vec2 size = {this->size[0], this->size[1]};
        Transform t = body->GetTransform();
        const float arrowHeight = size.y * -0.03f; // How far the tip extends beyond the chassis
        const float arrowWidth = size.x * 0.5f;    // Width of arrow base

        std::array<Vec2, 3> arrow_head_offsets = {{
            {Vec2(0, size.y / 2 + arrowHeight)},   // tip of arrow (forward of the chassis)
            {Vec2(-arrowWidth / 2, size.y * 0.3)}, // left corner of arrow
            {Vec2(arrowWidth / 2, size.y * 0.3)}   // right corner of arrow
        }};

        std::array<Vec2, 3> arrow_world_points;
        for (int i = 0; i < 3; ++i) {
            // Get local offset
            Vec2 localOffset = arrow_head_offsets[i];

            // Rotate the offset according to car's rotation
            Vec2 rotatedOffset;
            rotatedOffset.x = localOffset.x * t.rotation.c - localOffset.y * t.rotation.s;
            rotatedOffset.y = localOffset.x * t.rotation.s + localOffset.y * t.rotation.c;

            // Add the rotated offset to the car's position
            arrow_world_points[i].x = t.position.x + rotatedOffset.x;
            arrow_world_points[i].y = t.position.y + rotatedOffset.y;
        }

        const rerun::Position3D vertex_positions[3] = {{arrow_world_points[0].x, arrow_world_points[0].y, 0.0f},
                                                       {arrow_world_points[1].x, arrow_world_points[1].y, 0.0f},
                                                       {arrow_world_points[2].x, arrow_world_points[2].y, 0.0f}};

        pigment::HSV hsv = pigment::HSV::fromRGB(color);
        hsv.adjustBrightness(0.7f);
        auto c = hsv.toRGB();

        const rerun::Color vertex_colors[3] = {
            {static_cast<uint8_t>(c.r), static_cast<uint8_t>(c.g), static_cast<uint8_t>(c.b)},
            {static_cast<uint8_t>(c.r), static_cast<uint8_t>(c.g), static_cast<uint8_t>(c.b)},
            {static_cast<uint8_t>(c.r), static_cast<uint8_t>(c.g), static_cast<uint8_t>(c.b)},
        };
        rec->log_static(this->name + "/heading", rerun::Mesh3D(vertex_positions)
                                                     .with_vertex_normals({{0.0, 0.0, 1.0}})
                                                     .with_vertex_colors(vertex_colors)
                                                     .with_triangle_indices({{2, 1, 0}}));
    }

    void Vehicle::update(float steering[4], float throttle[4]) {
        constexpr float MAX_STEER_DEG = 45.0f;
        for (int i = 0; i < 4; ++i) {
            auto steer = std::clamp(steering[i], -MAX_STEER_DEG, MAX_STEER_DEG);
            float angle = DegToRad(steer);
            joints[i]->SetAngularOffset(angle);
            Vec2 f2 = wheels[i].forward * (throttle[i] * wheels[i].force);
            wheels[i].wheel->ApplyForce(wheels[i].wheel->GetPosition(), f2, true);
        }
    }

    void Vehicle::teleport(concord::Pose pose) {
        Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw;
        body->SetTransform(t);
        body->SetSleeping(true);

        auto w = float(size.x);
        auto h = float(size.y);

        std::array<std::pair<Vec2, std::string>, 4> wheelOffsets = {{
            {Vec2(w / 2, h / 2), "fr"},   // front-right
            {Vec2(-w / 2, h / 2), "fl"},  // front-left
            {Vec2(w / 2, -h / 2), "rr"},  // rear-right
            {Vec2(-w / 2, -h / 2), "rl"}, // rear-left
        }};

        for (int i = 0; i < 4; ++i) {
            // Transform the local offset to world coordinates
            Vec2 localOffset = wheelOffsets[i].first;
            // Rotate the offset according to car's rotation
            Vec2 rotatedOffset;
            rotatedOffset.x = localOffset.x * t.rotation.c - localOffset.y * t.rotation.s;
            rotatedOffset.y = localOffset.x * t.rotation.s + localOffset.y * t.rotation.c;
            // Add the rotated offset to the car's position
            Vec2 wheelPosition;
            wheelPosition.x = t.position.x + rotatedOffset.x;
            wheelPosition.y = t.position.y + rotatedOffset.y;
            // Create the wheel transform
            Transform wheelTf{wheelPosition, t.rotation};
            wheels[i].teleport(wheelTf);
            wheels[i].wheel->SetSleeping(true);
        }
    }

} // namespace mvs
