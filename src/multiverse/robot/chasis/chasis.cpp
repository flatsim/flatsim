#include "multiverse/robot/chasis/chasis.hpp"

namespace mvs {

    Chasis::Chasis(World *world, std::shared_ptr<rerun::RecordingStream> rec, concord::Bound bound,
                   const pigment::RGB &color, std::string name, uint32_t cid, std::vector<concord::Size> wheel_sizes,
                   CollisionFilter filter)
        : world(world), rec(rec), name(name), bound(bound), color(color), group(cid) {
        init(bound, color, name, cid, wheel_sizes, filter);
    }

    void Chasis::init(concord::Bound &bound, const pigment::RGB &color, std::string name, uint32_t group,
                      std::vector<concord::Size> wheel_sizes, CollisionFilter filter) {
        float w = bound.size.x; // usually 0.5
        float h = bound.size.y; // usually 2 * w

        Transform t;
        t.position.x = bound.pose.point.enu.x;
        t.position.y = bound.pose.point.enu.y;
        t.rotation = bound.pose.angle.yaw;

        body = world->CreateBox(w, h, t);
        body->SetCollisionFilter(filter);

        body->SetLinearDamping(linearDamping);
        body->SetAngularDamping(angularDamping);

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
            concord ::Bound wheel_bound;
            wheel_bound.size = wheel_sizes[i];
            concord::Pose wheel_pose;
            wheel_pose.point.enu.x = wheelPosition.x;
            wheel_pose.point.enu.y = wheelPosition.y;
            wheel_pose.angle.yaw = 0.0f; // TODO: fix this
            wheel_bound.pose = wheel_pose;
            // Create the wheel transform
            Transform wheelTf{wheelPosition, t.rotation};
            wheels[i].init(world, rec, color, name + std::to_string(i), wheel_bound, wheelTf, filter, linearDamping,
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

    void Chasis::tick(float dt) {
        for (int i = 0; i < 4; ++i) {
            wheels[i].tick(dt);
        }
    }

    muli::Transform Chasis::get_transform() const { return body->GetTransform(); }

    void Chasis::visualize() {
        auto x = body->GetPosition().x;
        auto y = body->GetPosition().y;
        auto th = body->GetRotation().GetAngle();
        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(color.r, color.g, color.b));
        auto w = float(bound.size.x);
        auto h = float(bound.size.y);
        rec->log_static(
            this->name + "/chassis",
            rerun::Boxes3D::from_centers_and_half_sizes({{x, y, 0}}, {{w / 2, h / 2, 0.0f}})
                .with_radii({{0.02f}})
                .with_labels({this->name})
                // .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));
    }

    void Chasis::update(float steering[4], float throttle[4]) {
        constexpr float MAX_STEER_DEG = 45.0f;
        for (int i = 0; i < 4; ++i) {
            auto steer = std::clamp(steering[i], -MAX_STEER_DEG, MAX_STEER_DEG);
            float angle = DegToRad(steer);
            joints[i]->SetAngularOffset(angle);
            Vec2 f2 = wheels[i].forward * (throttle[i] * wheels[i].force);
            wheels[i].wheel->ApplyForce(wheels[i].wheel->GetPosition(), f2, true);
        }
    }

    void Chasis::teleport(concord::Pose pose) {
        Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw;
        body->SetTransform(t);
        body->SetSleeping(true);

        auto w = float(bound.size.x);
        auto h = float(bound.size.y);

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
