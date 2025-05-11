#include "multiverse/robot/chasis/chasis.hpp"

namespace mvs {

    float distance(float x1, float y1, float x2, float y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    Chasis::Chasis(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                   muli::CollisionFilter filter)
        : world(world), rec(rec), filter(filter) {}

    void Chasis::init(concord::Bound &bound, const pigment::RGB &color, std::string name,
                      std::vector<concord::Bound> wheels_s, std::vector<concord::Bound> karosseries) {

        this->bound = bound;
        this->color = color;
        this->name = name;

        float w = bound.size.x; // usually 0.5
        float h = bound.size.y; // usually 2 * w
                                //
        Transform t;
        t.position.x = bound.pose.point.enu.x;
        t.position.y = bound.pose.point.enu.y;
        t.rotation = bound.pose.angle.yaw;

        body = world->CreateBox(w, h, t);
        body->SetCollisionFilter(filter);

        body->SetLinearDamping(linearDamping);
        body->SetAngularDamping(angularDamping);

        float mf = -1;
        float fr = -1;
        float dr = 0.1f;
        float jm = body->GetMass();

        for (uint i = 0; i < wheels_s.size(); ++i) {
            Wheel wheel(world, rec, filter);
            wheel.init(color, name + std::to_string(i), bound, wheels_s[i], linearDamping, angularDamping, force,
                       friction, maxImpulse, brake, drag);
            wheelz.push_back(wheel);

            auto joint = world->CreateMotorJoint(body, wheel.wheel, wheel.wheel->GetPosition(), mf, torque, fr, dr, jm);
            jointz.emplace_back(joint);
        }
        //
        for (uint i = 0; i < karosseries.size(); ++i) {
            Karosserie karosserie(rec, world);
            karosserie.init(bound, karosseries[i], filter, color, name + std::to_string(i));
            karosseriez.push_back(karosserie);
            // auto left_hook = Vec2(karosseriez[i].get_transform().position.x - karosseries[i].size.x / 2,
            //                       karosseriez[i].get_transform().position.y);
            // auto right_hook = Vec2(karosseriez[i].get_transform().position.x + karosseries[i].size.x / 2,
            //                        karosseriez[i].get_transform().position.y);
            // auto dist_left = distance(body->GetPosition().x, body->GetPosition().y, left_hook.x, left_hook.y);
            // auto dist_right = distance(body->GetPosition().x, body->GetPosition().y, right_hook.x, right_hook.y);
            // world->CreateLimitedDistanceJoint(body, karosseriez[i].get_body(),
            //                                   Vec2(body->GetPosition().x, body->GetPosition().y), left_hook,
            //                                   dist_left);
            // world->CreateLimitedAngleJoint(body, karosseriez[i].get_body(), 0.1f, 0.1f);
        }
    }

    void Chasis::tick(float dt) {
        for (uint i = 0; i < wheelz.size(); ++i) {
            wheelz[i].tick(dt);
        }
        for (uint i = 0; i < karosseriez.size(); ++i) {
            karosseriez[i].tick(dt, body->GetTransform());
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
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{w, h, 0.0f}})
                .with_radii({{0.02f}})
                .with_labels({this->name})
                // .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));

        for (uint i = 0; i < wheelz.size(); ++i) {
            wheelz[i].visualize();
        }
        for (uint i = 0; i < karosseriez.size(); ++i) {
            karosseriez[i].visualize();
        }
    }

    void Chasis::update(std::vector<float> steering, std::vector<float> throttle) {
        for (uint i = 0; i < wheelz.size(); ++i) {
            jointz[i]->SetAngularOffset(steering[i]);
            Vec2 f2 = wheelz[i].forward * (throttle[i] * wheelz[i].force);
            wheelz[i].wheel->ApplyForce(wheelz[i].wheel->GetPosition(), f2, true);
        }
    }

    void Chasis::teleport(concord::Pose pose) {
        Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw;
        body->SetTransform(t);
        body->SetSleeping(true);

        for (uint i = 0; i < wheelz.size(); ++i) {
            wheelz[i].teleport(pose);
        }
    }

} // namespace mvs
