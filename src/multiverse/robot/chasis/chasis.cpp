#include "multiverse/robot/chasis/chasis.hpp"

namespace mvs {

    float distance(float x1, float y1, float x2, float y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    Chasis::Chasis(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                   muli::CollisionFilter filter)
        : world(world), rec(rec), filter(filter) {}

    void Chasis::init(mvs::RobotInfo &robo) {
        this->bound = robo.bound;
        this->color = robo.color;
        this->name = robo.name;

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

        for (uint i = 0; i < robo.wheels.size(); ++i) {
            Wheel wheel(world, rec, filter);
            wheel.init(color, name, std::to_string(i), bound, robo.wheels[i], force, friction, maxImpulse, brake, drag);
            wheelz.push_back(wheel);

            auto joint = world->CreateMotorJoint(body, wheel.wheel, wheel.wheel->GetPosition(), mf, torque, fr, dr, jm);
            jointz.emplace_back(joint);

            float mm = std::abs(robo.controlz.steerings_max[i] + robo.controlz.steerings_diff[i]);
            auto anglejoing = world->CreateLimitedAngleJoint(body, wheel.wheel, -mm, mm);
            anglejointz.emplace_back(anglejoing);
        }

        wheel_damping(linearDamping, angularDamping);

        auto main_left_hook = Vec2(bound.pose.point.enu.x - bound.size.x / 2, bound.pose.point.enu.y);
        auto main_right_hook = Vec2(bound.pose.point.enu.x + bound.size.x / 2, bound.pose.point.enu.y);

        for (auto const &k : robo.karos) {
            Karosserie karosserie(rec, world);
            karosserie.init(color, name, k.name, bound, k.bound, filter, k.has_physics);
            karosseriez.push_back(karosserie);
        }

        for (auto const &h : robo.hitches) {
            Hitch hitch(rec, world);
            hitch.init(color, name, h.first, bound, h.second, filter);
            hitchz.push_back(hitch);
        }
    }

    void Chasis::tick(float dt) {
        pose = utils::transform_to_pose(body->GetTransform());
        for (uint i = 0; i < wheelz.size(); ++i) {
            wheelz[i].tick(dt);
        }
        for (uint i = 0; i < karosseriez.size(); ++i) {
            karosseriez[i].tick(dt, pose);
        }
        for (uint i = 0; i < hitchz.size(); ++i) {
            hitchz[i].tick(dt, pose);
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
    }

    void Chasis::update(std::vector<float> steering, std::vector<float> throttle) {
        for (uint i = 0; i < wheelz.size(); ++i) {
            jointz[i]->SetAngularOffset(steering[i]);
            Vec2 f2 = wheelz[i].forward * (throttle[i] * wheelz[i].force);
            wheelz[i].wheel->ApplyForce(wheelz[i].wheel->GetPosition(), f2, true);
        }
    }

    void Chasis::wheel_damping(float linearDamping, float angularDamping) {
        for (uint i = 0; i < wheelz.size(); ++i) {
            wheelz[i].wheel->SetLinearDamping(linearDamping);
            wheelz[i].wheel->SetAngularDamping(angularDamping);
        }
    }

    void Chasis::toggle_work(std::string karosserie_name) {
        for (uint i = 0; i < karosseriez.size(); ++i) {
            if (karosseriez[i].name == karosserie_name) {
                karosseriez[i].toggle_work();
            }
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
            auto nw = utils::move(wheelz[i].get_bound().pose, pose);
            wheelz[i].teleport(nw);
        }
        for (uint i = 0; i < karosseriez.size(); ++i) {
            auto nw = utils::move(karosseriez[i].get_bound().pose, pose);
            karosseriez[i].teleport(nw);
        }
    }

} // namespace mvs
