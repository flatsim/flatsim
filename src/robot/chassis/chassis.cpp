#include "flatsim/robot/chassis/chassis.hpp"

namespace fs {

    float distance(float x1, float y1, float x2, float y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    Chassis::Chassis(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                     muli::CollisionFilter filter)
        : world(world), rec(rec), filter(filter) {}

    void Chassis::init(fs::RobotInfo &robo) {
        this->bound = robo.bound;
        this->color = robo.color;
        this->name = robo.name;

        float w = bound.size.x; // usually 0.5
        float h = bound.size.y; // usually 2 * w
                                //
        muli::Transform t;
        t.position.x = bound.pose.point.x;
        t.position.y = bound.pose.point.y;
        t.rotation = bound.pose.angle.yaw;

        // Create empty body for compound shape
        body = world->CreateEmptyBody(t);
        if (!body) {
            throw InitializationException("Failed to create chassis body");
        }

        // Add main chassis as first collider
        auto chassis_collider = body->CreateBoxCollider(w, h);

        // Set collision filter on the body AND explicitly on each collider
        body->SetCollisionFilter(filter);
        chassis_collider->SetFilter(filter);

        body->SetLinearDamping(fs::constants::linearDamping);
        body->SetAngularDamping(fs::constants::angularDamping);

        // Configure motor joint parameters for soft constraints
        float mf = 300.0f; // Max force (scaled in wheel update)
        float mt = 100.0f; // Max torque
        float fr = 30.0f;  // Frequency for soft joint
        float dr = 1.0f;   // Critical damping
        float jm = body->GetMass();

        for (uint i = 0; i < robo.wheels.size(); ++i) {
            Wheel wheel(world, rec, filter);
            wheel.init(color, name, std::to_string(i), bound, robo.wheels[i], fs::constants::force,
                       fs::constants::friction, fs::constants::maxImpulse, fs::constants::brake, fs::constants::drag,
                       robo.controls.throttles_max[i], robo.controls.steerings_max[i]);
            wheels.push_back(wheel);

            auto joint = world->CreateMotorJoint(body, wheel.get_wheel(), wheel.get_position(), mf, mt, fr, dr, jm);
            joints.emplace_back(joint);

            float mm = std::abs(robo.controls.steerings_max[i] + robo.controls.steerings_diff[i]);
            auto anglejoing = world->CreateLimitedAngleJoint(body, wheel.get_wheel(), -mm, mm);
            angle_joints.emplace_back(anglejoing);
        }

        wheel_damping(fs::constants::linearDamping, fs::constants::angularDamping);

        for (auto const &k : robo.karos) {
            Karosserie karosserie(rec, world);
            karosserie.init(color, name, k.name, bound, k.bound, filter, k.sections, k.has_physics);
            karosseries.push_back(karosserie);

            // Add karosserie as collider to chassis body if it has physics
            if (k.has_physics) {
                // Calculate relative transform of karosserie to chassis
                muli::Transform karos_transform;
                karos_transform.position.x = k.bound.pose.point.x;
                karos_transform.position.y = k.bound.pose.point.y;
                karos_transform.rotation = k.bound.pose.angle.yaw;

                // Add as collider to main chassis body and set filter explicitly
                auto karos_collider = body->CreateBoxCollider(k.bound.size.x, k.bound.size.y, 0.02f, karos_transform);
                karos_collider->SetFilter(filter);
            }
        }

        for (auto const &h : robo.hitches) {
            Hitch hitch(rec, world);
            hitch.init(color, name, h.first, bound, h.second.bound, filter, h.second.is_master);
            hitches.push_back(hitch);
        }
    }

    void Chassis::tick(float dt) {
        pose = utils::transform_to_pose(body->GetTransform());
        for (uint i = 0; i < wheels.size(); ++i) {
            wheels[i].tick(dt);
        }
        for (uint i = 0; i < karosseries.size(); ++i) {
            karosseries[i].tick(dt, pose);
        }
        for (uint i = 0; i < hitches.size(); ++i) {
            hitches[i].tick(dt, pose);
        }
    }

    muli::Transform Chassis::get_transform() const { return body->GetTransform(); }

    void Chassis::tock(const std::string &label) {
        for (uint i = 0; i < hitches.size(); ++i) hitches[i].tock();
        for (uint i = 0; i < wheels.size(); ++i) wheels[i].tock();
        for (uint i = 0; i < karosseries.size(); ++i) karosseries[i].tock();

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
                .with_labels({label})
                // .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));
    }

    void Chassis::update(std::vector<float> steering, std::vector<float> throttle, float dt) {
        for (uint i = 0; i < wheels.size(); ++i) {
            wheels[i].update(steering[i], throttle[i], joints[i], dt);
        }
    }

    void Chassis::wheel_damping(float linear_damping, float angular_damping) {
        for (uint i = 0; i < wheels.size(); ++i) {
            wheels[i].set_linear_damping(linear_damping);
            wheels[i].set_angular_damping(angular_damping);
        }
    }

    void Chassis::toggle_section_work(const std::string &karosserie_name, int section_id) {
        for (uint i = 0; i < karosseries.size(); ++i) {
            if (karosseries[i].name == karosserie_name) {
                karosseries[i].toggle_section_work(section_id);
            }
        }
    }

    void Chassis::toggle_all_sections_work(const std::string &karosserie_name) {
        for (uint i = 0; i < karosseries.size(); ++i) {
            if (karosseries[i].name == karosserie_name) {
                karosseries[i].toggle_all_sections_work();
            }
        }
    }

    void Chassis::toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id) {
        for (uint i = 0; i < karosseries.size(); ++i) {
            if (karosseries[i].name == karosserie_name) {
                karosseries[i].toggle_all_except_section_work(except_section_id);
            }
        }
    }

    void Chassis::teleport(concord::Pose pose) {
        muli::Transform t;
        t.position.x = pose.point.x;
        t.position.y = pose.point.y;
        t.rotation = pose.angle.yaw;
        body->SetTransform(t);
        body->SetSleeping(true);

        for (uint i = 0; i < wheels.size(); ++i) {
            auto nw = utils::move(wheels[i].get_bound().pose, pose);
            wheels[i].teleport(nw);
        }
        for (uint i = 0; i < karosseries.size(); ++i) {
            auto nw = utils::move(karosseries[i].get_bound().pose, pose);
            karosseries[i].teleport(nw);
        }
    }

    void Chassis::update_color(const pigment::RGB &new_color) {
        // Update chassis color
        color = new_color;

        // Update all wheels
        for (auto &wheel : wheels) {
            wheel.update_color(new_color);
        }

        // Update all karosseries
        for (auto &karosserie : karosseries) {
            karosserie.update_color(new_color);
        }

        // Note: hitches don't have color fields so we skip them
    }

} // namespace fs
