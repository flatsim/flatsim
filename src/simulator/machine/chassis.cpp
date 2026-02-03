#include "flatsim/simulator/machine/chassis.hpp"
#include "flatsim/utils.hpp"

#include <echo/echo.hpp>

namespace simulator {

    float distance(float x1, float y1, float x2, float y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    Chassis::Chassis(std::shared_ptr<flywheel::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                     flywheel::CollisionFilter filter, types::Machine *robot_info, types::State *robot_state)
        : world(world), rec(rec), filter(filter), robot_info(robot_info), robot_state(robot_state) {}

    void Chassis::init(types::Machine &robo) {
        this->bound = robo.bound;
        this->color = robo.color;

        echo::trace("[Chassis] init uuid=", robo.uuid, " pose x=", bound.pose.point.x, " y=", bound.pose.point.y,
                    " yaw=", utils::get_yaw(bound.pose));

        float w = bound.size.x; // usually 0.5
        float h = bound.size.y; // usually 2 * w
                                //
        flywheel::Transform t;
        t.position.x = bound.pose.point.x;
        t.position.y = bound.pose.point.y;
        t.rotation = utils::get_yaw(bound.pose);

        // Create empty body for compound shape
        body = world->CreateEmptyBody(t);
        if (!body) {
            throw std::runtime_error("Failed to create chassis body");
        }

        // Add main chassis as first collider
        auto chassis_collider = body->CreateBoxCollider(w, h);

        // Set collision filter on the body AND explicitly on each collider
        body->SetCollisionFilter(filter);
        chassis_collider->SetFilter(filter);

        static const types::Physics physics; // default values
        body->SetLinearDamping(physics.linear_damping);
        body->SetAngularDamping(physics.angular_damping);

        // Configure motor joint parameters for soft constraints
        float mf = 300.0f; // Max force (scaled in wheel update)
        float mt = 100.0f; // Max torque
        float fr = 30.0f;  // Frequency for soft joint
        float dr = 1.0f;   // Critical damping
        float jm = body->GetMass();

        for (uint i = 0; i < robo.wheels.size(); ++i) {
            // Use wheel's own values if controls vectors are empty/insufficient
            float throttle_max =
                (i < robo.controls.throttles_max.size()) ? robo.controls.throttles_max[i] : robo.wheels[i].throttle_max;
            float steering_max =
                (i < robo.controls.steerings_max.size()) ? robo.controls.steerings_max[i] : robo.wheels[i].steering_max;
            float steering_diff = (i < robo.controls.steerings_diff.size()) ? robo.controls.steerings_diff[i] : 0.0f;

            Wheel wheel(world, rec, filter, robot_info, robot_state);
            wheel.init(color, name, std::to_string(i), bound, robo.wheels[i].bound, physics.force, physics.friction,
                       physics.max_impulse, physics.brake, physics.drag, throttle_max, steering_max);
            wheels.push_back(wheel);

            auto joint = world->CreateMotorJoint(body, wheel.get_wheel(), wheel.get_position(), mf, mt, fr, dr, jm);
            joints.emplace_back(joint);

            float mm = std::abs(steering_max + steering_diff);
            auto anglejoing = world->CreateLimitedAngleJoint(body, wheel.get_wheel(), -mm, mm);
            angle_joints.emplace_back(anglejoing);
        }

        wheel_damping(physics.linear_damping, physics.angular_damping);

        for (auto const &k : robo.karosseries) {
            Karosserie karosserie(rec, world, robot_info, robot_state);
            karosserie.init(color, name, k.name, bound, k.bound, filter, k.sections.size(), k.has_physics);
            karosseries.push_back(karosserie);

            // Add karosserie as collider to chassis body if it has physics
            if (k.has_physics) {
                // Calculate relative transform of karosserie to chassis
                flywheel::Transform karos_transform;
                karos_transform.position.x = k.bound.pose.point.x;
                karos_transform.position.y = k.bound.pose.point.y;
                karos_transform.rotation = utils::get_yaw(k.bound.pose);

                // Add as collider to main chassis body and set filter explicitly
                auto karos_collider = body->CreateBoxCollider(k.bound.size.x, k.bound.size.y, 0.02f, karos_transform);
                karos_collider->SetFilter(filter);
            }
        }

        for (auto const &h : robo.hitches) {
            Hitch hitch(rec, world, robot_info, robot_state);
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

    flywheel::Transform Chassis::get_transform() const { return body->GetTransform(); }

    void Chassis::tock(const std::string &label) {
        static int dbg_tock = 0;
        if (dbg_tock < 10) {
            const bool has_body = (body != nullptr);
            const double x = has_body ? body->GetPosition().x : 0.0;
            const double y = has_body ? body->GetPosition().y : 0.0;
            echo::trace("[Chassis] tock uuid=", robot_info->uuid, " online=", robot_state->online,
                        " rec=", (rec != nullptr), " x=", x, " y=", y, " w=", bound.size.x, " h=", bound.size.y);
            dbg_tock++;
        }

        if (!robot_state->online) return;
        if (!rec) return;

        for (uint i = 0; i < hitches.size(); ++i) hitches[i].tock();
        for (uint i = 0; i < wheels.size(); ++i) wheels[i].tock();
        for (uint i = 0; i < karosseries.size(); ++i) karosseries[i].tock();

        auto x = body->GetPosition().x;
        auto y = body->GetPosition().y;
        auto th = body->GetRotation().GetAngle();
        auto w = float(bound.size.x);
        auto h = float(bound.size.y);
        const float z = 0.3f;
        rec->log_static(
            robot_info->uuid + "/chassis",
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{w, h, z}})
                .with_radii({{0.02f}})
                // .with_labels({label})
                // .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors({rerun::Color(color.r(), color.g(), color.b())}));
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

    void Chassis::teleport(datapod::Pose pose) {
        flywheel::Transform t;
        t.position.x = pose.point.x;
        t.position.y = pose.point.y;
        t.rotation = utils::get_yaw(pose);
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

    void Chassis::brake(float brake_force) {
        // Apply braking to all wheels
        for (auto &wheel : wheels) {
            wheel.apply_brake(brake_force);
        }

        // Also apply direct braking to the chassis body for immediate effect
        if (body) {
            flywheel::Vec2 v = body->GetLinearVelocity();
            float speed = flywheel::Length(v);

            if (speed > flywheel::epsilon) {
                // Apply impulse opposite to velocity
                flywheel::Vec2 brake_impulse = -flywheel::Normalize(v) * brake_force * body->GetMass() * 0.5f;

                // Clamp to not exceed current momentum
                float max_impulse = body->GetMass() * speed;
                if (flywheel::Length(brake_impulse) > max_impulse) {
                    brake_impulse = flywheel::Normalize(brake_impulse) * max_impulse;
                }

                body->ApplyLinearImpulse(body->GetPosition(), brake_impulse, true);
            }

            // Also brake angular velocity
            float angular_vel = body->GetAngularVelocity();
            if (flywheel::Abs(angular_vel) > flywheel::epsilon) {
                float angular_brake = -angular_vel * brake_force * 0.5f;
                body->ApplyTorque(angular_brake, true);
            }
        }
    }

    void Chassis::destroy() {
        // Destroy joints first
        for (auto *joint : joints) {
            if (joint) world->Destroy(joint);
        }
        joints.clear();

        for (auto *joint : angle_joints) {
            if (joint) world->Destroy(joint);
        }
        angle_joints.clear();

        // Destroy wheels
        for (auto &wheel : wheels) {
            wheel.destroy();
        }

        // Destroy karosseries
        for (auto &kaross : karosseries) {
            kaross.destroy();
        }

        // Destroy body last
        if (world && body) {
            world->Destroy(body);
            body = nullptr;
        }
    }

} // namespace simulator
