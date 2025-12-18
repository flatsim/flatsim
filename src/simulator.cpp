#include "flatsim/simulator.hpp"
#include <cmath>
#include <iostream>
#include <vector>

namespace simulator {

    // Physics constants
    constexpr float LINEAR_DAMPING = 0.3f;
    constexpr float ANGULAR_DAMPING = 0.5f;
    constexpr float MOTOR_MAX_FORCE = 300.0f;
    constexpr float MOTOR_MAX_TORQUE = 100.0f;
    constexpr float MOTOR_FREQUENCY = 30.0f;
    constexpr float MOTOR_DAMPING_RATIO = 1.0f;

    concord::Pose Simulator::shift_pose(const concord::Pose &parent, const concord::Pose &child) {
        float cos_a = std::cos(static_cast<float>(parent.angle.yaw));
        float sin_a = std::sin(static_cast<float>(parent.angle.yaw));

        concord::Pose result;
        result.point.x = parent.point.x + child.point.x * cos_a - child.point.y * sin_a;
        result.point.y = parent.point.y + child.point.x * sin_a + child.point.y * cos_a;
        result.angle.yaw = parent.angle.yaw + child.angle.yaw;
        return result;
    }

    Simulator::Simulator(Conn conn, const std::string &address, const WorldSettings &settings)
        : ctx_(1), conn_(conn), address_(address), world_settings_(settings) {

        // Setup ZMQ
        socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::rep);

        if (conn_ == Conn::IPC) {
            std::string addr = address_.empty() ? "ipc:///tmp/flatsim" : address_;
            socket_->bind(addr);
            std::cout << "[Simulator] Listening on " << addr << std::endl;
        } else {
            std::string addr = address_.empty() ? "tcp://*:5555" : address_;
            socket_->bind(addr);
            std::cout << "[Simulator] Listening on " << addr << std::endl;
        }
        socket_->set(zmq::sockopt::rcvtimeo, 0);

        // Setup physics world (no gravity for top-down 2D)
        muli::WorldSettings muli_settings;
        muli_settings.world_bounds =
            muli::AABB(muli::Vec2(-world_settings_.width / 2.0f, -world_settings_.height / 2.0f),
                       muli::Vec2(world_settings_.width / 2.0f, world_settings_.height / 2.0f));
        muli_settings.apply_gravity = false;
        muli_settings.gravity = muli::Vec2(0.0f, 0.0f);

        world_ = std::make_unique<muli::World>(muli_settings);
        std::cout << "[Simulator] Physics world created (" << world_settings_.width << "x" << world_settings_.height
                  << ")" << std::endl;
    }

    Simulator::~Simulator() {
        socket_->close();
        ctx_.close();
    }

    void Simulator::create_machine(const types::Machine &machine) {
        MachinePhysics mp;
        mp.config = machine;

        // Create collision filter (negative group = never collide with same group)
        uint32_t group = machine.group > 0 ? machine.group : next_group_++;
        mp.filter.group = -static_cast<int>(group);
        mp.filter.bit = 1;
        mp.filter.mask = 0xFFFFFFFF;

        // Create machine body transform
        muli::Transform machine_tf;
        machine_tf.position.x = static_cast<float>(machine.pose.point.x);
        machine_tf.position.y = static_cast<float>(machine.pose.point.y);
        machine_tf.rotation = static_cast<float>(machine.pose.angle.yaw);

        // Create empty body for compound shape
        mp.body = world_->CreateEmptyBody(machine_tf);
        if (!mp.body) {
            std::cerr << "[Simulator] Failed to create body for: " << machine.name << std::endl;
            return;
        }

        // Add main machine collider
        auto *machine_collider =
            mp.body->CreateBoxCollider(static_cast<float>(machine.size.x), static_cast<float>(machine.size.y));
        mp.body->SetCollisionFilter(mp.filter);
        machine_collider->SetFilter(mp.filter);

        mp.body->SetLinearDamping(LINEAR_DAMPING);
        mp.body->SetAngularDamping(ANGULAR_DAMPING);

        float body_mass = mp.body->GetMass();

        // Create wheels
        for (const auto &wheel_cfg : machine.wheels) {
            WheelPhysics wp;
            wp.config = wheel_cfg;

            // Calculate wheel world position
            concord::Pose wheel_pose = shift_pose(machine.pose, wheel_cfg.pose);
            muli::Transform wheel_tf;
            wheel_tf.position.x = static_cast<float>(wheel_pose.point.x);
            wheel_tf.position.y = static_cast<float>(wheel_pose.point.y);
            wheel_tf.rotation = static_cast<float>(wheel_pose.angle.yaw);

            // Create wheel body
            wp.body =
                world_->CreateBox(static_cast<float>(wheel_cfg.size.x), static_cast<float>(wheel_cfg.size.y), wheel_tf);
            if (!wp.body) {
                std::cerr << "[Simulator] Failed to create wheel: " << wheel_cfg.name << std::endl;
                continue;
            }

            wp.body->SetCollisionFilter(mp.filter);
            wp.body->SetLinearDamping(LINEAR_DAMPING);
            wp.body->SetAngularDamping(ANGULAR_DAMPING);

            // Create motor joint connecting wheel to machine
            muli::Vec2 wheel_pos(wheel_tf.position.x, wheel_tf.position.y);
            wp.motor_joint = world_->CreateMotorJoint(mp.body, wp.body, wheel_pos, MOTOR_MAX_FORCE, MOTOR_MAX_TORQUE,
                                                      MOTOR_FREQUENCY, MOTOR_DAMPING_RATIO, body_mass);

            // Create angle joint to limit steering range
            float max_steering = std::abs(wheel_cfg.steering_max);
            if (max_steering > 0.0f) {
                wp.angle_joint = world_->CreateLimitedAngleJoint(mp.body, wp.body, -max_steering, max_steering);
            }

            // Configure wheel physics rates based on size
            float wheel_radius = static_cast<float>(wheel_cfg.size.x) / 2.0f;
            float size_factor = 0.2f / wheel_radius;
            wp.steering_rate = 1.04f * std::sqrt(size_factor);
            wp.steering_rate = std::clamp(wp.steering_rate, 0.52f, 2.10f);
            wp.throttle_rate = 2.5f * std::sqrt(size_factor);
            wp.throttle_rate = std::clamp(wp.throttle_rate, 1.0f, 5.0f);

            mp.wheels.push_back(wp);
        }

        // Add karosseries as colliders to machine body
        for (const auto &karos : machine.karosseries) {
            if (karos.has_physics) {
                muli::Transform karos_tf;
                karos_tf.position.x = static_cast<float>(karos.pose.point.x);
                karos_tf.position.y = static_cast<float>(karos.pose.point.y);
                karos_tf.rotation = static_cast<float>(karos.pose.angle.yaw);

                auto *karos_collider = mp.body->CreateBoxCollider(static_cast<float>(karos.size.x),
                                                                  static_cast<float>(karos.size.y), 0.02f, karos_tf);
                karos_collider->SetFilter(mp.filter);
            }
        }

        machines_[machine.uuid] = std::move(mp);
        std::cout << "[Simulator] Created machine: " << machine.name << " with " << machine.wheels.size() << " wheels"
                  << std::endl;
    }

    void Simulator::apply_control(const types::MachineControl &control, float dt) {
        auto it = machines_.find(control.uuid);
        if (it == machines_.end()) {
            return;
        }

        auto &mp = it->second;

        // Apply brake if requested
        if (control.brake > 0.0f) {
            for (auto &wp : mp.wheels) {
                if (!wp.body) continue;

                muli::Vec2 v = wp.body->GetLinearVelocity();
                float speed = muli::Length(v);

                if (speed > muli::epsilon) {
                    float wheel_radius = static_cast<float>(wp.config.size.x) / 2.0f;
                    float scale_factor = std::sqrt(wheel_radius / 0.2f);
                    float scaled_brake = control.brake * wp.config.brake * scale_factor;

                    muli::Vec2 brake_impulse = -muli::Normalize(v) * scaled_brake * wp.body->GetMass();
                    float max_impulse = wp.body->GetMass() * speed;
                    if (muli::Length(brake_impulse) > max_impulse) {
                        brake_impulse = muli::Normalize(brake_impulse) * max_impulse;
                    }
                    wp.body->ApplyLinearImpulse(wp.body->GetPosition(), brake_impulse, true);
                }
            }
            return;
        }

        // Apply steering and throttle to each wheel (with rate limiting)
        for (size_t i = 0; i < mp.wheels.size(); ++i) {
            auto &wp = mp.wheels[i];
            if (!wp.body) continue;

            float target_steering = (i < control.steering.size()) ? control.steering[i] : 0.0f;
            float target_throttle = (i < control.throttle.size()) ? control.throttle[i] : 0.0f;

            // Steering rate limiting
            float max_steering_change = wp.steering_rate * dt;
            float steering_error = target_steering - wp.current_steering;
            float steering_change = std::clamp(steering_error, -max_steering_change, max_steering_change);
            wp.current_steering += steering_change;

            if (wp.motor_joint) {
                wp.motor_joint->SetAngularOffset(wp.current_steering);
            }

            // Throttle rate limiting
            float max_throttle_change = wp.throttle_rate * dt;
            float throttle_error = target_throttle - wp.current_throttle;
            float throttle_change = std::clamp(throttle_error, -max_throttle_change, max_throttle_change);
            wp.current_throttle += throttle_change;

            // Apply throttle force
            if (std::abs(wp.current_throttle) > muli::epsilon) {
                const muli::Vec2 up(0, 1);
                muli::Vec2 forward = Mul(wp.body->GetRotation(), up);

                float wheel_radius = static_cast<float>(wp.config.size.x) / 2.0f;
                float scale_factor = std::sqrt(wheel_radius / 0.2f);
                float scaled_force = wp.config.force * scale_factor;

                muli::Vec2 f = forward * (wp.current_throttle * scaled_force);
                wp.body->ApplyForce(wp.body->GetPosition(), f, true);
            }
        }
    }

    bool Simulator::destroy_machine(const std::string &uuid) {
        auto it = machines_.find(uuid);
        if (it == machines_.end()) {
            return false;
        }

        auto &mp = it->second;

        // Destroy all wheel bodies
        for (auto &wp : mp.wheels) {
            if (wp.body) {
                world_->Destroy(wp.body);
            }
        }

        // Destroy machine body
        if (mp.body) {
            world_->Destroy(mp.body);
        }

        machines_.erase(it);
        std::cout << "[Simulator] Destroyed machine: " << uuid << std::endl;
        return true;
    }

    types::ser::WorldState Simulator::get_world_state() const {
        types::ser::WorldState state;

        for (const auto &[uuid, mp] : machines_) {
            types::ser::MachineState ms;
            ms.uuid = uuid;

            if (mp.body) {
                ms.pose.position.x = mp.body->GetPosition().x;
                ms.pose.position.y = mp.body->GetPosition().y;
                ms.pose.angle = mp.body->GetAngle();
                ms.velocity.x = mp.body->GetLinearVelocity().x;
                ms.velocity.y = mp.body->GetLinearVelocity().y;
                ms.angular_vel = mp.body->GetAngularVelocity();
            }

            // Add wheel states
            for (const auto &wp : mp.wheels) {
                if (wp.body) {
                    types::ser::WheelState ws;
                    ws.pose.position.x = wp.body->GetPosition().x;
                    ws.pose.position.y = wp.body->GetPosition().y;
                    ws.pose.angle = wp.body->GetAngle();
                    ws.velocity.x = wp.body->GetLinearVelocity().x;
                    ws.velocity.y = wp.body->GetLinearVelocity().y;
                    ws.angular_vel = wp.body->GetAngularVelocity();
                    ms.wheels.push_back(ws);
                }
            }

            state.machines.push_back(ms);
        }

        return state;
    }

    void Simulator::tick(float dt) {
        // Apply wheel physics (friction, drag) for each machine
        for (auto &[uuid, mp] : machines_) {
            for (auto &wp : mp.wheels) {
                if (!wp.body) continue;

                // Get wheel orientation vectors
                const muli::Vec2 up(0, 1);
                const muli::Vec2 right(1, 0);
                muli::Vec2 forward = Mul(wp.body->GetRotation(), up);
                muli::Vec2 normal = Mul(wp.body->GetRotation(), right);

                // Get velocity components
                muli::Vec2 v = wp.body->GetLinearVelocity();
                float vf = Dot(v, forward);
                float vn = Dot(v, normal);

                // Apply lateral friction (prevents sliding)
                if (muli::Abs(vn) > muli::epsilon) {
                    float wheel_radius = static_cast<float>(wp.config.size.x) / 2.0f;
                    float scaled_friction = wp.config.friction * (1.0f + wheel_radius);
                    muli::Vec2 j = -wp.body->GetMass() * scaled_friction * vn * normal;

                    float scaled_max_impulse = wp.config.max_impulse * (1.0f + wheel_radius * 2.0f);
                    if (muli::Length(j) > scaled_max_impulse) {
                        j = muli::Normalize(j) * scaled_max_impulse;
                    }
                    wp.body->ApplyLinearImpulse(wp.body->GetPosition(), j, true);
                }

                // Apply drag force
                if (muli::Abs(vf) > muli::epsilon) {
                    float drag_force = -wp.config.drag * vf;
                    wp.body->ApplyForce(wp.body->GetPosition(), drag_force * forward, true);
                }
            }
        }

        // Step physics
        world_->Step(dt);

        // Process ZMQ messages
        zmq::message_t request;
        auto result = socket_->recv(request, zmq::recv_flags::dontwait);
        if (result) {
            // Copy to aligned buffer for cista deserialization
            std::vector<uint8_t> buffer(static_cast<uint8_t *>(request.data()),
                                        static_cast<uint8_t *>(request.data()) + request.size());

            auto *req = cista::deserialize<types::ser::Request>(buffer);
            if (!req) {
                types::ser::Response resp;
                resp.success = false;
                auto data = cista::serialize(resp);
                socket_->send(zmq::buffer(data), zmq::send_flags::none);
                return;
            }

            types::ser::Response resp;
            switch (req->type) {
            case types::ser::MsgType::SPAWN: {
                auto machine = req->machine.to_machine();
                create_machine(machine);
                resp.success = true;
                resp.state = get_world_state();
                break;
            }
            case types::ser::MsgType::GET_STATE: {
                resp.success = true;
                resp.state = get_world_state();
                break;
            }
            case types::ser::MsgType::DESPAWN: {
                std::string uuid_str(req->uuid.view());
                resp.success = destroy_machine(uuid_str);
                break;
            }
            case types::ser::MsgType::CONTROL: {
                auto control = req->control.to_control();
                apply_control(control, dt);
                resp.success = true;
                resp.state = get_world_state();
                break;
            }
            }

            auto data = cista::serialize(resp);
            socket_->send(zmq::buffer(data), zmq::send_flags::none);
        }
    }

    void Simulator::tock() {
        // Visualization updates go here (if needed)
    }

} // namespace simulator
