#include "flatsim/simulator/machine.hpp"
#include <cmath>
#include <iostream>

namespace simulator {

    // Physics constants
    constexpr float LINEAR_DAMPING = 0.3f;
    constexpr float ANGULAR_DAMPING = 0.5f;

    concord::Pose Machine::shift_pose(const concord::Pose &parent, const concord::Pose &child) {
        float cos_a = std::cos(static_cast<float>(parent.angle.yaw));
        float sin_a = std::sin(static_cast<float>(parent.angle.yaw));

        concord::Pose result;
        result.point.x = parent.point.x + child.point.x * cos_a - child.point.y * sin_a;
        result.point.y = parent.point.y + child.point.x * sin_a + child.point.y * cos_a;
        result.angle.yaw = parent.angle.yaw + child.angle.yaw;
        return result;
    }

    Machine::Machine(const types::Machine &config) : config_(config) {}

    void Machine::create(muli::World &world, uint32_t group) {
        // Create collision filter (negative group = never collide with same group)
        filter_.group = -static_cast<int>(group);
        filter_.bit = 1;
        filter_.mask = 0xFFFFFFFF;

        // Create machine body transform
        muli::Transform machine_tf;
        machine_tf.position.x = static_cast<float>(config_.pose.point.x);
        machine_tf.position.y = static_cast<float>(config_.pose.point.y);
        machine_tf.rotation = static_cast<float>(config_.pose.angle.yaw);

        // Create empty body for compound shape
        body_ = world.CreateEmptyBody(machine_tf);
        if (!body_) {
            std::cerr << "[Simulator] Failed to create body for: " << config_.name << std::endl;
            return;
        }

        // Add main machine collider
        auto *machine_collider =
            body_->CreateBoxCollider(static_cast<float>(config_.size.x), static_cast<float>(config_.size.y));
        body_->SetCollisionFilter(filter_);
        machine_collider->SetFilter(filter_);

        body_->SetLinearDamping(LINEAR_DAMPING);
        body_->SetAngularDamping(ANGULAR_DAMPING);

        float body_mass = body_->GetMass();

        // Create wheels
        for (const auto &wheel_cfg : config_.wheels) {
            Wheel wheel(wheel_cfg);
            concord::Pose wheel_pose = shift_pose(config_.pose, wheel_cfg.pose);
            wheel.create(world, body_, wheel_pose, filter_, body_mass);
            wheels_.push_back(std::move(wheel));
        }

        // Create karosseries
        for (const auto &karos_cfg : config_.karosseries) {
            Karosserie karos(karos_cfg);
            karos.create(body_, filter_);
            karosseries_.push_back(std::move(karos));
        }

        // Create hitches
        for (const auto &hitch_cfg : config_.hitches) {
            hitches_.emplace_back(hitch_cfg);
        }

        std::cout << "[Simulator] Created machine: " << config_.name << " with " << wheels_.size() << " wheels"
                  << std::endl;
    }

    void Machine::destroy(muli::World &world) {
        // Destroy wheels
        for (auto &wheel : wheels_) {
            wheel.destroy(world);
        }
        wheels_.clear();

        // Destroy machine body
        if (body_) {
            world.Destroy(body_);
            body_ = nullptr;
        }

        karosseries_.clear();
        hitches_.clear();

        std::cout << "[Simulator] Destroyed machine: " << config_.uuid << std::endl;
    }

    void Machine::apply_control(const types::MachineControl &control, float dt) {
        // Apply brake if requested
        if (control.brake > 0.0f) {
            for (auto &wheel : wheels_) {
                wheel.apply_brake(control.brake);
            }
            return;
        }

        // Apply steering and throttle to each wheel
        for (size_t i = 0; i < wheels_.size(); ++i) {
            float target_steering = (i < control.steering.size()) ? control.steering[i] : 0.0f;
            float target_throttle = (i < control.throttle.size()) ? control.throttle[i] : 0.0f;
            wheels_[i].apply_control(target_steering, target_throttle, dt);
        }
    }

    void Machine::apply_physics() {
        for (auto &wheel : wheels_) {
            wheel.apply_friction();
        }
    }

    void Machine::tick(float dt) {
        // Apply physics to all components
        apply_physics();

        // Tick all wheels (updates cached direction vectors)
        for (auto &wheel : wheels_) {
            wheel.tick(dt);
        }

        // Tick all karosseries
        for (auto &karosserie : karosseries_) {
            karosserie.tick(dt);
        }

        // Tick all hitches
        for (auto &hitch : hitches_) {
            hitch.tick(dt);
        }
    }

    void Machine::tock() {
        // Visualization/debug for all components
        for (auto &wheel : wheels_) {
            wheel.tock();
        }

        for (auto &karosserie : karosseries_) {
            karosserie.tock();
        }

        for (auto &hitch : hitches_) {
            hitch.tock();
        }
    }

    types::ser::MachineState Machine::get_state() const {
        types::ser::MachineState ms;
        ms.uuid = config_.uuid;

        if (body_) {
            ms.pose.position.x = body_->GetPosition().x;
            ms.pose.position.y = body_->GetPosition().y;
            ms.pose.angle = body_->GetAngle();
            ms.velocity.x = body_->GetLinearVelocity().x;
            ms.velocity.y = body_->GetLinearVelocity().y;

            // Debug
            static int state_count = 0;
            if (state_count++ % 120 == 0) {
                std::cout << "[Machine sim] get_state " << state_count << " - Pos: (" << ms.pose.position.x << ", "
                          << ms.pose.position.y << "), Vel: (" << ms.velocity.x << ", " << ms.velocity.y << ")"
                          << std::endl;
            }
            ms.angular_vel = body_->GetAngularVelocity();
        }

        // Add wheel states
        for (const auto &wheel : wheels_) {
            ms.wheels.push_back(wheel.get_state());
        }

        return ms;
    }

    Hitch *Machine::find_hitch(const std::string &name) {
        for (auto &hitch : hitches_) {
            if (hitch.config().name == name) {
                return &hitch;
            }
        }
        return nullptr;
    }

} // namespace simulator
