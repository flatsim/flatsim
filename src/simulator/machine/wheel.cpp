#include "flatsim/simulator/machine/wheel.hpp"
#include <algorithm>
#include <cmath>

namespace simulator {

    // Physics constants
    constexpr float LINEAR_DAMPING = 0.3f;
    constexpr float ANGULAR_DAMPING = 0.5f;
    constexpr float MOTOR_MAX_FORCE = 300.0f;
    constexpr float MOTOR_MAX_TORQUE = 100.0f;
    constexpr float MOTOR_FREQUENCY = 30.0f;
    constexpr float MOTOR_DAMPING_RATIO = 1.0f;

    Wheel::Wheel(const types::Wheel &config) : config_(config) {}

    void Wheel::create(muli::World &world, muli::RigidBody *parent_body, const concord::Pose &world_pose,
                       const muli::CollisionFilter &filter, float parent_mass) {
        // Create wheel body
        muli::Transform wheel_tf;
        wheel_tf.position.x = static_cast<float>(world_pose.point.x);
        wheel_tf.position.y = static_cast<float>(world_pose.point.y);
        wheel_tf.rotation = static_cast<float>(world_pose.angle.yaw);

        body_ = world.CreateBox(static_cast<float>(config_.size.x), static_cast<float>(config_.size.y), wheel_tf);
        if (!body_) {
            return;
        }

        body_->SetCollisionFilter(filter);
        body_->SetLinearDamping(LINEAR_DAMPING);
        body_->SetAngularDamping(ANGULAR_DAMPING);

        // Create motor joint connecting wheel to machine
        muli::Vec2 wheel_pos(wheel_tf.position.x, wheel_tf.position.y);
        motor_joint_ = world.CreateMotorJoint(parent_body, body_, wheel_pos, MOTOR_MAX_FORCE, MOTOR_MAX_TORQUE,
                                              MOTOR_FREQUENCY, MOTOR_DAMPING_RATIO, parent_mass);

        // Create angle joint to limit steering range
        float max_steering = std::abs(config_.steering_max);
        if (max_steering > 0.0f) {
            angle_joint_ = world.CreateLimitedAngleJoint(parent_body, body_, -max_steering, max_steering);
        }

        // Configure wheel physics rates based on size
        float wheel_radius = static_cast<float>(config_.size.x) / 2.0f;
        float size_factor = 0.2f / wheel_radius;
        steering_rate_ = 1.04f * std::sqrt(size_factor);
        steering_rate_ = std::clamp(steering_rate_, 0.52f, 2.10f);
        throttle_rate_ = 2.5f * std::sqrt(size_factor);
        throttle_rate_ = std::clamp(throttle_rate_, 1.0f, 5.0f);
    }

    void Wheel::destroy(muli::World &world) {
        if (body_) {
            world.Destroy(body_);
            body_ = nullptr;
            motor_joint_ = nullptr;
            angle_joint_ = nullptr;
        }
    }

    void Wheel::apply_control(float target_steering, float target_throttle, float dt) {
        if (!body_) return;

        // Steering rate limiting
        float max_steering_change = steering_rate_ * dt;
        float steering_error = target_steering - current_steering_;
        float steering_change = std::clamp(steering_error, -max_steering_change, max_steering_change);
        current_steering_ += steering_change;

        if (motor_joint_) {
            motor_joint_->SetAngularOffset(current_steering_);
        }

        // Throttle rate limiting
        float max_throttle_change = throttle_rate_ * dt;
        float throttle_error = target_throttle - current_throttle_;
        float throttle_change = std::clamp(throttle_error, -max_throttle_change, max_throttle_change);
        current_throttle_ += throttle_change;

        // Apply throttle force
        if (std::abs(current_throttle_) > muli::epsilon) {
            const muli::Vec2 up(0, 1);
            muli::Vec2 forward = Mul(body_->GetRotation(), up);

            float wheel_radius = static_cast<float>(config_.size.x) / 2.0f;
            float scale_factor = std::sqrt(wheel_radius / 0.2f);
            float scaled_force = config_.force * scale_factor;

            muli::Vec2 f = forward * (current_throttle_ * scaled_force);
            body_->ApplyForce(body_->GetPosition(), f, true);
        }
    }

    void Wheel::apply_brake(float brake_force) {
        if (!body_) return;

        muli::Vec2 v = body_->GetLinearVelocity();
        float speed = muli::Length(v);

        if (speed > muli::epsilon) {
            float wheel_radius = static_cast<float>(config_.size.x) / 2.0f;
            float scale_factor = std::sqrt(wheel_radius / 0.2f);
            float scaled_brake = brake_force * config_.brake * scale_factor;

            muli::Vec2 brake_impulse = -muli::Normalize(v) * scaled_brake * body_->GetMass();
            float max_impulse = body_->GetMass() * speed;
            if (muli::Length(brake_impulse) > max_impulse) {
                brake_impulse = muli::Normalize(brake_impulse) * max_impulse;
            }
            body_->ApplyLinearImpulse(body_->GetPosition(), brake_impulse, true);
        }
    }

    void Wheel::apply_friction() {
        if (!body_) return;

        // Get wheel orientation vectors
        const muli::Vec2 up(0, 1);
        const muli::Vec2 right(1, 0);
        muli::Vec2 forward = Mul(body_->GetRotation(), up);
        muli::Vec2 normal = Mul(body_->GetRotation(), right);

        // Get velocity components
        muli::Vec2 v = body_->GetLinearVelocity();
        float vf = Dot(v, forward);
        float vn = Dot(v, normal);

        // Apply lateral friction (prevents sliding)
        if (muli::Abs(vn) > muli::epsilon) {
            float wheel_radius = static_cast<float>(config_.size.x) / 2.0f;
            float scaled_friction = config_.friction * (1.0f + wheel_radius);
            muli::Vec2 j = -body_->GetMass() * scaled_friction * vn * normal;

            float scaled_max_impulse = config_.max_impulse * (1.0f + wheel_radius * 2.0f);
            if (muli::Length(j) > scaled_max_impulse) {
                j = muli::Normalize(j) * scaled_max_impulse;
            }
            body_->ApplyLinearImpulse(body_->GetPosition(), j, true);
        }

        // Apply drag force
        if (muli::Abs(vf) > muli::epsilon) {
            float drag_force = -config_.drag * vf;
            body_->ApplyForce(body_->GetPosition(), drag_force * forward, true);
        }
    }

    types::ser::WheelState Wheel::get_state() const {
        types::ser::WheelState ws;
        if (body_) {
            ws.pose.position.x = body_->GetPosition().x;
            ws.pose.position.y = body_->GetPosition().y;
            ws.pose.angle = body_->GetAngle();
            ws.velocity.x = body_->GetLinearVelocity().x;
            ws.velocity.y = body_->GetLinearVelocity().y;
            ws.angular_vel = body_->GetAngularVelocity();
        }
        return ws;
    }

} // namespace simulator
