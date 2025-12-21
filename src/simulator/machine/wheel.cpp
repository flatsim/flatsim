#include "flatsim/simulator/machine/wheel.hpp"

namespace simulator {

    Wheel::Wheel(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                 muli::CollisionFilter filter, types::Machine *robot_info, types::State *robot_state)
        : world(world), rec(rec), filter(filter), robot_info(robot_info), robot_state(robot_state) {}

    void Wheel::init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                     concord::Bound parent_bound, concord::Bound bound, float _force, float _friction,
                     float _maxImpulse, float _brake, float _drag, float throttle_max, float steering_max) {
        this->bound = bound;
        this->color = color;
        this->name = name;
        this->parent_name = parent_name;
        this->steering_max = steering_max;
        this->throttle_max = throttle_max;

        pose = utils::shift(parent_bound.pose, bound.pose);
        auto wheel_tf = utils::pose_to_transform(pose);

        wheel = world->CreateBox(bound.size.x, bound.size.y, wheel_tf);
        if (!wheel) {
            throw std::runtime_error("Failed to create wheel body");
        }
        wheel->SetCollisionFilter(filter);
        force = _force;
        friction = _friction;
        max_impulse = _maxImpulse;
        brake = _brake;
        drag = _drag;

        // Physics-based acceleration limits
        // Steering rate: realistic steering actuators can turn ~30-45 degrees per second
        // For tractors/heavy vehicles: slower (~20-30 deg/s)
        // For cars: faster (~45-60 deg/s)
        // We'll use 30 deg/s as a reasonable default (0.52 rad/s)
        steering_rate = 0.52f; // radians per second

        // Throttle rate: how fast the throttle/brake pedal can be actuated
        // Realistic: 0-100% in about 0.3-0.5 seconds for aggressive driving
        // For heavy machinery: 0.5-1.0 seconds
        // We'll use 2.0/s meaning 0-100% in 0.5 seconds
        throttle_rate = 2.5f; // units per second (throttle is -1 to 1)

        configure_physics_for_size();
    }

    void Wheel::tick(float dt) {
        const muli::Vec2 up(0, 1);
        const muli::Vec2 right(1, 0);

        forward = Mul(wheel->GetRotation(), up);
        normal = Mul(wheel->GetRotation(), right);

        muli::Vec2 v = wheel->GetLinearVelocity();
        float vf = Dot(v, forward);
        float vn = Dot(v, normal);

        // Apply lateral friction to prevent sliding
        if (muli::Abs(vn) > muli::epsilon) {
            // Scale friction impulse by wheel size and dt
            float wheel_radius = bound.size.x / 2.0f;
            float scaled_friction = friction * (1.0f + wheel_radius);
            muli::Vec2 j = -wheel->GetMass() * scaled_friction * vn * normal;

            // Scale max impulse by wheel size
            float scaled_max_impulse = max_impulse * (1.0f + wheel_radius * 2.0f);
            if (muli::Length(j) > scaled_max_impulse) {
                j = muli::Normalize(j) * scaled_max_impulse;
            }
            wheel->ApplyLinearImpulse(wheel->GetPosition(), j, true);
        }

        // Apply drag force (velocity-dependent)
        if (muli::Abs(vf) > muli::epsilon) {
            float dragForceMagnitude = -drag * vf;
            wheel->ApplyForce(wheel->GetPosition(), dragForceMagnitude * forward, true);
        }
    }

    void Wheel::update(float steering, float throttle, muli::MotorJoint *joint, float dt) {
        // Store target values
        throttle_val = throttle;
        steering_val = steering;

        // ============================================================================
        // STEERING RATE LIMITING - Gradual steering angle changes
        // ============================================================================
        // Calculate maximum change allowed this frame based on steering rate
        float max_steering_change = steering_rate * dt;

        // Calculate the difference between target and current steering
        float steering_error = steering - current_steering;

        // Clamp the change to the maximum allowed
        float steering_change = muli::Clamp(steering_error, -max_steering_change, max_steering_change);

        // Update current steering gradually
        current_steering += steering_change;

        // Apply the gradual steering to the joint
        joint->SetAngularOffset(current_steering);

        // ============================================================================
        // THROTTLE RATE LIMITING - Gradual throttle changes
        // ============================================================================
        // Calculate maximum change allowed this frame based on throttle rate
        float max_throttle_change = throttle_rate * dt;

        // Calculate the difference between target and current throttle
        float throttle_error = throttle - current_throttle;

        // Clamp the change to the maximum allowed
        float throttle_change = muli::Clamp(throttle_error, -max_throttle_change, max_throttle_change);

        // Update current throttle gradually
        current_throttle += throttle_change;

        // Apply force using the gradual throttle value
        if (muli::Abs(current_throttle) > muli::epsilon) {
            // Scale force by wheel size and apply dt correctly
            float wheel_radius = bound.size.x / 2.0f;
            float scale_factor = muli::Sqrt(wheel_radius / 0.2f); // Normalize to typical wheel size
            float scaled_force = force * scale_factor;

            // Apply force scaled by the gradual throttle value
            muli::Vec2 f2 = forward * (current_throttle * scaled_force);
            wheel->ApplyForce(wheel->GetPosition(), f2, true);
        }
    }

    void Wheel::teleport(concord::Pose trans_pose) {
        wheel->SetTransform(utils::pose_to_transform(trans_pose));
        wheel->SetSleeping(true);
    }

    void Wheel::tock() {
        if (!robot_state->online) return;
        if (!rec) return;

        auto x = wheel->GetPosition().x;
        auto y = wheel->GetPosition().y;
        auto th = wheel->GetRotation().GetAngle();

        rec->log_static(
            robot_info->uuid + "/chassis/wheel/" + this->name,
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{float(bound.size.x), float(bound.size.y), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors({rerun::Color(color.r, color.g, color.b)}));
    }

    void Wheel::configure_physics_for_size() {
        float wheel_radius = bound.size.x / 2.0f;

        // Scale damping based on wheel size
        // Larger wheels need more damping to prevent oscillation
        float linear_damping = 0.2f + (wheel_radius - 0.1f) * 0.3f;
        float angular_damping = 0.5f + (wheel_radius - 0.1f) * 1.5f;

        // Clamp values to reasonable ranges
        linear_damping = muli::Clamp(linear_damping, 0.2f, 0.8f);
        angular_damping = muli::Clamp(angular_damping, 0.5f, 3.0f);

        wheel->SetLinearDamping(linear_damping);
        wheel->SetAngularDamping(angular_damping);

        // Adjust steering rate based on wheel size
        // Larger wheels (like tractor wheels) turn slower due to more inertia
        // Small wheels (like car wheels) can turn faster
        // Base rate is 1.04 rad/s (~60 deg/s) for a 0.2m radius wheel
        float size_factor = 0.2f / wheel_radius; // Inverse relationship
        steering_rate = 1.04f * muli::Sqrt(size_factor);

        // Clamp to reasonable ranges: 30-120 deg/s (0.52-2.10 rad/s)
        steering_rate = muli::Clamp(steering_rate, 0.52f, 2.10f);

        // Larger vehicles also have slower throttle response
        // Base rate is 2.5/s for a 0.2m radius wheel
        throttle_rate = 2.5f * muli::Sqrt(size_factor);

        // Clamp to reasonable ranges: 1.0-5.0 per second
        throttle_rate = muli::Clamp(throttle_rate, 1.0f, 5.0f);
    }

    void Wheel::apply_brake(float brake_force) {
        if (!wheel) return;

        // Get current velocity
        muli::Vec2 v = wheel->GetLinearVelocity();
        float speed = muli::Length(v);

        if (speed < muli::epsilon) {
            // Already stopped, just zero out any residual velocity
            wheel->SetLinearVelocity(muli::Vec2(0, 0));
            wheel->SetAngularVelocity(0);
            return;
        }

        // Apply braking force opposite to velocity direction
        // Scale by wheel size for consistent braking across different vehicles
        float wheel_radius = bound.size.x / 2.0f;
        float scale_factor = muli::Sqrt(wheel_radius / 0.2f);
        float scaled_brake = brake_force * scale_factor;

        // Calculate braking impulse (opposite to velocity)
        muli::Vec2 brake_impulse = -muli::Normalize(v) * scaled_brake * wheel->GetMass();

        // Clamp impulse to not exceed current momentum (prevents reversing)
        float max_impulse = wheel->GetMass() * speed;
        if (muli::Length(brake_impulse) > max_impulse) {
            brake_impulse = muli::Normalize(brake_impulse) * max_impulse;
        }

        wheel->ApplyLinearImpulse(wheel->GetPosition(), brake_impulse, true);

        // Also apply angular braking to stop wheel rotation
        float angular_vel = wheel->GetAngularVelocity();
        if (muli::Abs(angular_vel) > muli::epsilon) {
            float angular_brake = -angular_vel * scaled_brake * 0.1f;
            wheel->ApplyTorque(angular_brake, true);
        }
    }

} // namespace simulator
