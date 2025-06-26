#include "flatsim/robot/chassis/wheel.hpp"

namespace fs {

    Wheel::Wheel(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                 muli::CollisionFilter filter)
        : world(world), rec(rec), filter(filter) {}

    void Wheel::init(const pigment::RGB &color, const std::string& parent_name, const std::string& name, concord::Bound parent_bound,
                     concord::Bound bound, float _force, float _friction, float _maxImpulse, float _brake, float _drag,
                     float throttle_max, float steering_max) {
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
            throw InitializationException("Failed to create wheel body");
        }
        wheel->SetCollisionFilter(filter);
        force = _force;
        friction = _friction;
        max_impulse = _maxImpulse;
        brake = _brake;
        drag = _drag;
        
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

        visualize();
    }

    void Wheel::update(float steering, float throttle, muli::MotorJoint *joint, float dt) {
        throttle_val = throttle;
        steering_val = steering;
        joint->SetAngularOffset(steering);
        
        if (muli::Abs(throttle) > muli::epsilon) {
            // Scale force by wheel size and apply dt correctly
            float wheel_radius = bound.size.x / 2.0f;
            float scale_factor = muli::Sqrt(wheel_radius / 0.2f); // Normalize to typical wheel size
            float scaled_force = force * scale_factor;
            
            // Apply force scaled by dt for consistent acceleration
            muli::Vec2 f2 = forward * (throttle * scaled_force);
            wheel->ApplyForce(wheel->GetPosition(), f2, true);
        }
    }

    void Wheel::teleport(concord::Pose trans_pose) {
        wheel->SetTransform(utils::pose_to_transform(trans_pose));
        wheel->SetSleeping(true);
    }

    void Wheel::visualize() {
        auto x = wheel->GetPosition().x;
        auto y = wheel->GetPosition().y;
        auto th = wheel->GetRotation().GetAngle();

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(color.r, color.g, color.b));

        rec->log_static(
            this->parent_name + "/chassis/wheel/" + this->name,
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{float(bound.size.x), float(bound.size.y), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));
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
    }

} // namespace fs
