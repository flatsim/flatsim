#include "multiverse/robot/chasis/wheel.hpp"

namespace mvs {

    Wheel::Wheel(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                 muli::CollisionFilter filter)
        : world(world), rec(rec), filter(filter) {}

    void Wheel::init(const pigment::RGB &color, std::string parent_name, std::string name, concord::Bound parent_bound,
                     concord::Bound bound, float _force, float _friction, float _maxImpulse, float _brake, float _drag,
                     float throttle_max, float steering_max) {
        this->bound = bound;
        this->color = color;
        this->name = name;
        this->parent_name = parent_name;
        this->steering_max = steering_max;
        this->throttle_max = throttle_max;

        pose = utils::shift(parent_bound.pose, bound.pose);
        auto wheelTf = utils::pose_to_transform(pose);

        wheel = world->CreateCapsule(bound.size.y, bound.size.x, false, wheelTf);
        if (!wheel) {
            throw InitializationException("Failed to create wheel body");
        }
        wheel->SetCollisionFilter(filter);
        force = _force;
        friction = _friction;
        maxImpulse = _maxImpulse;
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
            float wheelRadius = bound.size.x / 2.0f;
            float scaledFriction = friction * (1.0f + wheelRadius);
            muli::Vec2 j = -wheel->GetMass() * scaledFriction * vn * normal;
            
            // Scale max impulse by wheel size
            float scaledMaxImpulse = maxImpulse * (1.0f + wheelRadius * 2.0f);
            if (muli::Length(j) > scaledMaxImpulse) {
                j = muli::Normalize(j) * scaledMaxImpulse;
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
            float wheelRadius = bound.size.x / 2.0f;
            float scaleFactor = muli::Sqrt(wheelRadius / 0.2f); // Normalize to typical wheel size
            float scaledForce = force * scaleFactor;
            
            // Apply force scaled by dt for consistent acceleration
            muli::Vec2 f2 = forward * (throttle * scaledForce);
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
            this->parent_name + "/chasis/wheel/" + this->name,
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{float(bound.size.x), float(bound.size.y), 0.0f}})
                .with_radii({{0.02f}})
                .with_fill_mode(rerun::FillMode::Solid)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors(colors));
    }
    
    void Wheel::configure_physics_for_size() {
        float wheelRadius = bound.size.x / 2.0f;
        
        // Scale damping based on wheel size
        // Larger wheels need more damping to prevent oscillation
        float linearDamping = 0.2f + (wheelRadius - 0.1f) * 0.3f;
        float angularDamping = 0.5f + (wheelRadius - 0.1f) * 1.5f;
        
        // Clamp values to reasonable ranges
        linearDamping = muli::Clamp(linearDamping, 0.2f, 0.8f);
        angularDamping = muli::Clamp(angularDamping, 0.5f, 3.0f);
        
        wheel->SetLinearDamping(linearDamping);
        wheel->SetAngularDamping(angularDamping);
    }

} // namespace mvs
