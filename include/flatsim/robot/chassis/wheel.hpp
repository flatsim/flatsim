#pragma once

#include "muli/world.h"
#include "flatsim/exceptions.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"

namespace fs {
    // Remove duplicate function - use utils::deg2rad instead
    // --- Utility functions ---

    class Wheel {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::string name;
        std::string parent_name;
        pigment::RGB color;
        concord::Bound bound;
        concord::Pose pose;
        muli::CollisionFilter filter;

        muli::RigidBody *wheel; // Owned by physics world
        muli::Vec2 forward, normal;
        float force, torque;
        float brake, drag;
        float friction, max_impulse;

        float throttle_val = 0.0f, steering_val = 0.0f;
        float steering_max, throttle_max;

      public:

        Wheel() = default;
        Wheel(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec, muli::CollisionFilter filter);
        void init(const pigment::RGB &color, const std::string& parent_name, const std::string& name, concord::Bound bound,
                  concord::Bound parent_bound, float _force, float _friction, float _maxImpulse, float _brake,
                  float _drag, float throttle_max, float steering_max);

        void tick(float dt);
        void visualize();
        void teleport(concord::Pose pose);
        void update(float steering, float throttle, muli::MotorJoint *joint, float dt);
        void configure_physics_for_size();

        concord::Bound get_bound() const { return bound; }
        
        // Accessors for external access
        void set_linear_damping(float damping) { if (wheel) wheel->SetLinearDamping(damping); }
        void set_angular_damping(float damping) { if (wheel) wheel->SetAngularDamping(damping); }
        muli::RigidBody* get_wheel() { return wheel; }
        muli::Vec2 get_position() const { return wheel ? wheel->GetPosition() : muli::Vec2(0, 0); }
        void update_color(const pigment::RGB& new_color) { color = new_color; }
    };
} // namespace fs
