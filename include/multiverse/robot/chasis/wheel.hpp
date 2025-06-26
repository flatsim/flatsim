#pragma once

#include "muli/world.h"
#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"

namespace mvs {
    // Remove duplicate function - use utils::deg2rad instead
    // --- Utility functions ---

    class Wheel {
      public:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::string name;
        std::string parent_name;
        pigment::RGB color;
        concord::Bound bound;
        concord::Pose pose;
        muli::CollisionFilter filter;

        muli::RigidBody *wheel;
        muli::Vec2 forward, normal;
        float force, torque;
        float brake, drag;
        float friction, maxImpulse;

        float throttle_val = 0.0f, steering_val = 0.0f;
        float steering_max, throttle_max;

        Wheel() = default;
        Wheel(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec, muli::CollisionFilter filter);
        void init(const pigment::RGB &color, std::string parent_name, std::string name, concord::Bound bound,
                  concord::Bound parent_bound, float _force, float _friction, float _maxImpulse, float _brake,
                  float _drag, float throttle_max, float steering_max);

        void tick(float dt);
        void visualize();
        void teleport(concord::Pose pose);
        void update(float steering, float throttle, muli::MotorJoint *joint, float dt);
        void configure_physics_for_size();

        concord::Bound get_bound() const { return bound; }
    };
} // namespace mvs
