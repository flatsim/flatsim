#pragma once

#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "flywheel/world.h"
#include <rerun.hpp>

namespace simulator {
    // Remove duplicate function - use utils::deg2rad instead
    // --- Utility functions ---

    class Wheel {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<flywheel::World> world;
        std::string name;
        std::string parent_name;
        pigment::RGB color;
        datapod::Box bound;
        datapod::Pose pose;
        flywheel::CollisionFilter filter;
        types::Machine *robot_info = nullptr;
        types::State *robot_state = nullptr;

        flywheel::RigidBody *wheel; // Owned by physics world
        flywheel::Vec2 forward, normal;
        float force, torque;
        float brake, drag;
        float friction, max_impulse;

        float throttle_val = 0.0f, steering_val = 0.0f;
        float current_steering = 0.0f, current_throttle = 0.0f; // Actual current values
        float steering_max, throttle_max;

        // Physics-based acceleration limits
        float steering_rate; // rad/s - how fast steering can change
        float throttle_rate; // 1/s - how fast throttle can change

      public:
        Wheel() = default;
        Wheel(std::shared_ptr<flywheel::World> world, std::shared_ptr<rerun::RecordingStream> rec,
              flywheel::CollisionFilter filter, types::Machine *robot_info, types::State *robot_state);
        void init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                  datapod::Box bound, datapod::Box parent_bound, float _force, float _friction, float _maxImpulse,
                  float _brake, float _drag, float throttle_max, float steering_max);

        void tick(float dt);
        void tock();
        void teleport(datapod::Pose pose);
        void destroy();
        void update(float steering, float throttle, flywheel::MotorJoint *joint, float dt);
        void configure_physics_for_size();

        datapod::Box get_bound() const { return bound; }

        // Accessors for external access
        void set_linear_damping(float damping) {
            if (wheel) wheel->SetLinearDamping(damping);
        }
        void set_angular_damping(float damping) {
            if (wheel) wheel->SetAngularDamping(damping);
        }

        // Rate limiters - control how fast wheels can change steering/throttle
        void set_steering_rate(float rate_rad_per_sec) { steering_rate = rate_rad_per_sec; }
        void set_throttle_rate(float rate_per_sec) { throttle_rate = rate_per_sec; }
        float get_steering_rate() const { return steering_rate; }
        float get_throttle_rate() const { return throttle_rate; }

        flywheel::RigidBody *get_wheel() { return wheel; }
        flywheel::Vec2 get_position() const { return wheel ? wheel->GetPosition() : flywheel::Vec2(0, 0); }
        void update_color(const pigment::RGB &new_color) { color = new_color; }

        // Apply braking force to stop the wheel
        void apply_brake(float brake_force);
    };
} // namespace simulator
