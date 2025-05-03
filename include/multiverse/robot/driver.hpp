#pragma once
#include "concord/types_basic.hpp"
#include "muli/world.h"
#include "pigment/types_basic.hpp"
#include "pigment/types_hsv.hpp"

#include <cmath> // for M_PI
#include <memory>
#include <rerun.hpp>

namespace mvs {
    using namespace muli;

    // --- Global toggles & parameters ----------------
    inline constexpr bool followCam = true;
    inline constexpr bool rotateCam = false;
    inline constexpr bool drawAxis = false;
    inline constexpr float linearDamping = 0.2f;
    inline constexpr float angularDamping = 2.0f;
    inline constexpr float force = 30.0f;
    inline constexpr float torque = 10.0f;
    inline constexpr float friction = 0.4f;
    inline constexpr float maxImpulse = 0.5f;
    inline constexpr float brake = 10.0f;
    inline constexpr float drag = 0.5f;

    // --- Utility functions ---
    inline float DegToRad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

    struct Wheel {
        RigidBody *wheel;
        Vec2 forward, normal;
        float force, torque;
        float brake, drag;
        float friction, maxImpulse;
        std::string name;
        pigment::RGB color;
        std::shared_ptr<rerun::RecordingStream> rec;

        void init(World *world, std::shared_ptr<rerun::RecordingStream> rec, const pigment::RGB &color,
                  std::string name, float scale, Transform tf, CollisionFilter filter, float linearDamping,
                  float angularDamping, float _force, float _friction, float _maxImpulse, float _brake, float _drag);

        void step(float dt);
        void visualize();
    };

    class Vehicle {
      public:
        std::string name;
        pigment::RGB color;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::vector<float> size;
        Wheel wheels[4];
        RigidBody *body;

        Vehicle(World *world, std::shared_ptr<rerun::RecordingStream> rec, const concord::Pose &pose,
                const concord::Size &size, const pigment::RGB &color, std::string name);
        void tick(float dt);
        void update(float steering, float throttle);
        std::vector<float> get_position() const;

        void visualize();

      private:
        World *world;
        MotorJoint *joints[4];
    };

} // namespace mvs
