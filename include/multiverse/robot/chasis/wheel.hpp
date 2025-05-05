#pragma once

#include "concord/types_basic.hpp"
#include "muli/world.h"
#include "pigment/types_basic.hpp"
#include <rerun.hpp>

namespace mvs {
    using namespace muli;
    // --- Utility functions ---
    inline float DegToRad(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }

    struct Wheel {
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string name;
        pigment::RGB color;
        concord::Size size;

        RigidBody *wheel;
        Vec2 forward, normal;
        float force, torque;
        float brake, drag;
        float friction, maxImpulse;

        void init(World *world, std::shared_ptr<rerun::RecordingStream> rec, const pigment::RGB &color,
                  std::string name, concord::Size size, Transform tf, CollisionFilter filter, float linearDamping,
                  float angularDamping, float _force, float _friction, float _maxImpulse, float _brake, float _drag);

        void tick(float dt);
        void visualize();
        void teleport(Transform t);
        void update(float steering, float throttle, MotorJoint *joint);
    };
} // namespace mvs
