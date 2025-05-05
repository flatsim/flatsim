#pragma once

#include "concord/types_basic.hpp"
#include "muli/world.h"
#include "multiverse/robot/chasis/wheel.hpp"
#include "pigment/types_basic.hpp"

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

    class Chasis {
      private:
        World *world;
        RigidBody *body;
        Wheel wheels[4];
        MotorJoint *joints[4];

      public:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string name;
        uint32_t collision_id;
        pigment::RGB color;
        concord::Size size;

        Chasis(World *world, std::shared_ptr<rerun::RecordingStream> rec, const concord::Pose &pose,
               const concord::Size &size, const pigment::RGB &color, std::string name, uint32_t collision_id);

        void tick(float dt);
        void visualize();
        void teleport(concord::Pose);
        void update(float steering[4], float throttle[4]);
        muli::Transform get_transform() const;
    };
} // namespace mvs
