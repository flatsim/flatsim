#pragma once

#include "concord/types_basic.hpp"
#include "muli/world.h"
#include "multiverse/robot/chasis/karosserie.hpp"
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
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<Wheel> wheelz;
        std::vector<muli::MotorJoint *> jointz;
        std::vector<Karosserie> karosseriez;
        muli::CollisionFilter filter;

      public:
        RigidBody *body;
        std::string name;
        pigment::RGB color;
        concord::Bound bound;
        uint32_t group;

        Chasis(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec, CollisionFilter filter);

        void init(concord::Bound &bound, const pigment::RGB &color, std::string name,
                  std::vector<concord::Bound> wheels, std::vector<concord::Bound> karosseries);

        void tick(float dt);
        void visualize();
        void teleport(concord::Pose);
        void update(float steering[4], float throttle[4]);
        muli::Transform get_transform() const;
    };
} // namespace mvs
