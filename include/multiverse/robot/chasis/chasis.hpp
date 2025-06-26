#pragma once

#include "muli/world.h"
#include "multiverse/constants.hpp"
#include "multiverse/robot/chasis/hitch.hpp"
#include "multiverse/robot/chasis/karosserie.hpp"
#include "multiverse/robot/chasis/wheel.hpp"
#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"

#include <rerun.hpp>
#include <spdlog/spdlog.h>

namespace mvs {

    class Chasis {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<Wheel> wheelz;
        std::vector<muli::MotorJoint *> jointz;
        std::vector<muli::AngleJoint *> anglejointz;
        muli::CollisionFilter filter;

      public:
        std::shared_ptr<muli::RigidBody> bodyz;
        std::vector<Karosserie> karosseriez;
        std::vector<Hitch> hitchz;
        muli::RigidBody *body;
        std::string name;
        pigment::RGB color;
        concord::Bound bound;
        concord::Pose pose;
        uint32_t group;

        Chasis(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec, muli::CollisionFilter filter);

        void init(mvs::RobotInfo &robo);

        void tick(float dt);
        void visualize();
        void teleport(concord::Pose);
        void update(std::vector<float> steering, std::vector<float> throttle, float dt);
        void wheel_damping(float linearDamping, float angularDamping);
        muli::Transform get_transform() const;
        void toggle_work(std::string karosserie_name);
    };
} // namespace mvs
