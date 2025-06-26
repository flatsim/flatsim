#pragma once

#include "muli/world.h"
#include "flatsim/constants.hpp"
#include "flatsim/exceptions.hpp"
#include "flatsim/robot/chassis/hitch.hpp"
#include "flatsim/robot/chassis/karosserie.hpp"
#include "flatsim/robot/chassis/wheel.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"

#include <rerun.hpp>
#include <spdlog/spdlog.h>

namespace fs {

    class Chassis {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<Wheel> wheels;
        std::vector<muli::MotorJoint *> joints;
        std::vector<muli::AngleJoint *> angle_joints;
        muli::CollisionFilter filter;

      private:
        std::string name;
        pigment::RGB color;
        concord::Bound bound;
        concord::Pose pose;
        uint32_t group;
        
      public:
        std::vector<Karosserie> karosseries;
        std::vector<Hitch> hitches;
        muli::RigidBody *body; // Owned by physics world

        Chassis(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec, muli::CollisionFilter filter);

        void init(fs::RobotInfo &robo);

        void tick(float dt);
        void visualize();
        void visualize(const std::string& label);
        void teleport(concord::Pose);
        void update(std::vector<float> steering, std::vector<float> throttle, float dt);
        void wheel_damping(float linear_damping, float angular_damping);
        muli::Transform get_transform() const;
        void toggle_work(const std::string& karosserie_name);
        
        // Accessors
        const concord::Pose& get_pose() const { return pose; }
        const concord::Bound& get_bound() const { return bound; }
        const std::string& get_name() const { return name; }
        muli::RigidBody* get_body() { return body; }
        
    };
} // namespace fs
