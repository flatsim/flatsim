#pragma once

#include "flatsim/core/constants.hpp"
#include "flatsim/core/utils.hpp"
#include "flatsim/simulator/machine/hitch.hpp"
#include "flatsim/simulator/machine/karosserie.hpp"
#include "flatsim/simulator/machine/wheel.hpp"
#include "flatsim/types.hpp"
#include "muli/world.h"
#include <rerun.hpp>

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
        types::Machine *robot_info = nullptr;
        types::State *robot_state = nullptr;

      public:
        std::vector<Karosserie> karosseries;
        std::vector<Hitch> hitches;
        muli::RigidBody *body; // Owned by physics world

        Chassis(std::shared_ptr<muli::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                muli::CollisionFilter filter, types::Machine *robot_info, types::State *robot_state);

        void init(types::Machine &robo);

        void tick(float dt);
        void tock(const std::string &label);
        void teleport(concord::Pose);
        void update(std::vector<float> steering, std::vector<float> throttle, float dt);
        void wheel_damping(float linear_damping, float angular_damping);
        muli::Transform get_transform() const;
        void toggle_section_work(const std::string &karosserie_name, int section_id);
        void toggle_all_sections_work(const std::string &karosserie_name);
        void update_color(const pigment::RGB &new_color);
        void toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id);

        // Accessors
        const concord::Pose &get_pose() const { return pose; }
        const concord::Bound &get_bound() const { return bound; }
        const std::string &get_name() const { return name; }
        muli::RigidBody *get_body() { return body; }

        // Apply braking to all wheels
        void brake(float brake_force);
    };
} // namespace fs
