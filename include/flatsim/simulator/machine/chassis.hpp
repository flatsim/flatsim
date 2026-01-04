#pragma once

#include <vector>

#include "flatsim/simulator/machine/hitch.hpp"
#include "flatsim/simulator/machine/karosserie.hpp"
#include "flatsim/simulator/machine/wheel.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "flywheel/flywheel.hpp"
#include <rerun.hpp>

namespace simulator {

    class Chassis {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<flywheel::World> world;
        std::vector<Wheel> wheels;
        std::vector<flywheel::MotorJoint *> joints;
        std::vector<flywheel::AngleJoint *> angle_joints;
        flywheel::CollisionFilter filter;

      private:
        std::string name;
        pigment::RGB color;
        datapod::Box bound;
        datapod::Pose pose;
        uint32_t group;
        types::Machine *robot_info = nullptr;
        types::State *robot_state = nullptr;

      public:
        std::vector<Karosserie> karosseries;
        std::vector<Hitch> hitches;
        flywheel::RigidBody *body; // Owned by physics world

        Chassis(std::shared_ptr<flywheel::World> world, std::shared_ptr<rerun::RecordingStream> rec,
                flywheel::CollisionFilter filter, types::Machine *robot_info, types::State *robot_state);

        void init(types::Machine &robo);

        void tick(float dt);
        void tock(const std::string &label);
        void teleport(datapod::Pose);
        void update(std::vector<float> steering, std::vector<float> throttle, float dt);
        void wheel_damping(float linear_damping, float angular_damping);
        flywheel::Transform get_transform() const;
        void toggle_section_work(const std::string &karosserie_name, int section_id);
        void toggle_all_sections_work(const std::string &karosserie_name);
        void update_color(const pigment::RGB &new_color);
        void toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id);

        // Accessors
        const datapod::Pose &get_pose() const { return pose; }
        const datapod::Box &get_bound() const { return bound; }
        const std::string &get_name() const { return name; }
        flywheel::RigidBody *get_body() { return body; }

        // Apply braking to all wheels
        void brake(float brake_force);

        // Destroy all physics resources (body, wheels, joints, karosseries)
        void destroy();
    };
} // namespace simulator
