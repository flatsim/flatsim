#pragma once

#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "flywheel/collision_filter.h"
#include "flywheel/world.h"
#include <rerun.hpp>

namespace simulator {
    class Hitch {
      private:
        std::shared_ptr<flywheel::World> world;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string parent_name;
        types::Machine *robot_info = nullptr;
        types::State *robot_state = nullptr;

      public:
        std::string name;
        datapod::Box bound;
        datapod::Pose pose;
        pigment::RGB color;
        bool hooked = false;
        bool is_master = true; // true = master (can pull), false = slave (can be pulled)

        Hitch(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<flywheel::World> world,
              types::Machine *robot_info, types::State *robot_state);
        void init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                  datapod::Box parent_bound, datapod::Box bound, flywheel::CollisionFilter filter, bool is_master);
        void tick(float dt, datapod::Pose trans_pose);
        void tock();

        void teleport(datapod::Pose pose);

        void toggle_hook() { hooked = !hooked; }
        std::vector<datapod::Point> get_corners() const { return utils::get_corners(pose, bound.size); }
        datapod::Box get_bound() const { return bound; }
    };
} // namespace simulator
