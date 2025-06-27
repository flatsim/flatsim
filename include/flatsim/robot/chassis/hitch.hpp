#pragma once

#include "muli/collision_filter.h"
#include "muli/world.h"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"

namespace fs {
    class Hitch {
      private:
        std::shared_ptr<muli::World> world;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string parent_name;

      public:
        std::string name;
        concord::Bound bound;
        concord::Pose pose;
        pigment::RGB color;
        bool hooked = false;
        bool is_master = true;  // true = master (can pull), false = slave (can be pulled)

        Hitch(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world);
        void init(const pigment::RGB &color, const std::string& parent_name, const std::string& name, concord::Bound parent_bound,
                  concord::Bound bound, muli::CollisionFilter filter, bool is_master = true);
        void tick(float dt, concord::Pose trans_pose);

        void teleport(concord::Pose pose);
        void visualize();

        void toggle_hook() { hooked = !hooked; }
        std::vector<concord::Point> get_corners() const { return pose.get_corners(bound.size); }
        concord::Bound get_bound() const { return bound; }
    };
} // namespace fs
