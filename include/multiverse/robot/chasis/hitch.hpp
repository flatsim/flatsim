#pragma once

#include "muli/collision_filter.h"
#include "muli/world.h"
#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"

namespace mvs {
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

        Hitch(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world);
        void init(const pigment::RGB &color, std::string parent_name, std::string name, concord::Bound parent_bound,
                  concord::Bound bound, muli::CollisionFilter filter);
        void tick(float dt, concord::Pose trans_pose);

        void teleport(concord::Pose pose);
        void visualize();

        void toggle_hook() { hooked = !hooked; }
        std::vector<concord::Point> get_corners() { return pose.get_corners(bound.size); }
        concord::Bound get_bound() const { return bound; }
    };
} // namespace mvs
