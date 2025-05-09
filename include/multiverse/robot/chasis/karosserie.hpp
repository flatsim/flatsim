#pragma once

#include "concord/types_basic.hpp"
#include "muli/collision_filter.h"
#include "muli/world.h"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

namespace mvs {
    class Karosserie {
      private:
        std::shared_ptr<muli::World> world;
        concord::Bound bound;
        muli::RigidBody *karosserie;
        muli::RigidBody *parent;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string name;
        pigment::RGB color;

      public:
        Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world);

        void init(concord::Bound parent, concord::Bound bound, muli::CollisionFilter filter, pigment::RGB color,
                  std::string name);
        void tick(float dt, muli::Transform t);

        muli::Transform shift(concord::Bound parent, concord::Bound child);
        void teleport(concord::Pose pose);

        void visualize();
    };
} // namespace mvs
