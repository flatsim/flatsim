#pragma once

#include "concord/types_basic.hpp"
#include "muli/collision_filter.h"
#include "muli/world.h"
#include "multiverse/utils.hpp"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

namespace mvs {
    class Karosserie {
      private:
        std::shared_ptr<muli::World> world;
        muli::RigidBody *karosserie;
        muli::RigidBody *parent;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string parent_name;

      public:
        std::string name;
        concord::Bound bound;
        concord::Pose pose;
        pigment::RGB color;
        bool working = false;
        bool has_physics = true;

        Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world);
        void init(const pigment::RGB &color, std::string parent_name, std::string name, concord::Bound parent_bound,
                  concord::Bound bound, muli::CollisionFilter filter, bool has_physics = true);
        void tick(float dt, concord::Pose trans_pose);

        muli::Transform get_transform() const;
        muli::RigidBody *get_body() const;
        void teleport(concord::Pose pose);
        void visualize();

        void toggle_work() { working = !working; }
        std::vector<concord::Point> get_corners() { return pose.get_corners(bound.size); }
        concord::Bound get_bound() const { return bound; }
    };
} // namespace mvs
