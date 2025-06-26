#pragma once

#include "muli/collision_filter.h"
#include "muli/world.h"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/robot/chassis/section.hpp"

namespace fs {
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
        std::vector<Section> sections;

        Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world);
        void init(const pigment::RGB &color, const std::string& parent_name, const std::string& name, concord::Bound parent_bound,
                  concord::Bound bound, muli::CollisionFilter filter, int num_sections = 0, bool has_physics = true);
        void tick(float dt, concord::Pose trans_pose);

        muli::Transform get_transform() const;
        muli::RigidBody *get_body() const;
        void teleport(concord::Pose pose);
        void visualize();

        void toggle_section_work(int section_id);
        void toggle_all_sections_work();
        void toggle_all_except_section_work(int except_section_id);
        std::vector<concord::Point> get_corners() const { return pose.get_corners(bound.size); }
        concord::Bound get_bound() const { return bound; }
    };
} // namespace fs
