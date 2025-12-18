#pragma once

#include <vector>

#include "flatsim/simulator/machine/section.hpp"
#include "flatsim/utils.hpp"
#include "muli/collision_filter.h"
#include "muli/world.h"
#include <rerun.hpp>

namespace simulator {
    class Karosserie {
      private:
        std::shared_ptr<muli::World> world;
        muli::RigidBody *karosserie;
        muli::RigidBody *parent;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string parent_name;
        types::Machine *robot_info = nullptr;
        types::State *robot_state = nullptr;

      public:
        std::string name;
        concord::Bound bound;
        concord::Pose pose;
        pigment::RGB color;
        bool working = false;
        bool has_physics = true;
        std::vector<Section> sections;

        Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world,
                   types::Machine *robot_info, types::State *robot_state);

        void init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                  concord::Bound parent_bound, concord::Bound bound, muli::CollisionFilter filter, int num_sections,
                  bool has_physics);
        void tick(float dt, concord::Pose trans_pose);
        void tock();

        muli::Transform get_transform() const;
        muli::RigidBody *get_body() const;
        void teleport(concord::Pose pose);

        void toggle_section_work(int section_id);
        void toggle_all_sections_work();
        void toggle_all_except_section_work(int except_section_id);
        std::vector<concord::Point> get_corners() const { return pose.get_corners(bound.size); }
        concord::Bound get_bound() const { return bound; }
        void update_color(const pigment::RGB &new_color) { color = new_color; }
    };
} // namespace simulator
