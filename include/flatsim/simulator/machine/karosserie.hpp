#pragma once

#include <vector>

#include "flatsim/simulator/machine/section.hpp"
#include "flatsim/utils.hpp"
#include "flywheel/flywheel.hpp"
#include <rerun.hpp>

namespace simulator {
    class Karosserie {
      private:
        std::shared_ptr<flywheel::World> world;
        flywheel::RigidBody *karosserie;
        flywheel::RigidBody *parent;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string parent_name;
        types::Machine *robot_info = nullptr;
        types::State *robot_state = nullptr;

      public:
        std::string name;
        datapod::Box bound;
        datapod::Pose pose;
        pigment::RGB color;
        bool working = false;
        bool has_physics = true;
        std::vector<Section> sections;

        Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<flywheel::World> world,
                   types::Machine *robot_info, types::State *robot_state);

        void init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                  datapod::Box parent_bound, datapod::Box bound, flywheel::CollisionFilter filter, int num_sections,
                  bool has_physics);
        void tick(float dt, datapod::Pose trans_pose);
        void tock();

        flywheel::Transform get_transform() const;
        flywheel::RigidBody *get_body() const;
        void teleport(datapod::Pose pose);

        void toggle_section_work(int section_id);
        void toggle_all_sections_work();
        void toggle_all_except_section_work(int except_section_id);
        std::vector<datapod::Point> get_corners() const { return utils::get_corners(pose, bound.size); }
        datapod::Box get_bound() const { return bound; }
        void update_color(const pigment::RGB &new_color) { color = new_color; }

        // Destroy physics resources
        void destroy();
    };
} // namespace simulator
