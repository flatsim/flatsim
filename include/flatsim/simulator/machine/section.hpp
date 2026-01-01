#pragma once

#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "pigment/pigment.hpp"
#include <rerun.hpp>

namespace simulator {
    class Section {
      private:
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
        int section_id;

        Section(std::shared_ptr<rerun::RecordingStream> rec, types::Machine *robot_info, types::State *robot_state);
        void init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                  datapod::Box section_bound, int id);
        void tick(float dt, datapod::Pose trans_pose);
        void tock();
        void teleport(datapod::Pose trans_pose);
        void toggle_work() { working = !working; }

        std::vector<datapod::Point> get_corners() const { return utils::get_corners(pose, bound.size); }
        datapod::Box get_bound() const { return bound; }
    };
} // namespace simulator
