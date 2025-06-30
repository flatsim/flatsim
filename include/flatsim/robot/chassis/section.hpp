#pragma once

#include "concord/concord.hpp"
#include "pigment/pigment.hpp"
#include "rerun.hpp"

namespace fs {
    class Section {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::string parent_name;

      public:
        std::string name;
        concord::Bound bound;
        concord::Pose pose;
        pigment::RGB color;
        bool working = false;
        int section_id;

        Section(std::shared_ptr<rerun::RecordingStream> rec);
        void init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                  concord::Bound section_bound, int id);
        void tick(float dt, concord::Pose trans_pose);
        void tock();
        void teleport(concord::Pose trans_pose);
        void toggle_work() { working = !working; }

        std::vector<concord::Point> get_corners() const { return pose.get_corners(bound.size); }
        concord::Bound get_bound() const { return bound; }
    };
} // namespace fs
