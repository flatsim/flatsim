#pragma once

#include "multiverse/robot.hpp"
#include "multiverse/types.hpp"
#include "multiverse/world.hpp"

#include "pigment/types_basic.hpp"

#include "muli/world.h"
#include <rerun.hpp>

namespace mvs {
    class Simulator {
      public:
        std::shared_ptr<World> world;
        concord::Datum world_datum;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::vector<std::shared_ptr<Robot>> robots;

      private:
        bool controls_set = false;
        std::shared_ptr<muli::World> physics_world;
        int selected_robot_idx = -1;

      public:
        Simulator(std::shared_ptr<rerun::RecordingStream> rec);
        ~Simulator();
        void tick(float dt);
        void init(concord::Datum datum, concord::Size world_size, float grid_size);
        concord::Datum get_datum() const { return world_datum; }
        Robot &get_robot(uint i) { return *robots[i]; }
        int num_robots() const { return robots.size(); }

        void add_robot(concord::Pose robot_pose, pigment::RGB robot_color, concord::Size chassis_size, std::string name,
                       std::string uuid, std::vector<concord::Bound> wheels,
                       std::pair<std::vector<float>, std::vector<float>> controls,
                       std::vector<concord::Bound> karosserie);
        void add_robot(Robo robot_info);
        void set_controls(uint robot_idx, float steering, float throttle);
    };
} // namespace mvs
