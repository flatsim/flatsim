#pragma once

#include "multiverse/robot.hpp"
#include "multiverse/types.hpp"
#include "multiverse/world.hpp"

#include "muli/world.h"
#include <optional>

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

        void init(concord::Datum datum, concord::Size world_size);
        void tick(float dt);

        // ROBOT
        void add_robot(RobotInfo robot_info);
        void set_controls(uint robot_idx, float steering, float throttle);
        void toggle_work(uint robot_idx, std::string karosserie_name);
        Robot &get_robot(uint i);
        Robot &get_robot(std::string uuid);
        int num_robots() const;
        // WORLD
        concord::Datum get_datum() const;
        World &get_world() { return *world; }
        Layer &get_layer(uint i);
        Layer &get_layer(std::string uuid);
        void add_layer(LayerInfo layz, bool noise = false);
    };
} // namespace mvs
