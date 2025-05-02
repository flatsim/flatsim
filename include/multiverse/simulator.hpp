#pragma once

#include "multiverse/robot.hpp"
#include "multiverse/world.hpp"

#include "muli/world.h"
#include <rerun.hpp>

namespace mvs {
    class Simulator {
      public:
        std::shared_ptr<World> world;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::vector<std::unique_ptr<Robot>> robots;

      private:
        void doit() {}
        std::shared_ptr<muli::World> physics_world;

      public:
        Simulator(std::shared_ptr<rerun::RecordingStream> rec);
        ~Simulator();
        void tick(float dt) {
            world->tick(dt);
            for (auto &robot : robots) {
                robot->tick(dt);
            }
        }
        void init(concord::Datum datum, mvs::Size world_size, mvs::Size grid_size) {
            world = std::make_shared<mvs::World>(rec);
            world->init(datum, world_size, grid_size);

            Robot robot(rec, world->get_world());
            concord::Pose robot_pose;
            robot_pose.point.enu.x = 0;
            robot_pose.point.enu.y = 0;
            robot_pose.point.enu.z = 0;
            robot_pose.point.enu.toWGS(world->get_settings().get_datum());
            robot.init(robot_pose, "robot");
            robots.push_back(std::make_unique<Robot>(rec, world->get_world()));
        }
    };
} // namespace mvs
