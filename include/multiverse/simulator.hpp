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
        std::unique_ptr<Robot> robot;

      private:
        void doit() {}
        std::shared_ptr<muli::World> physics_world;

      public:
        Simulator(std::shared_ptr<rerun::RecordingStream> rec);
        ~Simulator();
        void tick(float dt) {
            world->tick(dt);
            for (auto &robott : robots) {
                robott->tick(dt);
            }
        }
        void init(concord::Datum datum, mvs::Size world_size, mvs::Size grid_size) {
            world = std::make_shared<mvs::World>(rec);
            world->init(datum, world_size, grid_size);

            for (int i = 0; i < 4; ++i) {
                concord::Pose robot_pose;
                robot_pose.point.enu.x = 0;
                robot_pose.point.enu.y = 0;
                robot_pose.point.enu.z = 0;
                robot_pose.point.enu.toWGS(world->get_settings().get_datum());
                robots.emplace_back([&] {
                    auto r = std::make_unique<Robot>(rec, world->get_world());
                    r->init(robot_pose, "robot");
                    return r;
                }());
            }
        }
    };
} // namespace mvs
