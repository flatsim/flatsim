#pragma once

#include "multiverse/robot.hpp"
#include "multiverse/world.hpp"

#include "pigment/types_basic.hpp"

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
        void tick(float dt);
        void init(concord::Datum datum, mvs::Size world_size, mvs::Size grid_size);
        void on_key(char key);
    };
} // namespace mvs
