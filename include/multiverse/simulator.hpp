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

      public:
        Simulator() {}
        ~Simulator() {}
        void tick(float dt) {}
    };
} // namespace mvs
