#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types.hpp"

#include <rerun.hpp>

namespace mvs {
    class Robot {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        muli::RigidBody *body;

      public:
        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world);
        ~Robot();

        void tick(float dt);
        void visualize_once();
        void visualize();

      private:
        std::vector<std::array<float, 3>> enu_corners_;
    };
} // namespace mvs
