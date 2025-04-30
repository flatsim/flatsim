#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types.hpp"

namespace mv {

    class World {
      private:
        std::unique_ptr<muli::World> world;
        concord::WGS world_datum;

      public:
        World();
        ~World();

        void init();
        void tick();
    };
} // namespace mv
