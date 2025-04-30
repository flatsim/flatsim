#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types.hpp"

namespace mv {

    struct Size {
        float width;
        float height;
    };

    struct Grid {
        float col;
        float row;
    };

    class World {
      private:
        std::unique_ptr<muli::World> world;
        concord::Datum world_datum;
        Size world_size;
        Grid world_grid;

      public:
        World();
        ~World();

        void init(concord::Datum datum, Size size, Grid grid);
        void tick();
    };
} // namespace mv
