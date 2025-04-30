#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types.hpp"

#include <rerun.hpp>

namespace mvs {

    struct Size {
        float width;
        float height;
    };

    struct Grid {
        float col;
        float row;
    };

    class WorldSettings : public muli::WorldSettings {
      private:
        concord::Datum world_datum;
        Size world_size;
        Grid world_grid;

      public:
        WorldSettings() : muli::WorldSettings() {}
        void set_datum(const concord::Datum &datum);
        void set_size(const Size &size);
        void set_grid(const Grid &grid);
    };

    class World {
      private:
        WorldSettings settings;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::unique_ptr<muli::World> world;

      public:
        World(std::shared_ptr<rerun::RecordingStream> rec);
        ~World();

        void init(WorldSettings settings);
        void tick(float dt);
    };
} // namespace mvs
