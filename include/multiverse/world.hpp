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
        float grid_size;
    };

    class WorldSettings : public muli::WorldSettings {
      private:
        concord::Datum world_datum_;
        Size world_size_;

      public:
        WorldSettings(concord::Datum world_datum, Size world_size)
            : muli::WorldSettings(), world_datum_(world_datum), world_size_(world_size) {}
        void set_datum(const concord::Datum &datum);
        void set_size(const Size &size);
    };

    class World {
      private:
        WorldSettings settings;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::unique_ptr<muli::World> world;

      public:
        World(std::shared_ptr<rerun::RecordingStream> rec, WorldSettings settings);
        ~World();

        void init(WorldSettings settings);
        void tick(float dt);
    };
} // namespace mvs
