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

    struct Square {
        float x_center;
        float y_center;
        float side;
    };

    class WorldSettings : public muli::WorldSettings {
      private:
        concord::Datum world_datum_;
        Size world_size_;

      public:
        WorldSettings(concord::Datum world_datum, Size world_size)
            : muli::WorldSettings(), world_datum_(world_datum), world_size_(world_size) {}
        concord::Datum get_datum() const { return world_datum_; }
        Size get_size() const { return world_size_; }
    };

    class World {
      private:
        WorldSettings settings;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<std::vector<Square>> grid;

      public:
        World(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum, Size size);
        ~World();

        const WorldSettings &get_settings() const { return settings; }
        void init(WorldSettings settings);
        void tick(float dt);
        void visualize_once();
        void visualize();

      private:
        std::vector<std::array<float, 3>> enu_corners_;
        std::vector<std::array<float, 3>> enu_grid_;
        std::vector<rerun::LatLon> wgs_corners_;
    };
} // namespace mvs
