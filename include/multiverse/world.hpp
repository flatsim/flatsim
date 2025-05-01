#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types_basic.hpp"
#include "concord/types_square.hpp"

#include <rerun.hpp>

namespace mvs {
    using Size = concord::Size;
    using Square = concord::Square;
    typedef std::vector<std::vector<Square>> theGrid;

    class WorldSettings : public muli::WorldSettings {
      private:
        concord::Datum world_datum_;
        concord::Size world_size_;
        concord::Size grid_size_;

      public:
        WorldSettings(concord::Datum world_datum, Size world_size, Size grid_size)
            : muli::WorldSettings(), world_datum_(world_datum), world_size_(world_size), grid_size_(grid_size) {}
        concord::Datum get_datum() const { return world_datum_; }
        Size get_world_size() const { return world_size_; }
        Size get_grid_size() const { return grid_size_; }
    };

    class World {
      private:
        WorldSettings settings;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        theGrid grid;

      public:
        World(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum, Size world_size, Size grid_size);
        ~World();

        const WorldSettings &get_settings() const { return settings; }
        const theGrid &get_grid() const { return grid; }
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
