#pragma once

#include "concord/types_basic.hpp"
#include "concord/types_grid.hpp"
#include "concord/types_square.hpp"
#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

#include "multiverse/robot.hpp"
#include "multiverse/world/layer.hpp"

namespace mvs {
    class WorldSettings : public muli::WorldSettings {
      private:
        concord::Datum world_datum_;
        concord::Size world_size_;
        float grid_size_;

      public:
        WorldSettings() = default;
        void init(concord::Datum datum, concord::Size world_size, float grid_size) {
            world_datum_ = datum;
            world_size_ = world_size;
            grid_size_ = grid_size;
        }
        concord::Datum get_datum() const { return world_datum_; }
        concord::Size get_world_size() const { return world_size_; }
        float get_grid_size() const { return grid_size_; }
    };

    class World {
      private:
        WorldSettings settings;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        Layer<pigment::RGB> grid;
        std::vector<std::shared_ptr<Layer<pigment::RGB>>> layers;

      public:
        World(std::shared_ptr<rerun::RecordingStream> rec);
        ~World();

        const WorldSettings &get_settings() const { return settings; }
        // get world shared pointer
        std::shared_ptr<muli::World> get_world() const { return world; }
        void init(concord::Datum datum, concord::Size world_size, float grid_size);
        void tick(float dt);
        void visualize();
        void add_layer(std::string name, float inradius);

      private:
        std::vector<std::array<float, 3>> enu_corners_;
        std::vector<rerun::LatLon> wgs_corners_;
    };
} // namespace mvs
