#include "multiverse/world.hpp"

namespace mv {
    World::World() {
        muli::WorldSettings settings;
        world = std::make_unique<muli::World>(settings);
    }

    World::~World() { world.reset(); }

    void World::init(concord::Datum datum, Size size, Grid grid) {
        world_datum = datum;
        world_size = size;
        world_grid = grid;
    }

    void World::tick() {}
} // namespace mv
