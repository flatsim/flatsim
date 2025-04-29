#include "multiverse/world.hpp"

namespace mv {
    World::World() {
        muli::WorldSettings settings;
        world = std::make_unique<muli::World>(settings);
    }

    World::~World() { world.reset(); }

    void World::init() {}

    void World::tick() {}
} // namespace mv
