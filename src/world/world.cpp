#include "multiverse/world.hpp"

namespace mvs {
    World::World(rerun::RecordingStream &rec) : rec(&rec) {}
    World::~World() { world.reset(); }

    void World::init(WorldSettings settings) {
        settings.apply_gravity = false;
        world = std::make_unique<muli::World>(settings);
    }

    void World::tick() {}
} // namespace mvs
