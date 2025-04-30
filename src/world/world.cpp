#include "multiverse/world.hpp"

namespace mvs {
    World::World(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}
    World::~World() { world.reset(); }

    void World::init(WorldSettings settings) {
        settings.apply_gravity = false;
        world = std::make_unique<muli::World>(settings);
    }

    void World::tick(float dt) { world->Step(dt); }
} // namespace mvs
