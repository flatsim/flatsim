#include "multiverse/world.hpp"

#include "rerun/recording_stream.hpp"
#include <rerun.hpp>

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("multiverse", "space");
    mvs::World world(rec);
    mvs::WorldSettings settings;

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) {
        world.tick(dt);
    }
}
