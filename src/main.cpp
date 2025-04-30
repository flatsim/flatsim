#include "multiverse/world.hpp"

#include "rerun/recording_stream.hpp"
#include <rerun.hpp>

int main() {
    mvs::World world;
    mvs::WorldSettings settings;

    while (true) {
        world.tick();
    }
}
