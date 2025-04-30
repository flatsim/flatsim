#include "multiverse/world.hpp"

#include "rerun/recording_stream.hpp"
#include <rerun.hpp>

#include "rerun/archetypes/line_strips3d.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("multiverse", "space");
    auto rec_running = rec->connect_grpc("rerun+http://127.0.0.1:9876");

    concord::Datum world_datum;
    world_datum.lat = 51.987305;
    world_datum.lon = 5.663625;
    world_datum.alt = 53.801823;

    mvs::Size world_size;
    world_size.width = 100.0f;
    world_size.height = 100.0f;
    world_size.grid_size = 1.0f;

    mvs::WorldSettings settings(world_datum, world_size);
    mvs::World world(rec, settings);

    float dt = 1.0f / 60.0f;
    while (true) {
        world.tick(dt);
        std::cout << "tick" << std::endl;
    }
}
