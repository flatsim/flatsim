#include <chrono>
#include <iostream>
#include <thread>

#include "multiverse/simulator.hpp"
// #include "multiverse/driver/controller.hpp"
#include "multiverse/world.hpp"
#include "rerun/recording_stream.hpp"
#include <rerun.hpp>

#include "rerun/archetypes/line_strips3d.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("multiverse", "space");
    auto rec_running = rec->connect_grpc("rerun+http://172.30.0.1:9876/proxy");
    if (rec_running.is_err()) {
        std::cout << "Failed to connect to rerun" << std::endl;
        return 1;
    }

    concord::Datum world_datum;
    world_datum.lat = 51.987305;
    world_datum.lon = 5.663625;
    world_datum.alt = 53.801823;

    mvs::Size world_size;
    world_size.x = 100.0f;
    world_size.y = 100.0f;
    world_size.z = 100.0f;

    mvs::Size grid_size;
    grid_size.x = 3.0f;
    grid_size.y = 3.0f;
    grid_size.z = 3.0f;

    // auto sim = std::make_shared<mvs::World>(rec);
    auto sim = std::make_shared<mvs::Simulator>(rec);
    sim->init(world_datum, world_size, grid_size);

    // muli::WorldSettings settings;
    // muli::World world(settings);

    // mvs::vehicle::Vehicle chassis(sim->world->get_world().get(), concord::Pose());

    auto last_time = std::chrono::steady_clock::now();
    std::cout << "Running...\n";
    while (true) {
        // Current time and delta
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = now - last_time;
        last_time = now;
        float dt = elapsed.count(); // seconds since last frame
        // Tick with “actual” dt
        // chassis.tick(dt);
        sim->tick(dt);
        // (Optional) tiny sleep so you don't spin at full CPU—
        // adjust or remove if you want completely time-driven stepping
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
