// Simple single-process example: Simulator + Agent in same process
// Demonstrates high-level Agent API (set_velocity) with in-process Simulator

#include "flatsim/agent.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/agent/loader.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include "flatsim/utils.hpp"
#include <iostream>
#include "flatsim/utils.hpp"
#include <rerun.hpp>
#include "flatsim/utils.hpp"
#include <thread>
#include "flatsim/utils.hpp"

int main() {
    std::cout << "[Example] Simple demo - Simulator + Agent in same process" << std::endl;

    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(simulator::Conn::IPC, "", 100.0f, 100.0f, datum, rec);

    try {
        datapod::Pose spawn_pose = utils::make_pose_2d(0.0, 0.0, 0.0);
        auto machine = agent::Loader::load_from_json("examples/machines/tractor.json", spawn_pose);

        std::cout << "[Loader] Loaded: " << machine.name << std::endl;

        agent::Agent robot("");
        robot.set_machine(machine);
        robot.spawn();

        const float dt = 0.016f;
        std::cout << "[Example] Driving in circle using robot.set_velocity()" << std::endl;

        for (int i = 0; i < 500; ++i) {
            robot.set_velocity(0.5f, 0.2f);

            sim.tick(dt);
            robot.tick(dt, 100);

            if (i % 2 == 0) {
                sim.tock();
                robot.tock();
            }

            if (i % 60 == 0) {
                auto pose = robot.machine().world_pose();
                std::cout << "[Robot] Tick " << i << " - Pos: (" << pose.point.x << ", " << pose.point.y
                          << ") yaw=" << utils::get_yaw(pose) << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        robot.despawn();
    } catch (const std::exception &e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
    }

    std::cout << "[Example] Done!" << std::endl;
    return 0;
}
