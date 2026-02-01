// Test camera tracking - demonstrates automatic camera following of last spawned agent
// This example spawns multiple agents and shows how the camera tracks the last one

#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <thread>

int main() {
    std::cout << "[Example] Camera Tracking Test - Last spawned agent is tracked" << std::endl;

    // Create Rerun visualization
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim_camera_tracking", "space");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started - camera will track the last spawned agent" << std::endl;

    // Create simulator in LOCAL mode
    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(200.0f, 200.0f, datum, rec);

    try {
        // Spawn first agent at (0, 0)
        std::cout << "\n[Spawn] Agent 1 at (0, 0) - Camera should track this" << std::endl;
        datapod::Pose spawn_pose1 = utils::make_pose_2d(0.0, 0.0, 0.0);
        auto &agent1 = sim.spawn_agent("examples/machines/urdf/tractor.urdf", spawn_pose1);
        agent1.set_velocity(0.5f, 0.0f); // Move forward

        // Run for a bit
        const float dt = 0.016f;
        for (int i = 0; i < 100; ++i) {
            sim.tick(dt);
            if (i % 2 == 0) {
                sim.tock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        // Spawn second agent at (20, 20)
        std::cout << "\n[Spawn] Agent 2 at (20, 20) - Camera should now track this one!" << std::endl;
        datapod::Pose spawn_pose2 = utils::make_pose_2d(20.0, 20.0, 1.57); // 90 degrees
        auto &agent2 = sim.spawn_agent("examples/machines/urdf/tractor.urdf", spawn_pose2);
        agent2.set_velocity(0.3f, 0.1f); // Move in a curve

        // Run for a bit
        for (int i = 0; i < 100; ++i) {
            sim.tick(dt);
            if (i % 2 == 0) {
                sim.tock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        // Spawn third agent at (-20, -20)
        std::cout << "\n[Spawn] Agent 3 at (-20, -20) - Camera should now track this one!" << std::endl;
        datapod::Pose spawn_pose3 = utils::make_pose_2d(-20.0, -20.0, 3.14); // 180 degrees
        auto &agent3 = sim.spawn_agent("examples/machines/urdf/tractor.urdf", spawn_pose3);
        agent3.set_velocity(0.4f, -0.1f); // Move in opposite curve

        // Run simulation with all three agents
        std::cout << "\n[Running] All three agents active, camera tracking agent 3" << std::endl;
        for (int i = 0; i < 500; ++i) {
            sim.tick(dt);
            if (i % 2 == 0) {
                sim.tock();
            }

            if (i % 60 == 0) {
                auto pose1 = agent1.machine().world_pose();
                auto pose2 = agent2.machine().world_pose();
                auto pose3 = agent3.machine().world_pose();
                std::cout << "[Tick " << i << "]" << std::endl;
                std::cout << "  Agent 1: (" << pose1.point.x << ", " << pose1.point.y << ")" << std::endl;
                std::cout << "  Agent 2: (" << pose2.point.x << ", " << pose2.point.y << ")" << std::endl;
                std::cout << "  Agent 3: (" << pose3.point.x << ", " << pose3.point.y << ") <- TRACKED" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

    } catch (const std::exception &e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n[Example] Done! The camera should have been tracking the last spawned agent." << std::endl;
    std::cout << "[Info] Check the Rerun viewer to verify the 3D view followed agent 3." << std::endl;
    return 0;
}
