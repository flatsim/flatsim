#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <thread>

int main() {
    std::cout << "[Example] Visualization demo with tick/tock pattern + Rerun" << std::endl;

    // Setup Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started" << std::endl;

    std::atomic<bool> running{true};
    const int viz_fps = 30;
    const auto viz_interval = std::chrono::milliseconds(1000 / viz_fps);

    // Start simulator in a thread
    std::thread sim_thread([&rec, &running]() {
        simulator::WorldSettings ws{100.0f, 100.0f};
        simulator::Simulator sim(simulator::Conn::IPC, "", ws, rec);

        std::cout << "[SimThread] Starting sim loop for 500 ticks..." << std::endl;
        for (int i = 0; i < 500 && running.load(); ++i) {
            if (i % 60 == 0) {
                std::cout << "[SimThread] Loop iteration " << i << std::endl;
            }

            std::cout << "[SimThread] Before tick " << i << std::endl;
            sim.tick(0.016f);
            std::cout << "[SimThread] After tick " << i << std::endl;

            // Call tock every few ticks for visualization (30 FPS = every 2 ticks at 60 FPS)
            if (i % 2 == 0) {
                std::cout << "[SimThread] Before tock " << i << std::endl;
                sim.tock();
                std::cout << "[SimThread] After tock " << i << std::endl;
            }

            std::cout << "[SimThread] Before sleep " << i << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            std::cout << "[SimThread] After sleep " << i << std::endl;
        }
        std::cout << "[SimThread] Sim loop ended" << std::endl;
    });

    // Give simulator time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Create agent with rerun
    agent::Agent agnt("", rec);

    // Create a simple 4-wheel machine
    types::Machine machine;
    machine.uuid = "robot_001";
    machine.name = "VisBot";
    machine.pose.point.x = 0.0;
    machine.pose.point.y = 0.0;
    machine.pose.angle.yaw = 0.0;
    machine.size = concord::Size(1.0, 2.0, 0.0);
    machine.color = pigment::RGB(0, 255, 0); // Green

    // Add 4 wheels
    types::Wheel fl, fr, rl, rr;

    fl.name = "front_left";
    fl.pose.point.x = -0.4;
    fl.pose.point.y = 0.8;
    fl.size = concord::Size(0.1, 0.2, 0.0);
    // fl.color defaults to (0,0,0) which will inherit machine color
    fl.steering_max = 0.5f;
    fl.force = 200.0f;

    fr.name = "front_right";
    fr.pose.point.x = 0.4;
    fr.pose.point.y = 0.8;
    fr.size = concord::Size(0.1, 0.2, 0.0);
    // fr.color defaults to (0,0,0) which will inherit machine color
    fr.steering_max = 0.5f;
    fr.force = 200.0f;

    rl.name = "rear_left";
    rl.pose.point.x = -0.4;
    rl.pose.point.y = -0.8;
    rl.size = concord::Size(0.1, 0.2, 0.0);
    // rl.color defaults to (0,0,0) which will inherit machine color
    rl.steering_max = 0.0f;
    rl.force = 200.0f;

    rr.name = "rear_right";
    rr.pose.point.x = 0.4;
    rr.pose.point.y = -0.8;
    rr.size = concord::Size(0.1, 0.2, 0.0);
    // rr.color defaults to (0,0,0) which will inherit machine color
    rr.steering_max = 0.0f;
    rr.force = 200.0f;

    machine.wheels = {fl, fr, rl, rr};

    agnt.set_machine(machine);

    if (agnt.spawn()) {
        std::cout << "[Agent] Spawn successful!" << std::endl;

        // Background visualization thread - calls tock() at fixed rate
        std::thread viz_thread([&agnt, &running, &viz_interval, &viz_fps]() {
            std::cout << "[VizThread] Started visualization thread at " << viz_fps << " FPS" << std::endl;
            int frame_count = 0;
            while (running.load()) {
                auto viz_start = std::chrono::steady_clock::now();

                // Call tock for agent visualization
                agnt.tock();

                // Debug: Print position every 30 frames (once per second)
                if (frame_count % 30 == 0) {
                    auto pose = agnt.machine().world_pose();
                    std::cout << "[VizThread] Frame " << frame_count << " - Robot at: (" << pose.point.x << ", "
                              << pose.point.y << ")" << std::endl;
                }
                frame_count++;

                // Maintain consistent frame rate
                auto viz_end = std::chrono::steady_clock::now();
                auto elapsed = viz_end - viz_start;
                if (elapsed < viz_interval) {
                    std::this_thread::sleep_for(viz_interval - elapsed);
                }
            }
            std::cout << "[VizThread] Stopped" << std::endl;
        });

        // Send control commands
        std::cout << "[Example] Starting control loop..." << std::endl;
        for (int i = 0; i < 250; ++i) {
            types::MachineControl ctrl;
            ctrl.uuid = machine.uuid;
            ctrl.steering = {0.1f, 0.1f, 0.0f, 0.0f}; // Slight turn
            ctrl.throttle = {0.5f, 0.5f, 0.5f, 0.5f}; // Forward

            bool success = agnt.control(ctrl);
            if (i % 60 == 0) {
                std::cout << "[Example] Control " << i << " - Success: " << success << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        std::cout << "[Example] Final pose: (" << agnt.machine().world_pose().point.x << ", "
                  << agnt.machine().world_pose().point.y << ")" << std::endl;

        running.store(false);
        viz_thread.join();
    } else {
        std::cout << "[Agent] Spawn failed!" << std::endl;
        running.store(false);
    }

    sim_thread.join();
    std::cout << "[Example] Done!" << std::endl;

    return 0;
}
