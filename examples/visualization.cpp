#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    std::cout << "[Example] Visualization demo with tick/tock pattern" << std::endl;

    std::atomic<bool> running{true};
    const int viz_fps = 30;
    const auto viz_interval = std::chrono::milliseconds(1000 / viz_fps);

    // Start simulator in main thread
    simulator::Simulator sim(simulator::Conn::IPC);

    // Create a test machine
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
    fl.color = pigment::RGB(50, 50, 50);
    fl.steering_max = 0.5f;
    fl.force = 200.0f;

    fr.name = "front_right";
    fr.pose.point.x = 0.4;
    fr.pose.point.y = 0.8;
    fr.size = concord::Size(0.1, 0.2, 0.0);
    fr.color = pigment::RGB(50, 50, 50);
    fr.steering_max = 0.5f;
    fr.force = 200.0f;

    rl.name = "rear_left";
    rl.pose.point.x = -0.4;
    rl.pose.point.y = -0.8;
    rl.size = concord::Size(0.1, 0.2, 0.0);
    rl.color = pigment::RGB(50, 50, 50);
    rl.steering_max = 0.0f;
    rl.force = 200.0f;

    rr.name = "rear_right";
    rr.pose.point.x = 0.4;
    rr.pose.point.y = -0.8;
    rr.size = concord::Size(0.1, 0.2, 0.0);
    rr.color = pigment::RGB(50, 50, 50);
    rr.steering_max = 0.0f;
    rr.force = 200.0f;

    machine.wheels = {fl, fr, rl, rr};

    sim.create_machine(machine);
    std::cout << "[Simulator] Created machine: " << machine.name << std::endl;

    // Background visualization thread - calls tock() at fixed rate
    std::thread viz_thread([&sim, &running, &viz_interval, &viz_fps]() {
        std::cout << "[VizThread] Started visualization thread at " << viz_fps << " FPS" << std::endl;
        while (running.load()) {
            auto viz_start = std::chrono::steady_clock::now();

            // Call tock for visualization
            sim.tock();

            // Maintain consistent frame rate
            auto viz_end = std::chrono::steady_clock::now();
            auto elapsed = viz_end - viz_start;
            if (elapsed < viz_interval) {
                std::this_thread::sleep_for(viz_interval - elapsed);
            }
        }
        std::cout << "[VizThread] Stopped" << std::endl;
    });

    // Main physics loop - tick at full speed
    std::cout << "[PhysicsThread] Starting physics loop" << std::endl;
    auto last_time = std::chrono::steady_clock::now();
    int tick_count = 0;
    const int max_ticks = 300; // Run for ~5 seconds at 60 FPS

    // Apply some control to make the robot move
    types::MachineControl ctrl;
    ctrl.uuid = machine.uuid;
    ctrl.steering = {0.1f, 0.1f, 0.0f, 0.0f}; // Slight turn
    ctrl.throttle = {0.3f, 0.3f, 0.3f, 0.3f}; // Forward

    while (tick_count < max_ticks) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt = now - last_time;
        last_time = now;

        // Apply control
        sim.apply_control(ctrl, dt.count());

        // Physics tick
        sim.tick(dt.count());

        tick_count++;

        // Print progress every second
        if (tick_count % 60 == 0) {
            auto state = sim.get_world_state();
            if (!state.machines.empty()) {
                auto &ms = state.machines[0];
                std::cout << "[Physics] Tick " << tick_count << " - Position: (" << ms.pose.position.x << ", "
                          << ms.pose.position.y << "), Velocity: (" << ms.velocity.x << ", " << ms.velocity.y << ")"
                          << std::endl;
            }
        }

        // Sleep to maintain ~60 FPS physics
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "[PhysicsThread] Stopping after " << tick_count << " ticks" << std::endl;

    // Stop visualization thread
    running.store(false);
    viz_thread.join();

    std::cout << "[Example] Done!" << std::endl;

    return 0;
}
