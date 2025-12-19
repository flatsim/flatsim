#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    // Start simulator in a thread
    std::thread sim_thread([]() {
        simulator::Simulator sim(simulator::Conn::IPC);

        for (int i = 0; i < 100; ++i) {
            sim.tick(0.016f);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    });

    // Give simulator time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Create agent and send machine
    agent::Agent agent;

    // Create a simple 4-wheel machine
    types::Machine machine;
    machine.uuid = "robot_001";
    machine.name = "TestBot";
    machine.bound.pose.point.x = 0.0;
    machine.bound.pose.point.y = 0.0;
    machine.bound.pose.angle.yaw = 0.0;
    machine.bound.size = concord::Size(1.0, 2.0, 0.0);
    machine.color = pigment::RGB(255, 0, 0);

    // Add 4 wheels (front steerable, rear fixed)
    types::Wheel fl, fr, rl, rr;

    fl.name = "front_left";
    fl.bound.pose.point.x = -0.4;
    fl.bound.pose.point.y = 0.8;
    fl.bound.size = concord::Size(0.1, 0.2, 0.0);
    fl.color = pigment::RGB(50, 50, 50);
    fl.steering_max = 0.5f; // Can steer
    fl.force = 200.0f;

    fr.name = "front_right";
    fr.bound.pose.point.x = 0.4;
    fr.bound.pose.point.y = 0.8;
    fr.bound.size = concord::Size(0.1, 0.2, 0.0);
    fr.color = pigment::RGB(50, 50, 50);
    fr.steering_max = 0.5f;
    fr.force = 200.0f;

    rl.name = "rear_left";
    rl.bound.pose.point.x = -0.4;
    rl.bound.pose.point.y = -0.8;
    rl.bound.size = concord::Size(0.1, 0.2, 0.0);
    rl.color = pigment::RGB(50, 50, 50);
    rl.steering_max = 0.0f; // Fixed
    rl.force = 200.0f;

    rr.name = "rear_right";
    rr.bound.pose.point.x = 0.4;
    rr.bound.pose.point.y = -0.8;
    rr.bound.size = concord::Size(0.1, 0.2, 0.0);
    rr.color = pigment::RGB(50, 50, 50);
    rr.steering_max = 0.0f;
    rr.force = 200.0f;

    machine.wheels = {fl, fr, rl, rr};

    agent.set_machine(machine);

    if (agent.spawn()) {
        std::cout << "Spawn successful!" << std::endl;

        // Send some control commands
        for (int i = 0; i < 50; ++i) {
            types::WheelControl ctrl;
            ctrl.uuid = machine.uuid;
            ctrl.steering = {0.1f, 0.1f, 0.0f, 0.0f}; // Slight turn
            ctrl.throttle = {0.5f, 0.5f, 0.5f, 0.5f}; // Forward

            agent.control(ctrl);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        std::cout << "Final pose: (" << agent.machine().world_pose().point.x << ", "
                  << agent.machine().world_pose().point.y << ")" << std::endl;
    } else {
        std::cout << "Spawn failed!" << std::endl;
    }

    sim_thread.join();

    return 0;
}
