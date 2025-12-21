// Simple simulator-only example (no agent, no network communication)
// This demonstrates direct simulator API usage for testing/debugging
// For production use, see simulator_server.cpp and agent_client.cpp

#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    std::cout << "[Example] Simple simulator-only demo (direct API, no agent)" << std::endl;

    simulator::SimulatorSettings ws{100.0f, 100.0f};
    simulator::Simulator sim(simulator::Conn::IPC, "", ws, nullptr);

    // Create a simple 4-wheel machine (no IPC/Agent involved)
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

    sim.create_machine(machine);

    const float dt = 0.016f;
    for (int i = 0; i < 200; ++i) {
        types::WheelControl ctrl;
        ctrl.uuid = machine.uuid;
        ctrl.steering = {0.1f, 0.1f, 0.0f, 0.0f}; // Slight turn
        ctrl.throttle = {0.6f, 0.6f, 0.6f, 0.6f}; // Forward

        sim.apply_control(ctrl, dt);
        sim.tick(dt);

        // Call tock occasionally; in this example `rec==nullptr` so this is a no-op.
        if (i % 2 == 0) {
            sim.tock();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    auto world_state = sim.get_world_state();
    for (const auto &ms : world_state.machines) {
        if (std::string(ms.uuid.view()) == machine.uuid) {
            std::cout << "Final pose: (" << ms.pose.position.x << ", " << ms.pose.position.y
                      << ") yaw=" << ms.pose.angle << std::endl;
        }
    }

    return 0;
}
