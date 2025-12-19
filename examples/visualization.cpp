#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <thread>

int main() {
    std::cout << "[Example] Visualization demo (single-thread tick()/tock() + Rerun)" << std::endl;

    // Setup Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started" << std::endl;

    simulator::WorldSettings ws{100.0f, 100.0f};
    simulator::Simulator sim(simulator::Conn::IPC, "", ws, rec);

    // Create a simple 4-wheel machine
    types::Machine machine;
    machine.uuid = "robot_001";
    machine.name = "VisBot";
    machine.bound.pose.point.x = 0.0;
    machine.bound.pose.point.y = 0.0;
    machine.bound.pose.angle.yaw = 0.0;
    machine.bound.size = concord::Size(1.0, 2.0, 0.0);
    machine.color = pigment::RGB(0, 255, 0); // Green

    // Add 4 wheels
    types::Wheel fl, fr, rl, rr;

    fl.name = "front_left";
    fl.bound.pose.point.x = -0.4;
    fl.bound.pose.point.y = 0.8;
    fl.bound.size = concord::Size(0.1, 0.2, 0.0);
    // fl.color defaults to (0,0,0) which will inherit machine color
    fl.steering_max = 0.5f;
    fl.force = 200.0f;

    fr.name = "front_right";
    fr.bound.pose.point.x = 0.4;
    fr.bound.pose.point.y = 0.8;
    fr.bound.size = concord::Size(0.1, 0.2, 0.0);
    // fr.color defaults to (0,0,0) which will inherit machine color
    fr.steering_max = 0.5f;
    fr.force = 200.0f;

    rl.name = "rear_left";
    rl.bound.pose.point.x = -0.4;
    rl.bound.pose.point.y = -0.8;
    rl.bound.size = concord::Size(0.1, 0.2, 0.0);
    // rl.color defaults to (0,0,0) which will inherit machine color
    rl.steering_max = 0.0f;
    rl.force = 200.0f;

    rr.name = "rear_right";
    rr.bound.pose.point.x = 0.4;
    rr.bound.pose.point.y = -0.8;
    rr.bound.size = concord::Size(0.1, 0.2, 0.0);
    // rr.color defaults to (0,0,0) which will inherit machine color
    rr.steering_max = 0.0f;
    rr.force = 200.0f;

    machine.wheels = {fl, fr, rl, rr};

    sim.create_machine(machine);

    const float dt = 0.016f;
    std::cout << "[Example] Driving in a circle (tick at ~60Hz, tock at ~30Hz)..." << std::endl;
    for (int i = 0; i < 500; ++i) {
        types::WheelControl ctrl;
        ctrl.uuid = machine.uuid;
        ctrl.steering = {0.3f, 0.3f, 0.0f, 0.0f};
        ctrl.throttle = {0.6f, 0.6f, 0.6f, 0.6f};

        sim.apply_control(ctrl, dt);
        sim.tick(dt);

        if (i % 2 == 0) {
            sim.tock();
        }

        if (i % 60 == 0) {
            auto ws_state = sim.get_world_state();
            for (const auto &ms : ws_state.machines) {
                if (std::string(ms.uuid.view()) == machine.uuid) {
                    std::cout << "[Sim] Tick " << i << " - Pose: (" << ms.pose.position.x << ", " << ms.pose.position.y
                              << ") yaw=" << ms.pose.angle << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "[Example] Done!" << std::endl;

    return 0;
}
