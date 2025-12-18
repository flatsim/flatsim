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

    // Create agent and send chassis
    agent::Agent agent;

    types::Chassis chassis;
    chassis.uuid = "robot_001";
    chassis.name = "TestBot";
    chassis.pose.point.x = 0.0;
    chassis.pose.point.y = 0.0;
    chassis.pose.angle.yaw = 0.0;
    chassis.size = concord::Size(1.0, 2.0, 0.0);
    chassis.color = pigment::RGB(255, 0, 0);

    agent.set_chassis(chassis);

    if (agent.spawn()) {
        std::cout << "Spawn successful!" << std::endl;
    } else {
        std::cout << "Spawn failed!" << std::endl;
    }

    sim_thread.join();

    return 0;
}
