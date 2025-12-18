#include "flatsim/agent.hpp"
#include "flatsim/agent/loader.hpp"
#include "flatsim/simulator.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <thread>

int main() {
    std::cout << "[Example] Loader demo - Load robot from JSON" << std::endl;

    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started" << std::endl;

    std::atomic<bool> running{true};

    std::thread sim_thread([&rec, &running]() {
        simulator::WorldSettings ws{100.0f, 100.0f};
        simulator::Simulator sim(simulator::Conn::IPC, "", ws, rec);

        std::cout << "[SimThread] Starting sim loop..." << std::endl;
        for (int i = 0; i < 500 && running.load(); ++i) {
            sim.tick(0.016f);
            if (i % 2 == 0) {
                sim.tock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        std::cout << "[SimThread] Sim loop ended" << std::endl;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    agent::Agent agnt("", rec);

    try {
        concord::Pose spawn_pose;
        spawn_pose.point.x = 0.0;
        spawn_pose.point.y = 0.0;
        spawn_pose.angle.yaw = 0.0;

        auto machine = agent::Loader::load_from_json("examples/machines/husky.json", spawn_pose);

        std::cout << "[Loader] Loaded machine: " << machine.name << std::endl;
        std::cout << "[Loader] Type: " << machine.type << std::endl;
        std::cout << "[Loader] UUID: " << machine.uuid << std::endl;
        std::cout << "[Loader] Wheels: " << machine.wheels.size() << std::endl;

        agnt.set_machine(machine);

        if (agnt.spawn()) {
            std::cout << "[Agent] Spawn successful!" << std::endl;

            std::thread viz_thread([&agnt, &running]() {
                while (running.load()) {
                    agnt.tock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }
            });

            std::cout << "[Example] Driving in circle - steering left with forward throttle" << std::endl;

            for (int i = 0; i < 250; ++i) {
                types::MachineControl ctrl;
                ctrl.uuid = machine.uuid;

                // Front wheels steer left (0.3 radians ~17 degrees)
                // Rear wheels don't steer (0.0)
                // This creates circular motion (Ackermann steering)
                ctrl.steering = {0.3f, 0.3f, 0.0f, 0.0f};

                // All wheels drive forward
                ctrl.throttle = {0.6f, 0.6f, 0.6f, 0.6f};

                agnt.control(ctrl);

                if (i % 60 == 0) {
                    auto pose = agnt.machine().world_pose();
                    std::cout << "[Example] Frame " << i << " - Position: (" << pose.point.x << ", " << pose.point.y
                              << ") Yaw: " << pose.angle.yaw << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }

            running.store(false);
            viz_thread.join();
        }
    } catch (const std::exception &e) {
        std::cerr << "[Error] Failed to load machine: " << e.what() << std::endl;
        running.store(false);
    }

    sim_thread.join();
    std::cout << "[Example] Done!" << std::endl;

    return 0;
}
