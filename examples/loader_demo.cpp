#include "flatsim/agent/loader.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <thread>

int main() {
    std::cout << "[Example] Loader demo - Load robot from JSON" << std::endl;

    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(simulator::Conn::IPC, "", 100.0f, 100.0f, datum, rec);

    try {
        datapod::Pose spawn_pose;
        spawn_pose.point.x = 0.0;
        spawn_pose.point.y = 0.0;
        utils::set_yaw(spawn_pose, 0.0);

        auto machine = agent::Loader::load_from_json("examples/machines/husky.json", spawn_pose);

        std::cout << "[Loader] Loaded machine: " << machine.name << std::endl;
        std::cout << "[Loader] Type: " << machine.type << std::endl;
        std::cout << "[Loader] UUID: " << machine.uuid << std::endl;
        std::cout << "[Loader] Wheels: " << machine.wheels.size() << std::endl;

        sim.create_machine(machine);

        const float dt = 0.016f;
        const auto wheel_count = machine.wheels.size();
        std::cout << "[Example] Driving in circle - steering left with forward throttle" << std::endl;

        for (int i = 0; i < 500; ++i) {
            types::WheelControl ctrl;
            ctrl.uuid = machine.uuid;
            ctrl.steering.assign(wheel_count, 0.0f);
            ctrl.throttle.assign(wheel_count, 0.6f);

            // Best-effort: assume the first 2 wheels are steerable
            if (wheel_count >= 2) {
                ctrl.steering[0] = 0.3f;
                ctrl.steering[1] = 0.3f;
            }

            sim.apply_control(ctrl, dt);
            sim.tick(dt);

            if (i % 2 == 0) {
                sim.tock();
            }

            if (i % 60 == 0) {
                auto ws_state = sim.get_world_state();
                for (const auto &ms : ws_state.machines) {
                    if (std::string(ms.uuid.view()) == machine.uuid) {
                        std::cout << "[Sim] Tick " << i << " - Position: (" << ms.pose.position.x << ", "
                                  << ms.pose.position.y << ") yaw=" << ms.pose.angle << std::endl;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    } catch (const std::exception &e) {
        std::cerr << "[Error] Failed to load machine: " << e.what() << std::endl;
    }

    std::cout << "[Example] Done!" << std::endl;

    return 0;
}
