#include "flatsim/agent/loader.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/types.hpp"
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
    std::cout << "[Example] Visualization demo (single-thread tick()/tock() + Rerun)" << std::endl;

    // Setup Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    rec->spawn().exit_on_failure();
    std::cout << "[Rerun] Visualization started" << std::endl;

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(simulator::Conn::IPC, "", 100.0f, 100.0f, datum, rec);

    // Load tractor from JSON
    std::cout << "[Example] Loading tractor from JSON..." << std::endl;
    types::Machine machine =
        agent::Loader::load_from_json("examples/machines/tractor.json", utils::make_pose_2d(0.0, 0.0, 0.0));
    std::cout << "[Example] Loaded machine: " << machine.name << " with " << machine.wheels.size() << " wheels, "
              << machine.karosseries.size() << " karosseries, " << machine.hitches.size() << " hitches" << std::endl;

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
