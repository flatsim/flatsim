#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <chrono>
#include <iostream>
#include <rerun.hpp>
#include <thread>

int main() {
    std::cout << "[Example] Loader demo - Load robot from URDF" << std::endl;

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

        auto model = agent::Agent::load_model_from_urdf("examples/machines/urdf/husky.urdf");
        std::cout << "[URDF] Parsed dp::robot::Model from husky.urdf" << std::endl;
        (void)model;

        std::cout << "[Info] Loader removed; dp::robot::Model parsed + validated." << std::endl;
        return 0;

        // Old demo used Simulator + types::Machine; that path is intentionally removed.
    } catch (const std::exception &e) {
        std::cerr << "[Error] Failed to load machine: " << e.what() << std::endl;
    }

    std::cout << "[Example] Done!" << std::endl;

    return 0;
}
