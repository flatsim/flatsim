#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
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

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(simulator::Conn::IPC, "", 100.0f, 100.0f, datum, rec);

    std::cout << "[Example] Parsing URDF into dp::robot::Model..." << std::endl;
    auto model = agent::Agent::load_model_from_urdf("examples/machines/urdf/tractor.urdf");
    std::cout << "[Example] URDF parsed + validated" << std::endl;
    (void)model;

    std::cout << "[Info] Loader removed; dp::robot::Model parsed + validated." << std::endl;
    return 0;

    // Old demo relied on Simulator + types::Machine; that path is intentionally removed.

    std::cout << "[Example] Done!" << std::endl;

    return 0;
}
