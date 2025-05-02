#include "multiverse/robot.hpp"

namespace mvs {
    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}
    Robot::~Robot() {}

    void Robot::tick(float dt) {
        for (auto &sensor : sensors) {
            sensor->tick(dt, position);
        }
        chassis->tick(dt);
    }

    void Robot::init(concord::Pose pose, std::string name) {
        std::cout << "Initializing robot " << name << "...\n";
        this->name = name;
        chassis = std::make_unique<vehicle::Vehicle>(world, pose);
    }

    // void Robot::teleport(float x, float y) { chassis->teleport({x, y}); }
} // namespace mvs
