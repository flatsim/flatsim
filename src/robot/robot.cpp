#include "multiverse/robot.hpp"

namespace mvs {
    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}
    Robot::~Robot() {};

    void Robot::teleport(float x, float y) { chassis->teleport({x, y}); }
} // namespace mvs
