#include "multiverse/robot.hpp"

namespace mvs {
    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}
    Robot::~Robot() { delete body; }

    void Robot::tick(float dt) {
        for (auto &sensor : sensors) {
            sensor->tick(dt, position);
        }
    }

    void Robot::teleport(float x, float y) {
        auto tf = muli::Transform({x, y});
        body->SetTransform(tf);
    }
} // namespace mvs
