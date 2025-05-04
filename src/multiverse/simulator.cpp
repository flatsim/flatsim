#include "multiverse/simulator.hpp"

namespace mvs {
    Simulator::Simulator(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}
    Simulator::~Simulator() {}
    void Simulator::tick(float dt) {
        world->tick(dt);
        for (auto &robott : robots) {
            robott->tick(dt);
        }
    }
    void Simulator::init(concord::Datum datum, mvs::Size world_size, mvs::Size grid_size) {
        world = std::make_shared<mvs::World>(rec);
        world->init(datum, world_size, grid_size);

        for (int i = 0; i < 4; ++i) {
            std::cout << "creating robot " << i << std::endl;
            concord::Pose robot_pose;
            robot_pose.point.enu.x = i * 3;
            robot_pose.point.enu.y = i * 3;
            robot_pose.point.enu.toWGS(world->get_settings().get_datum());
            robot_pose.angle.yaw = 0.0f;
            robots.emplace_back([&] {
                pigment::RGB color = pigment::RGB::random();
                auto r = std::make_unique<Robot>(rec, world);
                r->init(robot_pose, color, "robot" + std::to_string(i));
                return r;
            }());
        }
    }
    void Simulator::on_key(char key) {
        if (key >= '0' && key <= '9') {
            size_t idx = key - '0';
            if (idx < robots.size()) {
                std::cout << "Selected robot #" << robots[idx]->id() << std::endl;
                selected_robot_idx = idx;
            } else {
                std::cout << "No robot at index " << idx << "\n";
            }
        }
        if (selected_robot_idx >= 0) {
            if (key == 'w') {
                std::cout << "Pressed w" << std::endl;
                robots[selected_robot_idx]->update(0, 1);
            } else if (key == 's') {
                std::cout << "Pressed s" << std::endl;
                robots[selected_robot_idx]->update(0, -1);
            } else if (key == 'a') {
                std::cout << "Pressed a" << std::endl;
                robots[selected_robot_idx]->update(-25, 0);
            } else if (key == 'd') {
                std::cout << "Pressed d" << std::endl;
                robots[selected_robot_idx]->update(25, 0);
            }
        }
    }
} // namespace mvs
