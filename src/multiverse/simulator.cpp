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
                selected_robot = robots[idx]; // just copy the shared_ptr
                std::cout << "Selected robot #" << idx << " (use count=" << selected_robot.use_count() << ")\n";
            } else {
                selected_robot.reset();
                std::cout << "No robot at index " << idx << "\n";
            }
        }
        if (selected_robot) {
            if (key == 'w') {
                std::cout << "Pressed W\n" << std::endl;
            } else if (key == 's') {
                std::cout << "Pressed S\n" << std::endl;
            } else if (key == 'a') {
                std::cout << "Pressed A\n" << std::endl;
            } else if (key == 'd') {
                std::cout << "Pressed D\n" << std::endl;
            }
        }
    }
} // namespace mvs
