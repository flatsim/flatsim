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

    void Simulator::on_joystick_axis(int axis, float value) {
        if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
            if (axis == 0) {
                angle = -value * 45;
            } else if (axis == 1) {
                throttle = -value;
            }
            robots[selected_robot_idx]->update(angle, throttle);

            // std::cout << "Joystick axis " << axis << " value: " << value << std::endl;
            // robots[selected_robot_idx]->update(value * 10, 0);
        }
    }
    void Simulator::on_joystick_button(int button, bool pressed) {
        if (button < 4 && pressed) {
            selected_robot_idx = button;
            std::cout << "Selected robot #" << selected_robot_idx << std::endl;
        }
    }
} // namespace mvs
