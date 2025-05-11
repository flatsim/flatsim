#include "multiverse/simulator.hpp"

namespace mvs {
    double mapper(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    Simulator::Simulator(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}
    Simulator::~Simulator() {}
    void Simulator::tick(float dt) {
        world->tick(dt);
        for (auto &robott : robots) {
            robott->tick(dt);
        }
    }
    void Simulator::init(concord::Datum datum, concord::Size world_size, float grid_size) {
        world = std::make_shared<mvs::World>(rec);

        world->init(datum, world_size, grid_size);
        world_datum = datum;
    }

    void Simulator::add_robot(concord::Pose robot_pose, pigment::RGB robot_color, concord::Size chassis_size,
                              std::vector<concord::Bound> wheels,
                              std::pair<std::vector<float>, std::vector<float>> controls,
                              std::vector<concord::Bound> karosserie) {
        robots.emplace_back([&] {
            auto r = std::make_shared<Robot>(rec, world->get_world(), robots.size());
            r->init(world_datum, robot_pose, chassis_size, robot_color, "robot" + std::to_string(robots.size()), wheels,
                    karosserie);
            r->set_controls(controls.first, controls.second);
            return r;
        }());
    }

    void Simulator::on_joystick_axis(int axis, float value) {
        if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
            if (axis == 0) {
                float steering = value * 30.0f;
                robots[selected_robot_idx]->set_angular(steering);
            }

            if (axis == 1) {
                float throttle = value * 0.2f;
                robots[selected_robot_idx]->set_linear(throttle);
            }
        }
    }
    void Simulator::on_joystick_button(int button, bool pressed) {
        if (button < 4 && pressed) {
            selected_robot_idx = button;
            std::cout << "Selected robot #" << selected_robot_idx << std::endl;
        }
        if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
            if ((button == 4 || button == 5) && pressed) {
                robots[selected_robot_idx]->respawn();
            }
            if (button == 9 && pressed) {
                robots[selected_robot_idx]->pulsining = true;
            }
        }
    }
} // namespace mvs
