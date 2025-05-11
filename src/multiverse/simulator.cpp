#include "multiverse/simulator.hpp"

namespace mvs {
    Simulator::Simulator(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}
    Simulator::~Simulator() {}
    void Simulator::tick(float dt) {
        world->tick(dt);
        for (auto &robott : robots) {
            robott->tick(dt);
        }
        if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
            robots[selected_robot_idx]->update(steerings, throttles);
        }
    }
    void Simulator::init(concord::Datum datum, concord::Size world_size, float grid_size) {
        world = std::make_shared<mvs::World>(rec);

        world->init(datum, world_size, grid_size);
        world_datum = datum;
    }

    void Simulator::add_robot(concord::Pose robot_pose, pigment::RGB robot_color, concord::Size chassis_size,
                              std::vector<concord::Bound> wheels, std::vector<concord::Bound> karosserie) {
        throttles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        steerings = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        robots.emplace_back([&] {
            auto r = std::make_shared<Robot>(rec, world->get_world(), robots.size());
            r->init(world_datum, robot_pose, chassis_size, robot_color, "robot" + std::to_string(robots.size()), wheels,
                    karosserie);
            return r;
        }());
    }

    void Simulator::on_joystick_axis(int axis, float value) {
        if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
            auto ster_mult = 30.0f;
            auto thr_mult = 0.2f;
            if (axis == 0) {
                steering = -value * ster_mult;
                steerings[0] = steering;
                steerings[1] = steering;
            }

            if (axis == 1) {
                auto preval = -value * thr_mult;
                throttle = (fabs(preval) < 0.1f) ? 0.0f : preval;
                throttles[0] = throttle;
                throttles[1] = throttle;
            }

            if (axis == 3) {
                steering = -value * ster_mult;
                steerings[4] = steering;
                steerings[5] = steering;
            }

            if (axis == 4) {
                auto preval = -value * thr_mult;
                throttle = (fabs(preval) < 0.1f) ? 0.0f : preval;
                throttles[2] = throttle;
                throttles[3] = throttle;
            }
            if (axis == 2 || axis == 8) {
                std::cout << "Axis " << axis << " value " << value << std::endl;
                auto preval = value * thr_mult;
                throttle = (fabs(preval) < 0.1f) ? 0.0f : preval;
                throttles[0] = throttle;
                throttles[1] = -throttle;
                throttles[2] = throttle;
                throttles[3] = -throttle;
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
