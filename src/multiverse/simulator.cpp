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
        if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
            if (controls_set) {
                robots[selected_robot_idx]->update(steerings, throttles);
            }
        }
    }
    void Simulator::init(concord::Datum datum, concord::Size world_size, float grid_size) {
        world = std::make_shared<mvs::World>(rec);

        world->init(datum, world_size, grid_size);
        world_datum = datum;
    }

    void Simulator::add_robot(concord::Pose robot_pose, pigment::RGB robot_color, concord::Size chassis_size,
                              std::vector<concord::Bound> wheels, std::vector<concord::Bound> karosserie) {
        robots.emplace_back([&] {
            auto r = std::make_shared<Robot>(rec, world->get_world(), robots.size());
            r->init(world_datum, robot_pose, chassis_size, robot_color, "robot" + std::to_string(robots.size()), wheels,
                    karosserie);
            return r;
        }());
    }

    void Simulator::set_controls(std::vector<float> steerings_max, std::vector<float> throttles_max) {
        this->steerings_max = steerings_max;
        this->throttles_max = throttles_max;
        this->steerings.resize(steerings_max.size(), 0.0f);
        this->throttles.resize(throttles_max.size(), 0.0f);
        controls_set = true;
    }

    void Simulator::on_joystick_axis(int axis, float value) {
        if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
            auto ster_mult = 30.0f;
            auto thr_mult = 0.2f;
            if (axis == 0) {
                steering = -value * ster_mult;
                for (uint i = 0; i < steerings.size(); ++i) {
                    steerings[i] = mapper(steering, 1.0f, -1.0f, steerings_max[i], -steerings_max[i]);
                    std::cout << "Steering " << steerings[i] << std::endl;
                }
            }

            if (axis == 1) {
                auto preval = -value * thr_mult;
                for (uint i = 0; i < throttles.size(); ++i) {
                    throttles[i] = mapper(preval, 1.0f, -1.0f, throttles_max[i], -throttles_max[i]);
                    std::cout << "Throttle " << throttles[i] << std::endl;
                }
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
