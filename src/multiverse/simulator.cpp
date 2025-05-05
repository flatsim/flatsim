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
    void Simulator::init(concord::Datum datum, concord::Size world_size, concord::Size grid_size) {
        world = std::make_shared<mvs::World>(rec);
        world->init(datum, world_size, grid_size);

        for (int i = 0; i < 4; ++i) {
            std::cout << "creating robot " << i << std::endl;
            concord::Pose robot_pose;
            robot_pose.point.enu.x = i * 3;
            robot_pose.point.enu.y = i * 3;
            robot_pose.point.enu.toWGS(world->get_settings().get_datum());
            robot_pose.angle.yaw = 0.0f;
            concord::Size chassis_size{0.8f, 1.4f, 0.0f};
            std::vector<concord::Size> wheel_sizes;
            wheel_sizes.push_back({0.1f, 0.2f, 0.0f});
            wheel_sizes.push_back({0.1f, 0.2f, 0.0f});
            wheel_sizes.push_back({0.2f, 0.4f, 0.0f});
            wheel_sizes.push_back({0.2f, 0.4f, 0.0f});
            robots.emplace_back([&] {
                pigment::RGB color = pigment::RGB::random();
                auto r = std::make_unique<Robot>(rec, world, i);
                r->init(robot_pose, chassis_size, color, "robot" + std::to_string(i), wheel_sizes);
                return r;
            }());
        }
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
                steerings[2] = steering;
                steerings[3] = steering;
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
        if ((button == 4 || button == 5) && pressed && selected_robot_idx >= 0 && selected_robot_idx < 4) {
            robots[selected_robot_idx]->respawn();
            //     concord::Pose pose;
            //     pose.point.enu.x = 0;
            //     pose.point.enu.y = 0;
            //     robots[selected_robot_idx]->teleport(pose);
        }
    }
} // namespace mvs
