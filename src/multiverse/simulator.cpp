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
    void Simulator::set_controls(uint robot_idx, float steering, float throttle) {
        if (robot_idx >= 0 && robot_idx < robots.size()) {
            robots[robot_idx]->set_angular(steering);
            robots[robot_idx]->set_linear(throttle);
        }
    }
    void Simulator::set_controls(std::string robot_name, float steering, float throttle) {
        for (uint i = 0; i < robots.size(); ++i) {
            if (robots[i]->id() == robot_name) {
                robots[i]->set_angular(steering);
                robots[i]->set_linear(throttle);
                break;
            }
        }
    }

} // namespace mvs
