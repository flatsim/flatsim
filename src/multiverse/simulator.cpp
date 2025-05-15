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
    void Simulator::init(concord::Datum datum, concord::Size world_size) {
        world = std::make_shared<mvs::World>(rec);

        world->init(datum, world_size);
        world_datum = datum;
    }

    // ROBOT
    void Simulator::add_robot(Robo robot_info) {
        robots.emplace_back([&] {
            auto r = std::make_shared<Robot>(rec, world->get_world(), robots.size());
            r->init(world_datum, robot_info);
            return r;
        }());
    }

    void Simulator::set_controls(uint robot_idx, float steering, float throttle) {
        if (robot_idx < robots.size()) {
            robots[robot_idx]->set_angular(steering);
            robots[robot_idx]->set_linear(throttle);
        }
    }

    // WORLD
    void Simulator::add_layer(Layz layz, bool noise) {
        auto field = layz.field;
        auto layer = std::make_shared<Layer>(rec, world_datum);
        auto rows = static_cast<std::size_t>(field.size.x / layz.resolution);
        auto cols = static_cast<std::size_t>(field.size.y / layz.resolution);
        layer->init(layz.name, layz.uuid, layz.color, layz.field, rows, cols, layz.resolution, layz.centered);
        world->layers.push_back(layer);
        if (noise) {
            layer->add_noise();
        }
    }
} // namespace mvs
