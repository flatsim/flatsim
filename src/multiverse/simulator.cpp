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
    void Simulator::add_robot(RobotInfo robot_info) {
        for (auto &robot : robots) {
            if (robot->info.uuid == robot_info.uuid) {
                spdlog::warn("Robot with uuid {} already exists, skipping", robot_info.uuid);
                return;
            }
        }
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

    Robot &Simulator::get_robot(uint i) {
        if (i >= robots.size()) {
            throw std::runtime_error("Robot index out of range");
        }
        return *robots[i];
    }
    Robot &Simulator::get_robot(std::string uuid) {
        for (auto &robot : robots) {
            if (robot->info.uuid == uuid) {
                return *robot;
            }
        }
        throw std::runtime_error("Robot with uuid " + uuid + " not found");
    };
    int Simulator::num_robots() const { return robots.size(); }

    // WORLD
    void Simulator::add_layer(LayerInfo layz, bool noise) {
        for (auto &layer : world->layers) {
            if (layer->info.uuid == layz.uuid) {
                spdlog::warn("Layer with uuid {} already exists, skipping", layz.uuid);
                return;
            }
        }
        auto layer = std::make_shared<Layer>(rec, world_datum);
        layer->init(layz);
        world->layers.push_back(layer);
        if (noise) {
            layer->add_noise();
        }
        layer->color_field();
    }
    concord::Datum Simulator::get_datum() const {
        if (!world_datum.is_set()) {
            throw std::runtime_error("Datum not set");
        }
        return world_datum;
    }
    Layer &Simulator::get_layer(uint i) { return *world->layers[i]; }
    Layer &Simulator::get_layer(std::string uuid) {
        for (auto &layer : world->layers) {
            if (layer->info.uuid == uuid) {
                return *layer;
            }
        }
        throw std::runtime_error("Layer with uuid " + uuid + " not found");
    };
} // namespace mvs
