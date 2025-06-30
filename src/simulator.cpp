#include "flatsim/simulator.hpp"
#include <cmath>
#include <execution>

namespace fs {
    Simulator::Simulator(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}
    Simulator::~Simulator() {}

    void Simulator::tick(float dt) {
        if (!world) {
            throw NullPointerException("world");
        }
        world->tick(dt);
        std::for_each(std::execution::par, robots.begin(), robots.end(), [this, dt](auto &robott) {
            if (!robott) {
                throw NullPointerException("robot");
            }
            robott->tick(dt);
            for (auto layer : world->layers) {
                if (!layer)
                    continue;
            }
        });
    }

    void Simulator::init(concord::Datum datum, concord::Size world_size) {
        world = std::make_shared<fs::World>(rec);
        world->init(datum, world_size);
        world_datum = datum;
    }

    // ROBOT
    void Simulator::add_robot(RobotInfo robot_info) {
        if (!world) {
            throw NullPointerException("world");
        }

        for (auto &robot : robots) {
            if (robot && robot->info.uuid == robot_info.uuid) {
                spdlog::warn("Robot with uuid {} already exists, skipping", robot_info.uuid);
                return;
            }
        }

        auto physics_world = world->get_world();
        if (!physics_world) {
            throw NullPointerException("physics_world");
        }

        robots.emplace_back([&] {
            auto r = std::make_shared<Robot>(rec, physics_world, robots.size());
            r->init(world_datum, robot_info);
            r->set_simulator(this); // Give robot reference to simulator
            return r;
        }());
    }

    void Simulator::set_controls(uint robot_idx, float steering, float throttle) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->set_angular(steering);
        robots[robot_idx]->set_linear(throttle);
    }

    void Simulator::toggle_section_work(uint robot_idx, const std::string &karosserie_name, int section_id) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->toggle_section_work(karosserie_name, section_id);
    }

    void Simulator::toggle_all_sections_work(uint robot_idx, const std::string &karosserie_name) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->toggle_all_sections_work(karosserie_name);
    }

    void Simulator::toggle_all_except_section_work(uint robot_idx, const std::string &karosserie_name,
                                                   int except_section_id) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->toggle_all_except_section_work(karosserie_name, except_section_id);
    }

    Robot &Simulator::get_robot(uint i) {
        if (i >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(i) + " >= " + std::to_string(robots.size()));
        }

        if (!robots[i]) {
            throw NullPointerException("robot at index " + std::to_string(i));
        }

        return *robots[i];
    }
    Robot &Simulator::get_robot(const std::string &uuid) {
        for (auto &robot : robots) {
            if (!robot) {
                continue; // Skip null robots
            }
            if (robot->info.uuid == uuid) {
                return *robot;
            }
        }
        throw EntityNotFoundException("Robot", uuid);
    };
    int Simulator::num_robots() const { return robots.size(); }

    // Spatial queries
    std::vector<std::pair<std::string, concord::Pose>> Simulator::get_all_robot_poses() const {
        std::vector<std::pair<std::string, concord::Pose>> poses;

        for (const auto &robot : robots) {
            if (robot) { // Check for null robots
                poses.emplace_back(robot->info.uuid, robot->info.bound.pose);
            }
        }

        return poses;
    }

    std::vector<Robot *> Simulator::get_all_robots() const {
        std::vector<Robot *> robot_ptrs;

        for (const auto &robot : robots) {
            if (robot) { // Check for null robots
                robot_ptrs.push_back(robot.get());
            }
        }

        return robot_ptrs;
    }

    Robot *Simulator::get_closest_robot(const Robot &from_robot, float max_distance) const {
        Robot *closest = nullptr;
        float min_distance = max_distance;

        concord::Point from_pos = from_robot.info.bound.pose.point;

        for (const auto &robot : robots) {
            if (!robot || robot->info.uuid == from_robot.info.uuid) {
                continue; // Skip null robots and self
            }

            concord::Point to_pos = robot->info.bound.pose.point;
            float distance = std::sqrt(std::pow(to_pos.x - from_pos.x, 2) + std::pow(to_pos.y - from_pos.y, 2));

            if (distance < min_distance) {
                min_distance = distance;
                closest = robot.get();
            }
        }

        return closest;
    }

    // WORLD
    void Simulator::add_layer(LayerInfo layer_info, bool noise) {
        if (!world) {
            throw NullPointerException("world");
        }

        for (auto &layer : world->layers) {
            if (layer && layer->info.uuid == layer_info.uuid) {
                spdlog::warn("Layer with uuid {} already exists, skipping", layer_info.uuid);
                return;
            }
        }

        auto layer = std::make_shared<Layer>(rec, world_datum);
        layer->init(layer_info);
        world->layers.push_back(layer);
        if (noise) {
            layer->add_noise();
        }
        layer->color_field();
        world->adjust_word();
    }
    concord::Datum Simulator::get_datum() const {
        if (!world_datum.is_set()) {
            throw InitializationException("Datum not set");
        }
        return world_datum;
    }
    Layer &Simulator::get_layer(uint i) {
        if (!world) {
            throw NullPointerException("world");
        }

        if (i >= world->layers.size()) {
            throw IndexOutOfRangeException("layer index " + std::to_string(i) +
                                           " >= " + std::to_string(world->layers.size()));
        }

        if (!world->layers[i]) {
            throw NullPointerException("layer at index " + std::to_string(i));
        }

        return *world->layers[i];
    }
    Layer &Simulator::get_layer(const std::string &uuid) {
        if (!world) {
            throw NullPointerException("world");
        }

        for (auto &layer : world->layers) {
            if (!layer) {
                continue; // Skip null layers
            }
            if (layer->info.uuid == uuid) {
                return *layer;
            }
        }
        throw EntityNotFoundException("Layer", uuid);
    };

    // RERUN MANAGEMENT
    void Simulator::reset_recording() {
        static int session_counter = 0;
        std::string session_id = "flatsim_session_" + std::to_string(++session_counter);

        // Create new recording stream
        rec = std::make_shared<rerun::RecordingStream>("flatsim", session_id);
        rec->spawn().exit_on_failure();
        rec->set_global();

        // Reset time state
        rec->reset_time();

        // Note: World and Robot classes would need to be updated to support
        // recording stream replacement if needed
    }

    void Simulator::clear_all_entities() {
        if (!rec) {
            return;
        }

        // Clear all entities recursively from root
        rec->log("", rerun::Clear::RECURSIVE);

        // Reset timeline
        rec->reset_time();
    }
} // namespace fs
