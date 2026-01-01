#include "flatsim/simulator/world.hpp"
#include <algorithm>
#include <iostream>

namespace simulator {

    World::World(std::shared_ptr<rerun::RecordingStream> rec) : rec_(rec) {}

    World::~World() = default;

    void World::init(datapod::Geo datum, datapod::Size size) {
        settings_.init(datum, size);
        // Pass settings_ (which extends muli::WorldSettings) directly to muli::World
        // This ensures all muli defaults are properly inherited
        physics_ = std::make_shared<muli::World>(settings_);
        std::cout << "[World] Created (" << size.x << "x" << size.y << ")" << std::endl;
    }

    void World::tick(float dt) { physics_->Step(dt); }

    void World::tock() {
        if (!rec_) return;

        // Visualize world boundaries
        float w = static_cast<float>(settings_.get_size().x);
        float h = static_cast<float>(settings_.get_size().y);
        std::vector<std::array<float, 3>> corners = {
            {-w / 2.0f, -h / 2.0f, 0.0f},
            {w / 2.0f, -h / 2.0f, 0.0f},
            {w / 2.0f, h / 2.0f, 0.0f},
            {-w / 2.0f, h / 2.0f, 0.0f},
            {-w / 2.0f, -h / 2.0f, 0.0f} // Close the loop
        };

        auto border = rerun::components::LineStrip3D(corners);
        rec_->log_static("border", rerun::LineStrips3D(border).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

        // Visualize static obstacles (RED flat squares)
        for (const auto &obs : static_obstacles_) {
            std::string name = "obstacles/static_" + std::to_string(obs.id());
            rec_->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                       {{float(obs.position().x), float(obs.position().y), 0.0f}},
                                       {{float(obs.radius()), float(obs.radius()), 0.0f}})
                                       .with_colors(rerun::Color(255, 0, 0)));
        }

        // Visualize dynamic obstacles (GREEN flat squares)
        for (const auto &obs : dynamic_obstacles_) {
            std::string name = "obstacles/dynamic_" + std::to_string(obs.id());
            rec_->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                       {{float(obs.position().x), float(obs.position().y), 0.0f}},
                                       {{float(obs.radius()), float(obs.radius()), 0.0f}})
                                       .with_colors(rerun::Color(0, 255, 0)));
        }
    }

    size_t World::add_obstacle(const types::StaticObstacle &obs) {
        types::StaticObstacle config = obs;
        if (config.id == 0) {
            config.id = next_obstacle_id_++;
        }

        StaticObstacle obstacle(config);
        obstacle.create(*physics_);
        static_obstacles_.push_back(std::move(obstacle));

        return config.id;
    }

    size_t World::add_obstacle(const types::DynamicObstacle &obs) {
        types::DynamicObstacle config = obs;
        if (config.id == 0) {
            config.id = next_obstacle_id_++;
        }

        DynamicObstacle obstacle(config);
        obstacle.create(*physics_);
        dynamic_obstacles_.push_back(std::move(obstacle));

        return config.id;
    }

    void World::remove_obstacle(size_t id) {
        // Remove from static obstacles
        auto static_it = std::find_if(static_obstacles_.begin(), static_obstacles_.end(),
                                      [id](const StaticObstacle &obs) { return obs.id() == id; });
        if (static_it != static_obstacles_.end()) {
            static_it->destroy(*physics_);
            static_obstacles_.erase(static_it);
            return;
        }

        // Remove from dynamic obstacles
        auto dynamic_it = std::find_if(dynamic_obstacles_.begin(), dynamic_obstacles_.end(),
                                       [id](const DynamicObstacle &obs) { return obs.id() == id; });
        if (dynamic_it != dynamic_obstacles_.end()) {
            dynamic_it->destroy(*physics_);
            dynamic_obstacles_.erase(dynamic_it);
        }
    }

    void World::clear_obstacles() {
        for (auto &obs : static_obstacles_) {
            obs.destroy(*physics_);
        }
        static_obstacles_.clear();

        for (auto &obs : dynamic_obstacles_) {
            obs.destroy(*physics_);
        }
        dynamic_obstacles_.clear();
    }

    void World::update_obstacles(float dt, double ref_x, double ref_y) {
        for (auto &obs : dynamic_obstacles_) {
            obs.update(dt, ref_x, ref_y);
        }
    }

} // namespace simulator
