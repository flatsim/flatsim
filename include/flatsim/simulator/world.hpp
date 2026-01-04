#pragma once

#include <memory>
#include <rerun.hpp>
#include <vector>

#include "flatsim/simulator/world/obstacle.hpp"
#include "flatsim/types.hpp"
#include "flywheel/flywheel.hpp"

namespace simulator {

    // WorldSettings extends flywheel::WorldSettings to inherit all physics defaults
    class WorldSettings : public flywheel::WorldSettings {
      private:
        datapod::Geo datum_;
        datapod::Size size_;

      public:
        WorldSettings() = default;

        void init(datapod::Geo datum, datapod::Size size) {
            datum_ = datum;
            size_ = size;
            // Set world bounds - this is critical for flywheel physics
            this->world_bounds = flywheel::AABB(flywheel::Vec2(-size.x / 2.0f, -size.y / 2.0f),
                                                flywheel::Vec2(size.x / 2.0f, size.y / 2.0f));
            // Disable gravity for top-down 2D simulation
            this->apply_gravity = false;
        }

        datapod::Geo get_datum() const { return datum_; }
        datapod::Size get_size() const { return size_; }
    };

    class World {
      private:
        WorldSettings settings_;
        std::shared_ptr<flywheel::World> physics_;

        std::vector<StaticObstacle> static_obstacles_;
        std::vector<DynamicObstacle> dynamic_obstacles_;

        size_t next_obstacle_id_ = 1;

        // Rerun visualization
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        World(std::shared_ptr<rerun::RecordingStream> rec = nullptr);
        ~World();

        // Initialize world with datum and size (must call before use)
        void init(datapod::Geo datum, datapod::Size size);

        // Physics tick (simulation step)
        void tick(float dt);

        // Visualization tock (for rendering/debug)
        void tock();

        // Obstacle management
        size_t add_obstacle(const types::StaticObstacle &obs);
        size_t add_obstacle(const types::DynamicObstacle &obs);
        void remove_obstacle(size_t id);
        void clear_obstacles();

        // Update dynamic obstacles based on a reference point (e.g., robot position)
        void update_obstacles(float dt, double ref_x, double ref_y);

        // Accessors
        flywheel::World &physics() { return *physics_; }
        const flywheel::World &physics() const { return *physics_; }
        std::shared_ptr<flywheel::World> physics_ptr() {
            return std::shared_ptr<flywheel::World>(physics_.get(), [](flywheel::World *) {});
        }
        const WorldSettings &settings() const { return settings_; }
        std::shared_ptr<flywheel::World> get_world() const { return physics_; }

        std::vector<StaticObstacle> &static_obstacles() { return static_obstacles_; }
        std::vector<DynamicObstacle> &dynamic_obstacles() { return dynamic_obstacles_; }
        const std::vector<StaticObstacle> &static_obstacles() const { return static_obstacles_; }
        const std::vector<DynamicObstacle> &dynamic_obstacles() const { return dynamic_obstacles_; }
    };

} // namespace simulator
