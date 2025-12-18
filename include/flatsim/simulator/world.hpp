#pragma once

#include <memory>
#include <vector>

#include "flatsim/simulator/world/obstacle.hpp"
#include "flatsim/types.hpp"
#include "muli/world.h"

namespace simulator {

    class World {
      private:
        std::unique_ptr<muli::World> physics_;
        types::WorldSettings settings_;

        std::vector<StaticObstacle> static_obstacles_;
        std::vector<DynamicObstacle> dynamic_obstacles_;

        size_t next_obstacle_id_ = 1;

      public:
        World(const types::WorldSettings &settings = {});
        ~World();

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
        muli::World &physics() { return *physics_; }
        const muli::World &physics() const { return *physics_; }
        const types::WorldSettings &settings() const { return settings_; }

        std::vector<StaticObstacle> &static_obstacles() { return static_obstacles_; }
        std::vector<DynamicObstacle> &dynamic_obstacles() { return dynamic_obstacles_; }
        const std::vector<StaticObstacle> &static_obstacles() const { return static_obstacles_; }
        const std::vector<DynamicObstacle> &dynamic_obstacles() const { return dynamic_obstacles_; }
    };

} // namespace simulator
