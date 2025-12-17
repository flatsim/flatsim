#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "drivekit/types.hpp"
#include "flatsim/core/exceptions.hpp"
#include "flatsim/core/utils.hpp"
#include "flatsim/world/obstacle/dynamic.hpp"
#include "flatsim/world/obstacle/static.hpp"

namespace fs {
    class WorldSettings : public muli::WorldSettings {
      private:
        concord::Datum world_datum_;
        concord::Size world_size_;

      public:
        WorldSettings() = default;
        void init(concord::Datum datum, concord::Size world_size) {
            world_datum_ = datum;
            world_size_ = world_size;
            this->world_bounds = muli::AABB(muli::Vec2(-world_size.x / 2.0f, -world_size.y / 2.0f),
                                            muli::Vec2(world_size.x / 2.0f, world_size.y / 2.0f));
        }
        concord::Datum get_datum() const { return world_datum_; }
        concord::Size get_world_size() const { return world_size_; }
    };

    class World {
      private:
        WorldSettings settings;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        concord::Rectangle world_bounds;

        // Obstacles
        std::vector<StaticObstacle> static_obstacles;
        std::vector<DynamicObstacle> dynamic_obstacles;

      public:
        World(std::shared_ptr<rerun::RecordingStream> rec);
        ~World();
        void adjust_word();

        void init(concord::Datum datum, concord::Size world_size);
        void tick(float dt);
        void tock();

        std::shared_ptr<muli::World> get_world() const { return world; }
        const WorldSettings &get_settings() const { return settings; }

        // Obstacle management
        void add_obstacle(const StaticObstacle &obs);
        void add_obstacle(const DynamicObstacle &obs);
        void clear_obstacles();

        // Update dynamic obstacles based on robot position
        void update_obstacles(float dt, double robot_x, double robot_y);

        // Convert obstacles to drivekit WorldConstraints for navigation
        // horizon_steps and dt are used for predicting dynamic obstacle trajectories
        drivekit::WorldConstraints get_world_constraints(size_t horizon_steps = 20, double dt = 0.1) const;

        // Access obstacles for visualization
        const std::vector<StaticObstacle> &get_static_obstacles() const { return static_obstacles; }
        const std::vector<DynamicObstacle> &get_dynamic_obstacles() const { return dynamic_obstacles; }
    };
} // namespace fs
