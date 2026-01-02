#pragma once

#include <cmath>

#include "flatsim/types.hpp"
#include "flywheel/common.h"
#include "flywheel/rigidbody.h"
#include "flywheel/world.h"

namespace simulator {

    // Static obstacle wrapper with physics body
    class StaticObstacle {
      private:
        types::StaticObstacle config_;
        flywheel::RigidBody *body_ = nullptr;

      public:
        StaticObstacle() = default;
        StaticObstacle(const types::StaticObstacle &config) : config_(config) {}

        void create(flywheel::World &world);
        void destroy(flywheel::World &world);

        size_t id() const { return config_.id; }
        const types::StaticObstacle &config() const { return config_; }
        const datapod::Point &position() const { return config_.position; }
        double radius() const { return config_.radius; }
    };

    // Dynamic obstacle wrapper with physics body and movement logic
    class DynamicObstacle {
      private:
        types::DynamicObstacle config_;
        flywheel::RigidBody *body_ = nullptr;

      public:
        DynamicObstacle() = default;
        DynamicObstacle(const types::DynamicObstacle &config) : config_(config) {}

        void create(flywheel::World &world);
        void destroy(flywheel::World &world);

        // Update position based on proximity to reference point
        void update(float dt, double ref_x, double ref_y);

        // Predict future position at time t
        datapod::Point predict(double t) const;

        size_t id() const { return config_.id; }
        const types::DynamicObstacle &config() const { return config_; }
        const datapod::Point &position() const { return config_.position; }
        double radius() const { return config_.radius; }
        bool is_active() const { return config_.is_active; }
    };

} // namespace simulator
