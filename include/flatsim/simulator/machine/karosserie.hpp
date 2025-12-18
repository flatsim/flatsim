#pragma once

#include "flatsim/types.hpp"
#include "muli/world.h"

namespace simulator {

    class Karosserie {
      private:
        muli::Collider *collider_ = nullptr;
        types::Karosserie config_;

      public:
        Karosserie() = default;
        Karosserie(const types::Karosserie &config);

        // Create collider attached to parent body
        void create(muli::RigidBody *parent_body, const muli::CollisionFilter &filter);

        // Accessors
        muli::Collider *collider() const { return collider_; }
        const types::Karosserie &config() const { return config_; }
    };

} // namespace simulator
