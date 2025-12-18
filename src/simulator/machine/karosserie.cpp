#include "flatsim/simulator/machine/karosserie.hpp"

namespace simulator {

    Karosserie::Karosserie(const types::Karosserie &config) : config_(config) {}

    void Karosserie::create(muli::RigidBody *parent_body, const muli::CollisionFilter &filter) {
        if (!config_.has_physics || !parent_body) {
            return;
        }

        muli::Transform karos_tf;
        karos_tf.position.x = static_cast<float>(config_.pose.point.x);
        karos_tf.position.y = static_cast<float>(config_.pose.point.y);
        karos_tf.rotation = static_cast<float>(config_.pose.angle.yaw);

        collider_ = parent_body->CreateBoxCollider(static_cast<float>(config_.size.x),
                                                   static_cast<float>(config_.size.y), 0.02f, karos_tf);
        if (collider_) {
            collider_->SetFilter(filter);
        }
    }

    void Karosserie::tick(float dt) {
        // Future: Update sensors, compute metrics, etc.
    }

    void Karosserie::tock() {
        // Future: Visualization, debug rendering
    }

} // namespace simulator
