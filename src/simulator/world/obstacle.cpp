#include "flatsim/simulator/world/obstacle.hpp"

namespace simulator {

    // ============================================================================
    // StaticObstacle
    // ============================================================================

    void StaticObstacle::create(muli::World &world) {
        if (body_) return; // Already created

        float x = static_cast<float>(config_.position.x);
        float y = static_cast<float>(config_.position.y);
        float r = static_cast<float>(config_.radius);

        muli::Transform tf;
        tf.position = muli::Vec2(x, y);
        tf.rotation = muli::Rotation(0.0f);

        body_ = world.CreateCircle(r, tf, muli::RigidBody::static_body);
    }

    void StaticObstacle::destroy(muli::World &world) {
        if (body_) {
            world.Destroy(body_);
            body_ = nullptr;
        }
    }

    // ============================================================================
    // DynamicObstacle
    // ============================================================================

    void DynamicObstacle::create(muli::World &world) {
        if (body_) return; // Already created

        float x = static_cast<float>(config_.position.x);
        float y = static_cast<float>(config_.position.y);
        float r = static_cast<float>(config_.radius);

        muli::Transform tf;
        tf.position = muli::Vec2(x, y);
        tf.rotation = muli::Rotation(0.0f);

        body_ = world.CreateCircle(r, tf, muli::RigidBody::kinematic_body);
    }

    void DynamicObstacle::destroy(muli::World &world) {
        if (body_) {
            world.Destroy(body_);
            body_ = nullptr;
        }
    }

    void DynamicObstacle::update(float dt, double ref_x, double ref_y) {
        double dx = config_.position.x - ref_x;
        double dy = config_.position.y - ref_y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < config_.activation_distance) {
            config_.is_active = true;
        }

        if (config_.is_active) {
            config_.position.x += config_.velocity.x * dt;
            config_.position.y += config_.velocity.y * dt;

            if (body_) {
                body_->SetPosition(
                    muli::Vec2(static_cast<float>(config_.position.x), static_cast<float>(config_.position.y)));
            }
        }
    }

    datapod::Point DynamicObstacle::predict(double t) const {
        if (!config_.is_active) {
            return config_.position;
        }
        return datapod::Point{config_.position.x + config_.velocity.x * t, config_.position.y + config_.velocity.y * t,
                              config_.position.z};
    }

} // namespace simulator
