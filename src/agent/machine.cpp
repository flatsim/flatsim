#include "flatsim/agent/machine.hpp"

namespace agent {

    Machine::Machine(const types::Machine &config, std::shared_ptr<rerun::RecordingStream> rec)
        : config_(config), rec_(rec) {
        // Create wheel objects
        for (const auto &wheel_cfg : config_.wheels) {
            wheels_.emplace_back(wheel_cfg, config_, rec_);
        }

        // Create karosserie objects
        for (const auto &karos_cfg : config_.karosseries) {
            karosseries_.emplace_back(karos_cfg, config_, rec_);
        }

        // Create hitch objects
        for (const auto &hitch_cfg : config_.hitches) {
            hitches_.emplace_back(hitch_cfg, config_, rec_);
        }

        // Initialize world pose from config
        world_pose_ = config_.pose;
    }

    void Machine::set_rerun(std::shared_ptr<rerun::RecordingStream> rec) {
        rec_ = rec;
        for (auto &wheel : wheels_) {
            wheel.set_rerun(rec);
        }
        for (auto &karos : karosseries_) {
            karos.set_rerun(rec);
        }
        for (auto &hitch : hitches_) {
            hitch.set_rerun(rec);
        }
    }

    void Machine::update_state(const types::ser::MachineState &state) {
        // Update world pose
        world_pose_ = state.pose.to_concord();

        // Debug
        static int update_count = 0;
        if (update_count++ % 60 == 0) {
            std::cout << "[Machine] State update " << update_count << " - Pose: (" << world_pose_.point.x << ", "
                      << world_pose_.point.y << ")" << std::endl;
        }

        // Update wheel poses (world poses from simulator)
        for (size_t i = 0; i < state.wheels.size() && i < wheels_.size(); ++i) {
            wheels_[i].update_state(state.wheels[i]);
        }
    }

    Hitch *Machine::find_hitch(const std::string &name) {
        for (auto &hitch : hitches_) {
            if (hitch.config().name == name) {
                return &hitch;
            }
        }
        return nullptr;
    }

    void Machine::tick(float dt) {
        // Tick all wheels
        for (auto &wheel : wheels_) {
            wheel.tick(dt);
        }

        // Tick all karosseries
        for (auto &karosserie : karosseries_) {
            karosserie.tick(dt);
        }

        // Tick all hitches
        for (auto &hitch : hitches_) {
            hitch.tick(dt);
        }
    }

    void Machine::tock() {
        if (!rec_) return;

        // Visualize machine chassis body
        float x = static_cast<float>(world_pose_.point.x);
        float y = static_cast<float>(world_pose_.point.y);
        float th = static_cast<float>(world_pose_.angle.yaw);
        float w = static_cast<float>(config_.size.x);
        float h = static_cast<float>(config_.size.y);

        rec_->log_static(
            config_.uuid + "/chassis",
            rerun::Boxes3D::from_centers_and_sizes({{x, y, 0.1f}}, {{w, h, 0.0f}})
                .with_radii({{0.02f}})
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(th))})
                .with_colors({rerun::Color(config_.color.r, config_.color.g, config_.color.b)}));

        // Tock all wheels
        for (auto &wheel : wheels_) {
            wheel.tock();
        }

        // Tock all karosseries
        for (auto &karosserie : karosseries_) {
            karosserie.tock();
        }

        // Tock all hitches
        for (auto &hitch : hitches_) {
            hitch.tock();
        }
    }

} // namespace agent
