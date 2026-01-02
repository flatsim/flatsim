#include "flatsim/simulator/machine.hpp"
#include "flatsim/gps.hpp"
#include "flatsim/utils.hpp"
#include <cmath>
#include <iostream>

namespace simulator {

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world,
                     const types::Machine &config, uint32_t group)
        : rec_(rec), world_(world), config_(config) {
        // Create collision filter (use bit/mask system)
        filter_.group = 0;
        filter_.bit = 1 << group;
        filter_.mask = ~(1 << group);
    }

    void Machine::create() {
        if (!world_) {
            std::cerr << "[Simulator] Cannot create machine - missing world" << std::endl;
            return;
        }

        // Create chassis which manages all physics
        chassis_ = std::make_unique<Chassis>(world_, rec_, filter_, &config_, &state_);
        chassis_->init(config_);

        std::cout << "[Simulator] Created machine: " << config_.name << " with " << config_.wheels.size() << " wheels"
                  << std::endl;
    }

    void Machine::destroy() {
        // Clear Rerun visualization for this machine's namespace recursively
        if (rec_) {
            rec_->log(config_.uuid, rerun::Clear::RECURSIVE);
            rec_->log_with_static(config_.uuid, true, rerun::Clear::RECURSIVE);
        }

        if (chassis_) {
            chassis_->destroy();
        }
        chassis_.reset();

        std::cout << "[Simulator] Destroyed machine: " << config_.uuid << std::endl;
    }

    void Machine::apply_control(const types::WheelControl &control, float dt) {
        if (!chassis_) return;

        // Apply brake if requested
        if (control.brake > 0.0f) {
            chassis_->brake(control.brake);
            return;
        }

        // Debug: print controls every 60 calls
        static int ctrl_debug = 0;
        if (ctrl_debug++ % 60 == 0) {
            std::cout << "[Machine::apply_control] steer=[";
            for (size_t i = 0; i < control.steering.size(); ++i) {
                std::cout << control.steering[i];
                if (i < control.steering.size() - 1) std::cout << ",";
            }
            std::cout << "] throttle=[";
            for (size_t i = 0; i < control.throttle.size(); ++i) {
                std::cout << control.throttle[i];
                if (i < control.throttle.size() - 1) std::cout << ",";
            }
            std::cout << "]" << std::endl;
        }

        // Apply per-wheel steering and throttle
        chassis_->update(control.steering, control.throttle, dt);
    }

    void Machine::tick(float dt) {
        if (!chassis_) return;

        // Update pose from physics
        config_.bound.pose = chassis_->get_pose();

        // Tick chassis (updates wheels, karosseries, hitches)
        chassis_->tick(dt);
    }

    void Machine::tock(datapod::Geo datum) {
        if (!chassis_) return;
        if (!rec_) return;

        // Create label with role info
        std::string role_prefix;
        switch (config_.role) {
        case types::MachineRole::MASTER:
            role_prefix = "(M)";
            break;
        case types::MachineRole::FOLLOWER:
            role_prefix = "(F)";
            break;
        case types::MachineRole::SLAVE:
            role_prefix = "(S)";
            break;
        }
        std::string label = role_prefix + config_.uuid;

        chassis_->tock(label);

        // GPS coordinates visualization - use current body position
        if (chassis_->body) {
            auto x = chassis_->body->GetPosition().x;
            auto y = chassis_->body->GetPosition().y;
            datapod::Point current_pos{x, y, 0.0};
            auto wgs_coords = flatsim::gps::enu_to_gps(current_pos, datum);
            rec_->log_static(config_.uuid + "/gps",
                             rerun::GeoPoints({{wgs_coords.latitude, wgs_coords.longitude}})
                                 .with_colors({rerun::Color(config_.color.r, config_.color.g, config_.color.b)}));
        }
    }

    types::ser::MachineState Machine::get_state() const {
        types::ser::MachineState ms;
        ms.uuid = datapod::String(config_.uuid);

        if (chassis_ && chassis_->body) {
            ms.pose.position.x = chassis_->body->GetPosition().x;
            ms.pose.position.y = chassis_->body->GetPosition().y;
            // Add M_PI/2 to convert from body angle to world heading
            // (model forward is +Y, so body angle 0 = heading +90deg)
            ms.pose.angle = chassis_->body->GetAngle() + M_PI / 2;
            ms.velocity.x = chassis_->body->GetLinearVelocity().x;
            ms.velocity.y = chassis_->body->GetLinearVelocity().y;
            ms.angular_vel = chassis_->body->GetAngularVelocity();
        }

        return ms;
    }

    Hitch *Machine::find_hitch(const std::string &name) {
        if (!chassis_) return nullptr;

        for (auto &hitch : chassis_->hitches) {
            if (hitch.name == name) {
                return &hitch;
            }
        }
        return nullptr;
    }

    void Machine::teleport(const datapod::Pose &pose) {
        if (chassis_) {
            chassis_->teleport(pose);
        }
    }

    void Machine::brake(float brake_force) {
        if (chassis_) {
            chassis_->brake(brake_force);
        }
    }

    void Machine::update_color(const pigment::RGB &new_color) {
        config_.color = new_color;
        if (chassis_) {
            chassis_->update_color(new_color);
        }
    }

    void Machine::update_sensors(Data &data, const datapod::Geo &datum, float dt) {
        if (!chassis_ || !chassis_->body) return;

        // Get current state
        auto pose = chassis_->get_pose();

        // IMPORTANT: The physics body's angle 0 = +X direction, but the model's forward is +Y.
        // So we add M_PI/2 to convert body angle to heading (same as get_state() does for the agent).
        float heading = utils::get_yaw(pose) + M_PI / 2.0f;

        auto vel = chassis_->body->GetLinearVelocity();
        float linear_vel = vel.x * std::cos(heading) + vel.y * std::sin(heading);
        float angular_vel = chassis_->body->GetAngularVelocity();

        // GPS - always fill if datum is set
        if (datum.is_set()) {
            // Use corrected heading for GPS
            datapod::Pose gps_pose = pose;
            utils::set_yaw(gps_pose, heading);
            sensor_data_.gps = data.pose_to_gps(gps_pose, std::abs(linear_vel));
            sensor_data_.has_gps = true;
        }

        // IMU - always fill
        sensor_data_.imu = data.compute_imu(pose, linear_vel, angular_vel, prev_linear_vel_, dt);
        sensor_data_.has_imu = true;
        prev_linear_vel_ = linear_vel;

        // LIDAR - only if configured
        if (config_.lidar.has_value() && config_.lidar->enabled) {
            const auto &lidar_cfg = config_.lidar.value();
            // Use corrected heading for LIDAR scan direction
            datapod::Pose lidar_pose = pose;
            utils::set_yaw(lidar_pose, heading);
            sensor_data_.lidar = data.scan_lidar(lidar_pose, lidar_cfg.min_range, lidar_cfg.max_range,
                                                 lidar_cfg.fov_deg, lidar_cfg.resolution_deg, filter_);
            sensor_data_.has_lidar = true;
        }
    }

} // namespace simulator
