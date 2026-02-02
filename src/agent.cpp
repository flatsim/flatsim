#include "flatsim/agent.hpp"
#include "datapod/datapod.hpp"
#include "flatsim/simulator/machine.hpp"
#include "flatsim/utils.hpp"
#include <agent47/model/urdf.hpp>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace agent {

    static std::filesystem::path find_urdf(const std::filesystem::path &path) {
        // If absolute or exists, use as-is
        if (path.is_absolute() || std::filesystem::exists(path)) {
            return path;
        }
        // Check multiple locations for relative paths
        std::vector<std::filesystem::path> search_paths = {
            path,                                  // Current directory
            std::filesystem::path("..") / path,    // Parent (running from build/)
            std::filesystem::path("../..") / path, // Two levels up
        };
        for (const auto &p : search_paths) {
            if (std::filesystem::exists(p)) {
                return std::filesystem::canonical(p);
            }
        }
        throw std::runtime_error("URDF not found: " + path.string());
    }

    datapod::robot::Model Agent::load_model_from_urdf(const std::filesystem::path &urdf_path) {
        auto resolved = find_urdf(urdf_path);
        std::ifstream file(resolved);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open URDF file: " + resolved.string());
        }
        std::stringstream buffer;
        buffer << file.rdbuf();

        auto result = agent47::from_urdf_string(dp::String(buffer.str().c_str()));
        if (result.is_err()) {
            throw std::runtime_error("Failed to parse URDF: " + urdf_path.string());
        }
        return result.value();
    }

    static drivekit::RobotConstraints build_constraints(const types::Machine &config) {
        drivekit::RobotConstraints constraints;

        // Determine steering type
        bool is_differential = true;
        for (const auto &wheel : config.wheels) {
            if (std::abs(wheel.steering_max) > 1e-6f) {
                is_differential = false;
                break;
            }
        }
        constraints.steering_type =
            is_differential ? drivekit::SteeringType::DIFFERENTIAL : drivekit::SteeringType::ACKERMANN;

        // Derive geometry from wheel positions
        if (!config.wheels.empty()) {
            double max_y = -1e9, min_y = 1e9, max_x = -1e9, min_x = 1e9;
            for (const auto &w : config.wheels) {
                max_y = std::max(max_y, static_cast<double>(w.bound.pose.point.y));
                min_y = std::min(min_y, static_cast<double>(w.bound.pose.point.y));
                max_x = std::max(max_x, static_cast<double>(w.bound.pose.point.x));
                min_x = std::min(min_x, static_cast<double>(w.bound.pose.point.x));
            }
            constraints.wheelbase = std::max(0.1, std::abs(max_y - min_y));
            constraints.track_width = std::max(0.1, std::abs(max_x - min_x));
        } else {
            constraints.wheelbase = 1.5;
            constraints.track_width = 1.5;
        }

        // Normalized velocity units [-1, 1]
        constraints.max_linear_velocity = 1.0;
        constraints.min_linear_velocity = -1.0;
        constraints.max_linear_acceleration = 1.0;
        constraints.max_angular_velocity = 1.0;

        // Steering limits
        double max_steer = 0.0;
        for (const auto &wheel : config.wheels) {
            max_steer = std::max(max_steer, static_cast<double>(std::abs(wheel.steering_max)));
        }
        constraints.max_steering_angle = max_steer > 0 ? max_steer : 30.0 * M_PI / 180.0;
        constraints.max_steering_rate = 1.0;

        constraints.min_turning_radius = config.turning_radius;
        constraints.robot_length = config.bound.size.y;
        constraints.robot_width = config.bound.size.x;

        return constraints;
    }

    static void init_tracker(std::unique_ptr<drivekit::Tracker> &tracker, const types::Machine &config,
                             std::shared_ptr<rerun::RecordingStream> rec) {
        tracker = std::make_unique<drivekit::Tracker>(drivekit::TrackerType::PID);
        auto constraints = build_constraints(config);
        tracker->init(constraints, rec, config.uuid);

        drivekit::ControllerConfig ctrl_config;
        ctrl_config.allow_reverse = false;
        tracker->get_controller()->set_config(ctrl_config);
    }

    // Constructor for local mode
    Agent::Agent(const types::Machine &config, std::shared_ptr<rerun::RecordingStream> rec)
        : local_mode_(true), rec_(rec), spawned_(true), config_(config) {
        world_pose_ = config_.bound.pose;
        init_tracker(tracker_, config_, rec_);
    }

    // Constructor for local mode with agent47 bridge
    Agent::Agent(dp::String urdf_path, dp::robot::Identity identity, agent47::Bridge *bridge,
                 std::shared_ptr<rerun::RecordingStream> rec)
        : local_mode_(true), rec_(rec), spawned_(true) {

        agent47_ = std::make_shared<agent47::Agent>(urdf_path, identity, bridge);
        config_ = simulator::machine_from_model(agent47_->model_.model, datapod::Pose{}, std::nullopt);
        world_pose_ = config_.bound.pose;
        init_tracker(tracker_, config_, rec_);
    }

    // Constructor for networked mode via agent47 PipeBridge
    Agent::Agent(const types::Machine &config, const std::string &agent47_endpoint,
                 std::shared_ptr<rerun::RecordingStream> rec)
        : local_mode_(false), agent47_endpoint_(agent47_endpoint), rec_(rec), spawned_(false), config_(config) {

        world_pose_ = config_.bound.pose;
        init_tracker(tracker_, config_, rec_);

        // Create agent47 wrapper with an owned PipeBridge
        auto *bridge = new agent47::PipeBridge();
        agent47_ = std::make_shared<agent47::Agent>(dp::robot::Robot{}, bridge);
    }

    Agent::~Agent() {
        if (!local_mode_ && spawned_) {
            despawn();
        }
    }

    bool Agent::spawn() {
        if (local_mode_) {
            std::cerr << "[Agent] spawn() should not be called in LOCAL mode" << std::endl;
            return false;
        }

        if (!agent47_ || !agent47_->bridge_) {
            std::cerr << "[Agent] agent47 bridge not configured" << std::endl;
            return false;
        }
        if (!agent47_endpoint_.has_value()) {
            std::cerr << "[Agent] missing agent47 endpoint" << std::endl;
            return false;
        }

        if (!agent47_->bridge_->connect(*agent47_endpoint_)) {
            std::cerr << "[Agent] failed to connect agent47 bridge to " << *agent47_endpoint_ << std::endl;
            return false;
        }

        spawned_ = true;
        return true;
    }

    bool Agent::despawn() {
        if (!spawned_ || local_mode_) {
            return false;
        }

        if (agent47_ && agent47_->bridge_) {
            agent47_->bridge_->disconnect();
        }
        spawned_ = false;
        return true;
    }

    void Agent::tick(float dt, int timeout_ms) {
        if (!spawned_) {
            return;
        }

        if (local_mode_) {
            // LOCAL MODE: State already updated via update_from_physics()
            // Run tracker if enabled
            if (tracker_enabled_ && tracker_) {
                drivekit::RobotState state;
                state.pose = world_pose_;
                state.velocity.linear = linear_velocity_;
                state.velocity.angular = angular_velocity_;
                state.timestamp = 0.0;

                auto cmd = tracker_->tick(state, dt);
                if (cmd.valid) {
                    // Invert angular: drivekit uses CCW positive, simulator uses CW positive
                    cmd_linear_ = static_cast<float>(cmd.linear_velocity);
                    cmd_angular_ = -static_cast<float>(cmd.angular_velocity);
                }
            }
            return;
        }

        // NETWORKED MODE: agent47 PipeBridge
        if (!agent47_ || !agent47_->bridge_) {
            return;
        }

        // Drain sensor packets
        for (int i = 0; i < 16; ++i) {
            agent47::types::SensorPacket pkt;
            if (!agent47_->bridge_->sensor(pkt, 0)) {
                break;
            }
            try {
                if (pkt.kind == agent47::types::SensorKind::Lidar) {
                    auto lidar = dp::deserialize<dp::Mode::WITH_VERSION, types::ser::LidarData>(pkt.payload);
                    sensor_data_.lidar = lidar.to_lidar();
                    sensor_data_.has_lidar = true;
                } else if (pkt.kind == agent47::types::SensorKind::Gnss) {
                    auto gps = dp::deserialize<dp::Mode::WITH_VERSION, types::ser::GpsData>(pkt.payload);
                    sensor_data_.gps = gps.to_gps();
                    sensor_data_.has_gps = true;
                } else if (pkt.kind == agent47::types::SensorKind::Imu) {
                    auto imu = dp::deserialize<dp::Mode::WITH_VERSION, types::ser::ImuData>(pkt.payload);
                    sensor_data_.imu = imu.to_imu();
                    sensor_data_.has_imu = true;
                }
            } catch (...) {
                // ignore malformed packets
            }
        }

        // Receive feedback (pose/twist) from simulator
        dp::Stamp<agent47::types::Feedback> fb;
        if (agent47_->bridge_->recv(fb, timeout_ms)) {
            world_pose_.point.x = fb.value.pose.point.x;
            world_pose_.point.y = fb.value.pose.point.y;
            world_pose_.rotation = fb.value.pose.rotation;

            float yaw = static_cast<float>(fb.value.pose.rotation.to_euler().yaw);
            float vx = static_cast<float>(fb.value.twist.linear.vx);
            float vy = static_cast<float>(fb.value.twist.linear.vy);
            linear_velocity_ = vx * std::cos(yaw) + vy * std::sin(yaw);
            angular_velocity_ = static_cast<float>(fb.value.twist.angular.vz);
        }

        // Run tracker if enabled
        if (tracker_enabled_ && tracker_) {
            drivekit::RobotState state;
            state.pose = world_pose_;
            state.velocity.linear = linear_velocity_;
            state.velocity.angular = angular_velocity_;
            state.timestamp = 0.0;

            auto cmd = tracker_->tick(state, dt);
            if (cmd.valid) {
                cmd_linear_ = static_cast<float>(cmd.linear_velocity);
                cmd_angular_ = -static_cast<float>(cmd.angular_velocity);
            }
        }

        // Send twist command
        dp::Stamp<agent47::types::Command> cmd;
        cmd.timestamp = dp::Stamp<agent47::types::Command>::now();
        cmd.value.valid = true;
        cmd.value.twist.linear.vx = cmd_linear_ * speed_scale_;
        cmd.value.twist.angular.vz = cmd_angular_;
        agent47_->bridge_->send(cmd);
    }

    void Agent::tock() {
        if (!spawned_) {
            return;
        }

        // Visualize tracker path if enabled
        if (tracker_enabled_ && tracker_) {
            tracker_->tock();
        }
    }

    void Agent::update_from_physics(const types::ser::MachineState &state) {
        world_pose_ = state.pose.to_datapod();
        float yaw = static_cast<float>(world_pose_.rotation.to_euler().yaw);
        linear_velocity_ = state.velocity.x * std::cos(yaw) + state.velocity.y * std::sin(yaw);
        angular_velocity_ = state.angular_vel;
    }

    void Agent::update_from_sensors(const types::ser::SensorState &state) { sensor_data_ = state.to_sensor_data(); }

} // namespace agent
