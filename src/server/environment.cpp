#include "flatsim/server/environment.hpp"

#include <algorithm>
#include <execution>

namespace fs::server {

    EnvironmentServer::EnvironmentServer(std::shared_ptr<rerun::RecordingStream> rec) : rec_(std::move(rec)) {}

    void EnvironmentServer::init(concord::Datum datum, concord::Size world_size) {
        world_ = std::make_shared<fs::World>(rec_);
        world_->init(datum, world_size);
        world_datum_ = datum;
    }

    void EnvironmentServer::add_robot(RobotInfo robot_info) {
        if (!world_) {
            throw NullPointerException("world");
        }

        for (auto &robot : robots_) {
            if (robot && robot->info.uuid == robot_info.uuid) {
                return;
            }
        }

        auto physics_world = world_->get_world();
        if (!physics_world) {
            throw NullPointerException("physics_world");
        }

        robots_.emplace_back([&] {
            auto r = std::make_shared<Robot>(rec_, physics_world, robots_.size());
            std::string seqid = robot_info.type + "_" + std::to_string(robots_.size());
            robot_info.seqid = seqid;
            r->init(world_datum_, robot_info);
            return r;
        }());
    }

    void EnvironmentServer::step(float dt, const std::vector<protocol::RobotCommand> &commands,
                                 std::vector<protocol::RobotState> &out_states) {
        // Apply commands to matching robots
        for (const auto &cmd : commands) {
            for (auto &robot : robots_) {
                if (robot && robot->info.uuid == cmd.id) {
                    robot->update(cmd.steering, cmd.throttle);
                    break;
                }
            }
        }

        // World physics step
        if (!world_) {
            throw NullPointerException("world");
        }
        world_->tick(dt);

        // Robots step (physics + sensors, navigation, etc.)
        std::for_each(std::execution::par, robots_.begin(), robots_.end(), [dt](auto &robott) {
            if (!robott) return;
            robott->tick(dt);
        });

        // Collect resulting states
        out_states.clear();
        out_states.reserve(robots_.size());
        for (const auto &robot : robots_) {
            if (!robot) continue;
            protocol::RobotState state;
            state.id = robot->info.uuid;
            state.timestamp = 0.0; // caller can fill wall-clock if needed
            state.pose = robot->info.bound.pose;
            // Velocity is currently approximated / left at defaults; can be
            // refined once EnvironmentServer gains access to body velocities.
            out_states.push_back(state);
        }
    }

    World &EnvironmentServer::world() {
        if (!world_) {
            throw NullPointerException("world");
        }
        return *world_;
    }

    const World &EnvironmentServer::world() const {
        if (!world_) {
            throw NullPointerException("world");
        }
        return *world_;
    }

    std::vector<std::shared_ptr<Robot>> &EnvironmentServer::robots() { return robots_; }

    const std::vector<std::shared_ptr<Robot>> &EnvironmentServer::robots() const { return robots_; }

} // namespace fs::server

