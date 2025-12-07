#pragma once

#include "flatsim/protocol/types.hpp"
#include "flatsim/robot.hpp"
#include "flatsim/world.hpp"

#include <memory>
#include <vector>

namespace fs::server {

    /**
     * @brief Physics-centric environment server.
     *
     * Owns the world and robot instances and provides a transport-agnostic
     * interface in terms of protocol::RobotCommand / protocol::RobotState.
     *
     * Initial implementation is a light wrapper around existing World/Robot
     * usage in fs::Simulator.
     */
    class EnvironmentServer {
      public:
        EnvironmentServer(std::shared_ptr<rerun::RecordingStream> rec);

        // World & robot setup
        void init(concord::Datum datum, concord::Size world_size);
        void add_robot(RobotInfo robot_info);

        // Step physics forward by dt, applying the given commands.
        // Populates out_states with the resulting robot states.
        void step(float dt, const std::vector<protocol::RobotCommand> &commands,
                  std::vector<protocol::RobotState> &out_states);

        // Accessors
        World &world();
        const World &world() const;

        std::vector<std::shared_ptr<Robot>> &robots();
        const std::vector<std::shared_ptr<Robot>> &robots() const;

      private:
        std::shared_ptr<rerun::RecordingStream> rec_;
        std::shared_ptr<World> world_;
        concord::Datum world_datum_;
        std::vector<std::shared_ptr<Robot>> robots_;
    };

} // namespace fs::server
