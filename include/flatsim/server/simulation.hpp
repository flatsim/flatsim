#pragma once

#include "flatsim/server/environment.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace fs::server {

    /**
     * @brief Simple simulation loop helper for EnvironmentServer.
     *
     * Runs a physics loop on the calling thread and periodically calls
     * world/robot tock() for visualization.
     *
     * @param env EnvironmentServer instance
     * @param user_loop Callback invoked each step with (dt, states).
     *                  Return false to stop the loop.
     * @param viz_fps Visualization rate (calls world.tock()/robot.tock())
     */
    void run(EnvironmentServer &env,
             const std::function<bool(float, const std::vector<protocol::RobotState> &)> &user_loop, int viz_fps = 30);

} // namespace fs::server
