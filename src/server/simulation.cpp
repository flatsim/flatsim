#include "flatsim/server/simulation.hpp"

namespace fs::server {

    void run(EnvironmentServer &env,
             const std::function<bool(float, const std::vector<protocol::RobotState> &)> &user_loop, int viz_fps) {
        std::vector<protocol::RobotState> states;
        std::vector<protocol::RobotCommand> commands;

        const auto viz_interval = std::chrono::milliseconds(1000 / std::max(viz_fps, 1));
        auto last_time = std::chrono::steady_clock::now();
        auto last_viz = last_time;

        while (true) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<float> dt = now - last_time;
            last_time = now;
            float dt_s = dt.count();
            if (dt_s <= 0.0f) dt_s = 0.016f;

            env.step(dt_s, commands, states);

            if (!user_loop(dt_s, states)) {
                break;
            }

            // Visualization at fixed rate
            if (now - last_viz >= viz_interval) {
                env.world().tock();
                for (auto &robot : env.robots()) {
                    if (robot) {
                        robot->tock();
                    }
                }
                last_viz = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

} // namespace fs::server
