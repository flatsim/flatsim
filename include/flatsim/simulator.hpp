#pragma once

#include "flatsim/exceptions.hpp"
#include "flatsim/robot.hpp"
#include "flatsim/types.hpp"
#include "flatsim/world.hpp"

#include "muli/world.h"
#include <atomic>
#include <chrono>
#include <optional>
#include <rerun.hpp>
#include <thread>

namespace fs {
    class Simulator {
      public:
        std::shared_ptr<World> world;
        concord::Datum world_datum;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::vector<std::shared_ptr<Robot>> robots;

      private:
        bool controls_set = false;
        std::shared_ptr<muli::World> physics_world;
        int selected_robot_idx = -1;
        uint ticks = 0;
        uint tocks = 0;

      public:
        Simulator(std::shared_ptr<rerun::RecordingStream> rec);
        ~Simulator();

        void init(concord::Datum datum, concord::Size world_size);
        void tick(float dt);
        void tock(int rate);

        // Threading method that combines tick/tock with user loop
        template <typename UserLoop> void ticktock(UserLoop user_loop, int viz_fps = 30);

        // ROBOT
        void add_robot(RobotInfo robot_info);
        void set_controls(uint robot_idx, float steering, float throttle);
        void toggle_section_work(uint robot_idx, const std::string &karosserie_name, int section_id);
        void toggle_all_sections_work(uint robot_idx, const std::string &karosserie_name);
        void toggle_all_except_section_work(uint robot_idx, const std::string &karosserie_name, int except_section_id);
        Robot &get_robot(uint i);
        Robot &get_robot(const std::string &uuid);
        int num_robots() const;

        // Spatial queries
        std::vector<std::pair<std::string, concord::Pose>> get_all_robot_poses() const;
        std::vector<Robot *> get_all_robots() const;
        Robot *get_closest_robot(const Robot &from_robot, float max_distance = 50.0f) const;
        // WORLD
        concord::Datum get_datum() const;
        World &get_world() { return *world; }
        Layer &get_layer(uint i);
        Layer &get_layer(const std::string &uuid);
        void add_layer(LayerInfo layer_info, bool noise = false);

        // RERUN MANAGEMENT
        void reset_recording();
        void clear_all_entities();
    };

    // Template implementation
    template <typename UserLoop> void Simulator::ticktock(UserLoop user_loop, int viz_fps) {
        std::atomic<bool> running{true};
        const auto viz_interval = std::chrono::milliseconds(1000 / viz_fps);

        // Background visualization thread - reads data only
        std::thread viz_thread([this, &running, viz_interval]() {
            while (running.load()) {
                auto viz_start = std::chrono::steady_clock::now();
                this->tock(1); // Always visualize when called

                // Maintain consistent frame rate
                auto viz_end = std::chrono::steady_clock::now();
                auto elapsed = viz_end - viz_start;
                if (elapsed < viz_interval) {
                    std::this_thread::sleep_for(viz_interval - elapsed);
                }
            }
        });

        // Main physics loop with user customization
        auto last_time = std::chrono::steady_clock::now();

        try {
            while (true) {
                // Calculate delta time
                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<float> dt = now - last_time;
                last_time = now;

                // Physics tick at full speed
                this->tick(dt.count());

                // User-defined logic (input handling, control logic, etc.)
                if (!user_loop(dt.count())) {
                    break; // User loop returns false to exit
                }

                // Small sleep to cap CPU usage
                std::this_thread::sleep_for(std::chrono::nanoseconds(100));
            }
        } catch (...) {
            running.store(false);
            if (viz_thread.joinable()) {
                viz_thread.join();
            }
            throw; // Re-throw the exception
        }

        // Clean shutdown
        running.store(false);
        if (viz_thread.joinable()) {
            viz_thread.join();
        }
    }

} // namespace fs
