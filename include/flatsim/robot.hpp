#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "drivekit.hpp"
#include "flatsim/core/exceptions.hpp"
#include "flatsim/core/utils.hpp"
#include "flatsim/robot/chain_manager.hpp"
#include "flatsim/robot/chassis_manager.hpp"
#include "flatsim/robot/control_manager.hpp"
#include "flatsim/robot/network_manager.hpp"
#include "flatsim/robot/power/power.hpp"
#include "flatsim/robot/power_manager.hpp"
#include "flatsim/robot/sensor/gps_sensor.hpp"
#include "flatsim/robot/sensor/sensor.hpp"
#include "flatsim/robot/sensor_manager.hpp"
#include "flatsim/robot/tank/tank.hpp"
#include "flatsim/robot/tank_manager.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/world.hpp"

#include <memory>
#include <optional>
#include <vector>

namespace fs {
    class Simulator;

    class Robot {
        friend class ControlManager;
        friend class ChainManager;

      private:
        bool pulsing = false;
        bool navigation_enabled = true;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        Simulator *simulator = nullptr;
        std::vector<std::shared_ptr<Robot>> slaves; // Legacy - can be removed later

        muli::CollisionFilter filter;
        concord::Pose spawn_position;
        pigment::RGB original_color; // Store original color for restoration when disconnected
        concord::Datum datum;
        concord::Circle pulse_enu;
        concord::Circle pulse_gps;

      public:
        RobotInfo info;
        RobotState state;

        // Device managers - direct public access
        SensorManager sensors;
        ControlManager controls;
        ChainManager chain;
        ChassisManager chassis;
        Network network;
        TankManager tank;
        PowerManager power;
        std::unique_ptr<drivekit::Tracker> tracker;

        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group);
        ~Robot();

        void init(concord::Datum datum, RobotInfo robo);
        void tick(float dt);
        void tock();
        void clean();

        void respawn();
        void update(float angular, float linear);
        void teleport(concord::Pose pose);
        void teleport(concord::Pose pose, bool propagate);
        void visualize_pulse(float p_s, float gps_mult = 5, float inc = 0.0015);
        void update_color(const pigment::RGB &new_color);

        const concord::Pose &get_position() const { return info.bound.pose; }
        const concord::Pose &get_spawn_position() const { return spawn_position; }
        void pulse() { pulsing = true; }

        // Spatial queries - robot can find other robots
        std::vector<Robot *> get_all_robots() const;
        Robot *get_closest_robot(float max_distance = 50.0f) const;

        // Set simulator reference (called by simulator when robot is added)
        void set_simulator(Simulator *sim) { simulator = sim; }

        // Enable or disable internal navigation (drivekit-based) for this robot.
        void set_navigation_enabled(bool enabled) { navigation_enabled = enabled; }
        bool is_navigation_enabled() const { return navigation_enabled; }

        // Approximate kinematic state from physics body.
        // linear: forward velocity in robot frame (m/s)
        // angular: yaw rate (rad/s)
        void get_velocity(double &linear, double &angular) const;

        // Simple helper method to update navigation and apply velocity commands
        void update_navigation(float dt);

        // Apply braking force to stop the robot
        void brake();

        /**
         * @brief Check if a point is within the robot's forward line of sight
         * @param point The point to check
         * @param sight_distance How far ahead to look (0 = use 4x robot length)
         * @param half_angle Half of the vision cone angle in radians (default: ~30 deg)
         * @return true if the point is within line of sight
         */
        bool in_line_of_sight(const concord::Point &point, float sight_distance = 0.0f, float half_angle = 0.52f) const;

        /**
         * @brief Check if another robot is within forward line of sight
         * @param other The other robot to check
         * @param sight_distance How far ahead to look (0 = use 4x robot length)
         * @param half_angle Half of the vision cone angle in radians (default: ~30 deg)
         * @return true if the other robot is within line of sight
         */
        bool robot_in_sight(const Robot &other, float sight_distance = 0.0f, float half_angle = 0.52f) const;

        /**
         * @brief Get all robots that are within forward line of sight
         * @param sight_distance How far ahead to look (0 = use 4x robot length)
         * @param half_angle Half of the vision cone angle in radians (default: ~30 deg)
         * @return Vector of pointers to robots in line of sight
         */
        std::vector<Robot *> get_robots_in_sight(float sight_distance = 0.0f, float half_angle = 0.52f) const;
    };
} // namespace fs
