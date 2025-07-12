#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "flatsim/exceptions.hpp"
#include "flatsim/robot/chassis/chassis.hpp"
#include "flatsim/robot/controller.hpp"
#include "flatsim/robot/power.hpp"
#include "flatsim/robot/sensor.hpp"
#include "flatsim/robot/sensors/gps_sensor.hpp"
#include "flatsim/robot/systems/chain.hpp"
#include "flatsim/robot/systems/control.hpp"
#include "flatsim/robot/tank.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/world.hpp"

#include <memory>
#include <optional>
#include <vector>

namespace fs {
    // Forward declaration to avoid circular dependency
    class Simulator;

    class Robot {
        friend class ControlSystem;
        friend class ChainManager;
        friend class NavigationController;

      private:
        bool pulsing = false;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        Simulator *simulator = nullptr;
        std::vector<std::unique_ptr<Sensor>> sensors;
        std::unique_ptr<Chassis> chassis;
        std::optional<std::unique_ptr<Power>> power;
        std::optional<Tank> tank;
        std::vector<std::shared_ptr<Robot>> slaves; // Legacy - can be removed later

        muli::CollisionFilter filter;
        concord::Pose spawn_position;
        pigment::RGB original_color; // Store original color for restoration when disconnected

        // New modular systems
        std::unique_ptr<ControlSystem> control_system;
        std::unique_ptr<ChainManager> chain_manager;
        std::unique_ptr<NavigationController> navigation_controller;

      public:
        RobotInfo info;
        OP mode = OP::IDLE;
        RobotRole role;

        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group);
        ~Robot();

        void init(concord::Datum datum, RobotInfo robo);
        void tick(float dt);
        void tock();

        void reset_controls();
        void set_angular(float angular);
        void set_linear(float linear);

        // Control propagation methods for chain control
        void set_angular_as_follower(float angular, const Robot &master);
        void set_linear_as_follower(float linear, const Robot &master);
        void respawn();
        void update(float angular, float linear);
        void teleport(concord::Pose pose);
        void teleport(concord::Pose pose, bool propagate);
        void visualize_pulse(float p_s, float gps_mult = 5, float inc = 0.0015);
        void update_color(const pigment::RGB &new_color);

        // Sensor management
        void add_sensor(std::unique_ptr<Sensor> sensor);
        template <typename T> T *get_sensor() const {
            for (const auto &sensor : sensors) {
                if (!sensor) continue;
                T *typed_sensor = dynamic_cast<T *>(sensor.get());
                if (typed_sensor) return typed_sensor;
            }
            return nullptr;
        }
        Sensor *get_sensor(const std::string &type) const;

        const concord::Pose &get_position() const { return info.bound.pose; }
        const concord::Pose &get_spawn_position() const { return spawn_position; }
        void pulse() { pulsing = true; }
        void toggle_section_work(const std::string &karosserie_name, int section_id) {
            chassis->toggle_section_work(karosserie_name, section_id);
        }
        void toggle_all_sections_work(const std::string &karosserie_name) {
            chassis->toggle_all_sections_work(karosserie_name);
        }
        void toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id) {
            chassis->toggle_all_except_section_work(karosserie_name, except_section_id);
        }
        std::vector<Karosserie> *get_karosseries() {
            if (!chassis) throw NullPointerException("chassis");
            return &chassis->karosseries;
        }

        // Tank management
        bool has_tank() const { return tank.has_value(); }
        Tank *get_tank() { return tank.has_value() ? &tank.value() : nullptr; }
        const Tank *get_tank() const { return tank.has_value() ? &tank.value() : nullptr; }
        void empty_tank() {
            if (tank.has_value()) tank->empty_all();
        }
        void fill_tank(float amount) {
            if (tank.has_value()) tank->fill(amount);
        }

        // Connection management - delegate to ChainManager
        bool try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>> &all_robots);
        bool try_connect_nearby();
        bool try_connect_from_chain_end();
        void disconnect_trailer();
        void disconnect_all_followers();
        void disconnect_last_follower();
        void disconnect_at_position(int position);
        void disconnect_from_position(int position);
        bool is_connected() const;

        // Chain management - delegate to ChainManager
        std::vector<Robot *> get_connected_followers() const;
        Robot *get_master_robot() const;
        bool is_follower() const;
        Robot *get_root_master() const;
        std::vector<Robot *> get_full_chain() const;
        int get_chain_length() const;
        int get_position_in_chain() const;
        void print_chain_status() const;

        // Capability management - delegate to ChainManager
        void update_follower_capabilities();
        const FollowerCapabilities &get_follower_capabilities() const;
        bool has_steering_capability() const;
        bool has_throttle_capability() const;
        bool has_available_master_hitches() const;

        // Power management
        bool has_power() const { return power.has_value(); }
        Power *get_power() const { return power ? power->get() : nullptr; }
        bool is_powered() const { return power && *power && !(*power)->is_empty(); }
        float get_power_percentage() const { return (power && *power) ? (*power)->get_percentage() : 0.0f; }

        // Spatial queries - robot can find other robots
        std::vector<Robot *> get_all_robots() const;
        Robot *get_closest_robot(float max_distance = 50.0f) const;

        // Set simulator reference (called by simulator when robot is added)
        void set_simulator(Simulator *sim) { simulator = sim; }

        // Navigation control methods
        void set_navigation_goal(const NavigationGoal &goal);
        void set_navigation_path(const PathGoal &path);
        void clear_navigation_goal();
        void clear_navigation_path();
        bool is_navigation_goal_reached() const;
        bool is_navigation_path_completed() const;
        float get_distance_to_navigation_goal() const;
        concord::Point get_current_navigation_target() const;
        void set_navigation_controller_type(ControllerType type);
        void emergency_navigation_stop();

      private:
        concord::Datum datum;
        concord::Circle pulse_enu;
        concord::Circle pulse_gps;
    };
} // namespace fs
