#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "flatsim/exceptions.hpp"
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
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/world.hpp"
#include "navcon.hpp"

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
        std::unique_ptr<navcon::Navcon> navcon;

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
        void toggle_section_work(const std::string &karosserie_name, int section_id) {
            chassis.toggle_section_work(karosserie_name, section_id);
        }
        void toggle_all_sections_work(const std::string &karosserie_name) {
            chassis.toggle_all_sections_work(karosserie_name);
        }
        void toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id) {
            chassis.toggle_all_except_section_work(karosserie_name, except_section_id);
        }
        std::vector<Karosserie> *get_karosseries() {
            if (!chassis.exists()) throw NullPointerException("chassis");
            return chassis.get_karosseries();
        }

        // Spatial queries - robot can find other robots
        std::vector<Robot *> get_all_robots() const;
        Robot *get_closest_robot(float max_distance = 50.0f) const;

        // Set simulator reference (called by simulator when robot is added)
        void set_simulator(Simulator *sim) { simulator = sim; }

        // Simple helper method to update navigation and apply velocity commands
        void update_navigation(float dt);
    };
} // namespace fs
