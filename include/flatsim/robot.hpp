#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "flatsim/exceptions.hpp"
#include "flatsim/robot/chassis/chassis.hpp"
#include "flatsim/robot/power.hpp"
#include "flatsim/robot/sensor.hpp"
#include "flatsim/robot/sensors/gps_sensor.hpp"
#include "flatsim/robot/tank.hpp"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/world.hpp"

#include <memory>
#include <optional>
#include <vector>

namespace fs {
    class Robot {
      private:
        bool pulsing = false;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<std::unique_ptr<Sensor>> sensors;
        std::unique_ptr<Chassis> chassis;
        std::optional<std::unique_ptr<Power>> power;
        std::optional<Tank> tank;
        std::vector<std::shared_ptr<Robot>> slaves;
        std::vector<float> steerings, throttles;
        std::vector<float> steerings_max, throttles_max;
        std::vector<float> steerings_diff;

        muli::CollisionFilter filter;
        concord::Pose spawn_position;

        // Connection tracking
        Robot *connected_slave = nullptr;
        muli::RevoluteJoint *connection_joint = nullptr;

      public:
        RobotInfo info;
        OP mode = OP::IDLE;
        RobotRole role;

        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group);
        ~Robot();

        void tick(float dt);
        void init(concord::Datum datum, RobotInfo robo);
        void reset_controls();
        void set_angular(float angular);
        void set_linear(float linear);
        void respawn();
        void update(float angular, float linear);
        void teleport(concord::Pose pose);
        void visualize_pulse(float p_s, float gps_mult = 5, float inc = 0.0015);

        // Sensor management
        void add_sensor(std::unique_ptr<Sensor> sensor);
        template <typename T> T *get_sensor() const {
            for (const auto &sensor : sensors) {
                if (!sensor) {
                    continue; // Skip null sensors
                }
                T *typed_sensor = dynamic_cast<T *>(sensor.get());
                if (typed_sensor) {
                    return typed_sensor;
                }
            }
            return nullptr;
        }
        Sensor *get_sensor(const std::string &type) const;

        const concord::Pose &get_position() const { return info.bound.pose; }
        void pulse() { pulsing = true; }
        void toggle_section_work(const std::string &karosserie_name, int section_id) { chassis->toggle_section_work(karosserie_name, section_id); }
        void toggle_all_sections_work(const std::string &karosserie_name) { chassis->toggle_all_sections_work(karosserie_name); }
        void toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id) { chassis->toggle_all_except_section_work(karosserie_name, except_section_id); }
        std::vector<Karosserie> *get_karosseries() {
            if (!chassis) {
                throw NullPointerException("chassis");
            }
            return &chassis->karosseries;
        }

        // Tank management
        bool has_tank() const { return tank.has_value(); }
        Tank *get_tank() { return tank.has_value() ? &tank.value() : nullptr; }
        const Tank *get_tank() const { return tank.has_value() ? &tank.value() : nullptr; }
        void empty_tank() {
            if (tank.has_value())
                tank->empty_all();
        }
        void fill_tank(float amount) {
            if (tank.has_value())
                tank->fill(amount);
        }

        // Connection management
        bool try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>> &all_robots);
        void disconnect_trailer();
        bool is_connected() const;

        // Power management
        bool has_power() const { return power.has_value(); }
        Power *get_power() const { return power ? power->get() : nullptr; }
        bool is_powered() const { return power && *power && !(*power)->is_empty(); }
        float get_power_percentage() const { return (power && *power) ? (*power)->get_percentage() : 0.0f; }

      private:
        concord::Datum datum;
        concord::Circle pulse_enu;
        concord::Circle pulse_gps;
        void visualize_once();
        void visualize();
    };
} // namespace fs
