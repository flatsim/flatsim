#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "multiverse/exceptions.hpp"
#include "multiverse/robot/chasis/chasis.hpp"
#include "multiverse/robot/sensor.hpp"
#include "multiverse/robot/sensors/gps_sensor.hpp"
#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"
#include "multiverse/world.hpp"

#include <vector>

namespace mvs {
    class Robot {
      private:
        bool pulsining = false;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<std::unique_ptr<Sensor>> sensors;
        std::unique_ptr<Chasis> chassis;
        std::vector<std::shared_ptr<Robot>> slaves;
        std::vector<float> steerings, throttles;
        std::vector<float> steerings_max, throttles_max;
        std::vector<float> steerings_diff;

        muli::CollisionFilter filter;
        concord::Pose spawn_position;

      public:
        RobotInfo info;
        OP mode = OP::IDLE;

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
        template<typename T>
        T* get_sensor() const {
            for (const auto& sensor : sensors) {
                if (!sensor) {
                    continue; // Skip null sensors
                }
                T* typed_sensor = dynamic_cast<T*>(sensor.get());
                if (typed_sensor) {
                    return typed_sensor;
                }
            }
            return nullptr;
        }
        Sensor* get_sensor(const std::string& type) const;

        const concord::Pose &get_position() const { return info.bound.pose; }
        void pulse() { pulsining = true; }
        void toggle_work(std::string karosserie_name) { chassis->toggle_work(karosserie_name); }
        std::vector<Karosserie> *get_karosseies() { 
            if (!chassis) {
                throw NullPointerException("chassis");
            }
            return &chassis->karosseriez; 
        }

      private:
        concord::Datum datum;
        concord::Circle pulse_enu;
        concord::Circle pulse_gps;
        void visualize_once();
        void visualize();
    };
} // namespace mvs
