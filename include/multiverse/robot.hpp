#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types_basic.hpp"
#include "concord/types_circle.hpp"
#include "concord/types_polygon.hpp"
#include "multiverse/robot/chasis/chasis.hpp"
#include "multiverse/robot/sensor.hpp"
#include "multiverse/world.hpp"
#include "pigment/types_basic.hpp"

#include <rerun.hpp>
#include <vector>

namespace mvs {
    class Robot {
      private:
        std::string name;
        uint32_t group;
        bool controls_set = false;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<std::unique_ptr<Sensor>> sensors;
        std::unique_ptr<Chasis> chassis;

        std::vector<float> steerings, throttles;
        std::vector<float> steerings_max, throttles_max;

        muli::CollisionFilter filter;

        concord::Pose position;
        concord::Polygon shape;
        concord::Size size;
        pigment::RGB color;
        concord::Pose spawn_position;

        int wheel_nr;

      public:
        bool pulsining = false;
        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group);
        ~Robot();

        std::string id() const { return name; }
        const concord::Pose &get_position() const { return position; }

        void tick(float dt);
        void init(concord::Datum datum, concord::Pose pose, concord::Size size, pigment::RGB color, std::string name,
                  std::vector<concord::Bound> wheels = {}, std::vector<concord::Bound> karosseries = {});
        void set_controls(std::vector<float> steerings_max, std::vector<float> throttles_max);
        void reset_controls();

        void set_angular(float angular);
        void set_linear(float linear);

        void update(float angular, float linear);
        void teleport(concord::Pose pose);
        void pulse_vis(float p_s, float gps_mult = 5, float inc = 0.0015);
        void respawn();

      private:
        concord::Datum datum;
        concord::Circle pulse;
        concord::Circle pulse_gps;
        void visualize_once();
        void visualize();
    };
} // namespace mvs
