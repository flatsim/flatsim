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
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<std::unique_ptr<Sensor>> sensors;
        std::unique_ptr<Chasis> chassis;

        muli::CollisionFilter filter;

        concord::Pose position;
        concord::Polygon shape;
        concord::Size size;
        pigment::RGB color;
        concord::Pose spawn_position;

      public:
        bool pulsining = false;
        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group);
        ~Robot();

        std::string id() const { return name; }
        const concord::Pose &get_position() const { return position; }

        void tick(float dt);
        void init(concord::Datum datum, concord::Pose pose, concord::Size size, pigment::RGB color, std::string name,
                  std::vector<concord::Size> wheel_sizes = {}, std::vector<concord::Size> karosserie_sizes = {});
        void update(float steering, float throttle);
        void update(float steering[4], float throttle[4]);
        void teleport(concord::Pose pose);
        void pulse_vis(float p_s);
        void respawn();

      private:
        concord::Datum datum;
        concord::Circle pulse;
        void visualize_once();
        void visualize();
    };
} // namespace mvs
