#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types_basic.hpp"
#include "multiverse/robot/driver.hpp"
#include "multiverse/robot/sensor.hpp"
#include "multiverse/world.hpp"
#include "pigment/types_basic.hpp"

#include <rerun.hpp>

namespace mvs {
    class Robot {
      private:
        std::string name;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<mvs::World> world;
        std::vector<std::unique_ptr<Sensor>> sensors;
        std::unique_ptr<Vehicle> chassis;

        concord::Pose position;
        concord::Size size;
        pigment::RGB color;

      public:
        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<mvs::World> world);
        ~Robot();

        std::string id() const { return name; }
        const concord::Pose &get_position() const { return position; }

        void tick(float dt);
        void init(concord::Pose, pigment::RGB color, std::string name);
        void teleport(float x, float y);
        void update(float steering, float throttle);
        void update(float steering[4], float throttle[4]);

      private:
        void visualize_once();
        void visualize();
    };
} // namespace mvs
