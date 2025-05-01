#pragma once

#include "muli/math.h"
#include "muli/polygon.h"
#include "muli/rigidbody.h"
#include "muli/settings.h"
#include "muli/world.h"

#include "concord/types_basic.hpp"
#include "multiverse/driver/controller.hpp"
#include "multiverse/sensors/sensor.hpp"

#include <rerun.hpp>

namespace mvs {
    class Robot {
      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        std::vector<std::unique_ptr<Sensor>> sensors;
        std::unique_ptr<VehicleController> chassis;

        concord::Pose position;

      public:
        Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world);
        ~Robot();

        void tick(float dt);
        void init(concord::Pose, std::string name);

        void teleport(float x, float y);
        const concord::Pose &get_position() const { return position; }

      private:
        void visualize_once();
        void visualize();
    };
} // namespace mvs
