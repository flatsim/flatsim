#pragma once

#include "multiverse/robot.hpp"
#include "multiverse/world.hpp"

#include "pigment/types_basic.hpp"

#include "muli/world.h"
#include <rerun.hpp>

namespace mvs {
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
        float steering = 0.0f, throttle = 0.0f;
        std::vector<float> steerings, throttles;
        std::vector<float> steerings_max, throttles_max;

      public:
        Simulator(std::shared_ptr<rerun::RecordingStream> rec);
        ~Simulator();
        void tick(float dt);
        void init(concord::Datum datum, concord::Size world_size, float grid_size);
        concord::Datum get_datum() const { return world_datum; }

        void add_robot(concord::Pose robot_pose, pigment::RGB robot_color, concord::Size chassis_size,
                       std::vector<concord::Bound> wheels, std::vector<concord::Bound> karosserie);

        void set_controls(std::vector<float> steerings_max, std::vector<float> throttles_max);

        void on_joystick_axis(int axis, float value);
        void on_joystick_button(int button, bool pressed);
    };
} // namespace mvs
