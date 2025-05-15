#pragma once

#include "concord/types_basic.hpp"
#include "concord/types_polygon.hpp"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

namespace mvs {
    // ROBOT
    struct RobotControll {
        std::vector<float> steerings_max;
        std::vector<float> throttles_max;
        std::vector<float> steerings_diff;
    };
    struct RobotInfo {
        uint RCI;
        std::string name;
        std::string uuid;
        pigment::RGB color;
        concord::Bound bound;
        std::vector<concord::Bound> wheels;
        RobotControll controlz;
        std::vector<concord::Bound> karosserie;
    };

    enum class OperationMode { IDLE, TRANSPORT, WORK };

    // WORLD
    struct LayerInfo {
        std::string name;
        std::string uuid;
        pigment::RGB color;
        concord::Bound bound;
        concord::Polygon field;
        float resolution;
        bool centered = false;
    };

} // namespace mvs
