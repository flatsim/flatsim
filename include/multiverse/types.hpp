#pragma once

#include "concord/types_basic.hpp"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

namespace mvs {
    // ROBOT
    struct Controllz {
        std::vector<float> steerings_max;
        std::vector<float> throttles_max;
        std::vector<float> steerings_diff;
    };
    struct Robo {
        uint RCI;
        std::string name;
        std::string uuid;
        pigment::RGB color;
        concord::Bound bound;
        std::vector<concord::Bound> wheels;
        Controllz controlz;
        std::vector<concord::Bound> karosserie;
    };

    enum class OperationMode { IDLE, TRANSPORT, WORK };

    // WORLD
    struct Layz {
        bool centered = false;
        std::string name;
        std::string uuid;
        pigment::RGB color;
        concord::Bound field;
        float resolution;
    };

} // namespace mvs
