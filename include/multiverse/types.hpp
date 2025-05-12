#pragma once

#include "concord/types_basic.hpp"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

namespace mvs {
    struct Robo {
        std::string name;
        std::string uuid;
        pigment::RGB color;
        concord::Bound bound;
        concord::Size size;
        std::vector<concord::Bound> wheels;
        std::pair<std::vector<float>, std::vector<float>> controls;
        std::vector<concord::Bound> karosserie;
    };
} // namespace mvs
