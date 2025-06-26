#pragma once

#include <any>
#include <string>
#include <vector>

#include "muli/types.h"
#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"

namespace fs {

    class Obstacle {
      private:
        std::string name;
        std::vector<std::any> args;
        concord::Pose position;

      public:
        virtual void tick(float dt, concord::Pose &position) = 0;
        virtual void init(std::string name, concord::Pose position, const std::vector<std::any> &args) = 0;
    };
} // namespace fs
