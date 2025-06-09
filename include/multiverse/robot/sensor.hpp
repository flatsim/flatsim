#pragma once

#include <any>
#include <memory>
#include <string>
#include <vector>

#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"

namespace mvs {
    class Sensor {
      private:
        std::string name;
        std::vector<std::any> args;
        concord::Pose position;

      public:
        virtual ~Sensor() = default;
        virtual void tick(float dt, concord::Pose &position) = 0;
        virtual void init(std::string name, concord::Pose position, const std::vector<std::any> &args) = 0;
    };
    
    // Factory function to create different sensor types
    std::unique_ptr<Sensor> createSensor(const std::string& sensor_type = "basic");
    
} // namespace mvs
