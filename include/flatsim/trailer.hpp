#pragma once

#include "muli/rigidbody.h"
#include "muli/world.h"
#include "flatsim/types.hpp"
#include "flatsim/robot/chassis/chassis.hpp"
#include "flatsim/robot/tank.hpp"
#include <memory>

namespace fs {

class Trailer {
private:
    std::shared_ptr<rerun::RecordingStream> rec;
    std::shared_ptr<muli::World> world;
    std::unique_ptr<Chassis> chassis;
    std::optional<Tank> tank;
    muli::CollisionFilter filter;
    concord::Pose spawn_position;
    
public:
    RobotInfo info;  // Reuse RobotInfo for consistency
    
    Trailer(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group);
    ~Trailer();
    
    void tick(float dt);
    void init(concord::Datum datum, RobotInfo trailer_info);
    void teleport(concord::Pose pose);
    
    // Get chassis body for hitching
    muli::RigidBody* get_body() const { 
        return chassis ? chassis->get_body() : nullptr; 
    }
    
    const concord::Pose& get_position() const { return info.bound.pose; }
    
    // Tank management (same as Robot)
    bool has_tank() const { return tank.has_value(); }
    Tank* get_tank() { return tank.has_value() ? &tank.value() : nullptr; }
    const Tank* get_tank() const { return tank.has_value() ? &tank.value() : nullptr; }
    void empty_tank() { 
        if (tank.has_value()) tank->empty_all(); 
    }
    void fill_tank(float amount) { 
        if (tank.has_value()) tank->fill(amount); 
    }
};

} // namespace fs