#include "flatsim/trailer.hpp"
#include <spdlog/spdlog.h>

namespace fs {

Trailer::Trailer(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group)
    : rec(rec), world(world) {
    filter.bit = 1 << group;
    filter.mask = ~(1 << group);
}

Trailer::~Trailer() {}

void Trailer::tick(float dt) {
    if (!chassis) {
        throw NullPointerException("chassis");
    }
    
    // Update position from physics
    this->info.bound.pose.point.x = chassis->get_transform().position.x;
    this->info.bound.pose.point.y = chassis->get_transform().position.y;
    this->info.bound.pose.angle.yaw = chassis->get_transform().rotation.GetAngle();
    
    // Update chassis components
    chassis->tick(dt);
    
    // Update tank if present
    if (tank.has_value()) {
        tank->tick(dt, chassis->get_pose(), rec);
    }
    
    // Visualize
    chassis->visualize(info.name);
}

void Trailer::init(concord::Datum datum, RobotInfo trailer_info) {
    spdlog::info("Initializing trailer {}...", trailer_info.name);
    this->info = trailer_info;
    this->spawn_position = trailer_info.bound.pose;
    
    if (!world) {
        throw NullPointerException("world");
    }
    
    if (!rec) {
        throw NullPointerException("recording stream");
    }
    
    // Create chassis (simpler than robot - no motors)
    chassis = std::make_unique<Chassis>(world, rec, filter);
    if (!chassis) {
        throw InitializationException("chassis creation failed");
    }
    chassis->init(trailer_info);
    
    // Initialize tank if present
    if (trailer_info.tank.has_value()) {
        tank = Tank(trailer_info.tank->name, Tank::Type::HARVEST, 
                   trailer_info.tank->capacity, 0.0f, 0.0f);
        tank->init(info.color, info.name, trailer_info.tank->bound);
    }
    
    // Note: No power source for trailers - they're pulled
}

void Trailer::teleport(concord::Pose pose) {
    chassis->teleport(pose);
}

} // namespace fs