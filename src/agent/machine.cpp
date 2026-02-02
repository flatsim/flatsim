#include "flatsim/agent/machine.hpp"
#include "flatsim/utils.hpp"
#include <cmath>

namespace agent {

    Machine::Machine(std::shared_ptr<rerun::RecordingStream> rec, const types::Machine &config)
        : rec_(rec), config_(config) {
        world_pose_ = config_.bound.pose;
    }

    void Machine::init() {
        // Initialize control manager
        controls.init(&config_, rec_);
    }

    void Machine::update_state(const types::ser::MachineState &state) {
        world_pose_ = state.pose.to_datapod();
        // Compute forward velocity along heading from 2D velocity vector
        float yaw = utils::get_yaw(world_pose_);
        linear_velocity_ = state.velocity.x * std::cos(yaw) + state.velocity.y * std::sin(yaw);
        angular_velocity_ = state.angular_vel;
    }

    void Machine::tick(float dt) {
        // Update control manager (navigation, path following)
        if (navigation_enabled_) {
            controls.tick(world_pose_, linear_velocity_, angular_velocity_, dt);
        }
    }

    void Machine::tick(float dt, const types::SensorData & /* sensor_data */) {
        // Sensor data available via agent47 protocol if needed by controls
        if (navigation_enabled_) {
            controls.tick(world_pose_, linear_velocity_, angular_velocity_, dt);
        }
    }

    void Machine::tock() {
        // Visualize tracker/path
        controls.tock(rec_);
    }

} // namespace agent
