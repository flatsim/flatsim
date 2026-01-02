#pragma once

#include "flatsim/types.hpp"
#include "flywheel/collision_filter.h"
#include "flywheel/world.h"
#include <memory>

namespace simulator {

    // ============================================================================
    // SensorData - Fills sensor structs from physics world
    // ============================================================================

    class Data {
      public:
        Data() = default;

        void set_world(std::shared_ptr<flywheel::World> world) { world_ = world; }
        void set_datum(const datapod::Geo &datum) { datum_ = datum; }

        // LIDAR - Raycast in physics world, returns ranges/angles
        types::LidarData scan_lidar(const datapod::Pose &pose, float min_range, float max_range, float fov_deg,
                                    float resolution_deg, const flywheel::CollisionFilter &filter);

        // GPS - Convert ENU pose to WGS84
        types::GpsData pose_to_gps(const datapod::Pose &pose, float speed);

        // IMU - Compute from velocities and pose
        types::ImuData compute_imu(const datapod::Pose &pose, float linear_vel, float angular_vel,
                                   float prev_linear_vel, float dt);

      private:
        std::shared_ptr<flywheel::World> world_;
        datapod::Geo datum_;
    };

} // namespace simulator
