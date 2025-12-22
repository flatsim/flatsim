#pragma once

#include "flatsim/types.hpp"
#include "muli/collision_filter.h"
#include "muli/world.h"
#include <memory>

namespace simulator {

    // ============================================================================
    // SensorData - Fills sensor structs from physics world
    // ============================================================================

    class Data {
      public:
        Data() = default;

        void set_world(std::shared_ptr<muli::World> world) { world_ = world; }
        void set_datum(const concord::Datum &datum) { datum_ = datum; }

        // LIDAR - Raycast in physics world, returns ranges/angles
        types::LidarData scan_lidar(const concord::Pose &pose, float min_range, float max_range, float fov_deg,
                                    float resolution_deg, const muli::CollisionFilter &filter);

        // GPS - Convert ENU pose to WGS84
        types::GpsData pose_to_gps(const concord::Pose &pose, float speed);

        // IMU - Compute from velocities and pose
        types::ImuData compute_imu(const concord::Pose &pose, float linear_vel, float angular_vel,
                                   float prev_linear_vel, float dt);

      private:
        std::shared_ptr<muli::World> world_;
        concord::Datum datum_;
    };

} // namespace simulator
