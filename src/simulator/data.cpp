#include "flatsim/simulator/data.hpp"
#include "flatsim/gps.hpp"
#include "flatsim/utils.hpp"
#include <cmath>

namespace simulator {

    types::LidarData Data::scan_lidar(const datapod::Pose &pose, float min_range, float max_range, float fov_deg,
                                      float resolution_deg, const flywheel::CollisionFilter &filter) {
        types::LidarData data;
        data.min_range = min_range;
        data.max_range = max_range;

        if (!world_) {
            return data;
        }

        // Convert to radians
        float fov_rad = fov_deg * M_PI / 180.0f;
        float resolution_rad = resolution_deg * M_PI / 180.0f;
        int num_beams = static_cast<int>(std::ceil(fov_rad / resolution_rad));

        data.ranges.reserve(num_beams);
        data.angles.reserve(num_beams);
        data.valid.reserve(num_beams);

        // Sensor position
        flywheel::Vec2 sensor_pos(pose.point.x, pose.point.y);
        float sensor_yaw = utils::get_yaw(pose);

        // Scan from -fov/2 to +fov/2
        float start_angle = -fov_rad / 2.0f;

        for (int i = 0; i < num_beams; ++i) {
            float beam_angle = start_angle + i * resolution_rad;
            float world_angle = sensor_yaw + beam_angle;

            // Ray direction
            flywheel::Vec2 direction(std::cos(world_angle), std::sin(world_angle));
            flywheel::Vec2 ray_end = sensor_pos + direction * max_range;

            // Raycast
            float closest_distance = max_range;
            bool hit = false;

            world_->RayCastAny(sensor_pos, ray_end, 0.0f,
                               [&](flywheel::Collider *collider, flywheel::Vec2 point, flywheel::Vec2 normal,
                                   float fraction) -> float {
                                   // Check collision filter - skip if same group (own robot)
                                   const auto &collider_filter = collider->GetFilter();
                                   if ((collider_filter.bit & filter.bit) != 0) {
                                       return 1.0f; // Continue raycasting
                                   }

                                   // Valid hit
                                   float distance = fraction * max_range;
                                   if (distance >= min_range && distance < closest_distance) {
                                       closest_distance = distance;
                                       hit = true;
                                   }
                                   return fraction;
                               });

            data.ranges.push_back(closest_distance);
            data.angles.push_back(beam_angle);
            data.valid.push_back(hit && closest_distance < max_range * 0.99f);
        }

        return data;
    }

    types::GpsData Data::pose_to_gps(const datapod::Pose &pose, float speed) {
        types::GpsData data;

        // Convert ENU to WGS84 using flatsim::gps
        datapod::Point enu_point = pose.point;
        datapod::Geo gps_coords = flatsim::gps::enu_to_gps(enu_point, datum_);

        data.latitude = gps_coords.latitude;
        data.longitude = gps_coords.longitude;
        data.altitude = gps_coords.altitude;
        data.heading = utils::get_yaw(pose);
        data.speed = speed;

        return data;
    }

    types::ImuData Data::compute_imu(const datapod::Pose &pose, float linear_vel, float angular_vel,
                                     float prev_linear_vel, float dt) {
        types::ImuData data;

        // Compute linear acceleration from velocity change
        float accel = (dt > 0.0f) ? (linear_vel - prev_linear_vel) / dt : 0.0f;

        // Acceleration in body frame (forward = x)
        data.accel_x = accel;
        data.accel_y = 0.0f;
        data.accel_z = 9.81f; // Gravity

        // Angular velocity (yaw rate)
        data.gyro_z = angular_vel;

        // Current yaw
        data.yaw = utils::get_yaw(pose);

        return data;
    }

} // namespace simulator
