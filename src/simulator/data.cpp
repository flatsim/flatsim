#include "flatsim/simulator/data.hpp"
#include <cmath>

namespace simulator {

    types::LidarData Data::scan_lidar(const concord::Pose &pose, float min_range, float max_range, float fov_deg,
                                      float resolution_deg, const muli::CollisionFilter &filter) {
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
        muli::Vec2 sensor_pos(pose.point.x, pose.point.y);
        float sensor_yaw = pose.angle.yaw;

        // Scan from -fov/2 to +fov/2
        float start_angle = -fov_rad / 2.0f;

        for (int i = 0; i < num_beams; ++i) {
            float beam_angle = start_angle + i * resolution_rad;
            float world_angle = sensor_yaw + beam_angle;

            // Ray direction
            muli::Vec2 direction(std::cos(world_angle), std::sin(world_angle));
            muli::Vec2 ray_end = sensor_pos + direction * max_range;

            // Raycast
            float closest_distance = max_range;
            bool hit = false;

            world_->RayCastAny(
                sensor_pos, ray_end, 0.0f,
                [&](muli::Collider *collider, muli::Vec2 point, muli::Vec2 normal, float fraction) -> float {
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

    types::GpsData Data::pose_to_gps(const concord::Pose &pose, float speed) {
        types::GpsData data;

        // Convert ENU to WGS84 using datum
        // Simple approximation: 1 degree latitude ~ 111km, 1 degree longitude ~ 111km * cos(lat)
        double lat_rad = datum_.lat * M_PI / 180.0;
        double meters_per_deg_lat = 111320.0;
        double meters_per_deg_lon = 111320.0 * std::cos(lat_rad);

        data.latitude = datum_.lat + (pose.point.y / meters_per_deg_lat);
        data.longitude = datum_.lon + (pose.point.x / meters_per_deg_lon);
        data.altitude = datum_.alt + pose.point.z;
        data.heading = pose.angle.yaw;
        data.speed = speed;

        return data;
    }

    types::ImuData Data::compute_imu(const concord::Pose &pose, float linear_vel, float angular_vel,
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
        data.yaw = pose.angle.yaw;

        return data;
    }

} // namespace simulator
