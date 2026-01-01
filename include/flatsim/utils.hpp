#pragma once

#include "flatsim/types.hpp"
#include "muli/muli.h"
#include <algorithm>
#include <cmath>
#include <datapod/datapod.hpp>

namespace utils {
    inline float rad2deg(float rad) { return rad * 180.0f / M_PI; }
    inline float deg2rad(float deg) { return deg * M_PI / 180.0f; }

    // ============================================================================
    // Pose helper utilities for datapod::Pose (Quaternion-based)
    // ============================================================================

    /// Get yaw angle from datapod::Pose (extracts from Quaternion)
    inline double get_yaw(const datapod::Pose &pose) { return pose.rotation.to_euler().yaw; }

    /// Get pitch angle from datapod::Pose (extracts from Quaternion)
    inline double get_pitch(const datapod::Pose &pose) { return pose.rotation.to_euler().pitch; }

    /// Get roll angle from datapod::Pose (extracts from Quaternion)
    inline double get_roll(const datapod::Pose &pose) { return pose.rotation.to_euler().roll; }

    /// Set yaw angle on datapod::Pose (converts to Quaternion, preserves roll/pitch as 0)
    inline void set_yaw(datapod::Pose &pose, double yaw) {
        pose.rotation = datapod::Quaternion::from_euler(datapod::Euler{0.0, 0.0, yaw});
    }

    /// Create datapod::Pose from position (x, y, z) and yaw angle
    inline datapod::Pose make_pose(double x, double y, double z, double yaw) {
        datapod::Pose pose;
        pose.point = datapod::Point{x, y, z};
        pose.rotation = datapod::Quaternion::from_euler(datapod::Euler{0.0, 0.0, yaw});
        return pose;
    }

    /// Create datapod::Pose from 2D position (x, y) and yaw angle (z=0)
    inline datapod::Pose make_pose_2d(double x, double y, double yaw) { return make_pose(x, y, 0.0, yaw); }

    /// Get corners of a rectangle given pose and size (for 2D bounding boxes)
    inline std::vector<datapod::Point> get_corners(const datapod::Pose &pose, const datapod::Size &size) {
        double yaw = get_yaw(pose);
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);

        double half_x = size.x / 2.0;
        double half_y = size.y / 2.0;

        std::vector<datapod::Point> corners(4);

        // Front-right corner
        corners[0].x = pose.point.x + (half_x * cos_yaw - half_y * sin_yaw);
        corners[0].y = pose.point.y + (half_x * sin_yaw + half_y * cos_yaw);
        corners[0].z = pose.point.z;

        // Front-left corner
        corners[1].x = pose.point.x + (half_x * cos_yaw + half_y * sin_yaw);
        corners[1].y = pose.point.y + (half_x * sin_yaw - half_y * cos_yaw);
        corners[1].z = pose.point.z;

        // Back-left corner
        corners[2].x = pose.point.x + (-half_x * cos_yaw + half_y * sin_yaw);
        corners[2].y = pose.point.y + (-half_x * sin_yaw - half_y * cos_yaw);
        corners[2].z = pose.point.z;

        // Back-right corner
        corners[3].x = pose.point.x + (-half_x * cos_yaw - half_y * sin_yaw);
        corners[3].y = pose.point.y + (-half_x * sin_yaw + half_y * cos_yaw);
        corners[3].z = pose.point.z;

        return corners;
    }

    inline float mapper(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    inline u_int8_t float_to_byte(float v, float min = 0.0f, float max = 255.0f) {
        v = std::clamp(v, 0.0f, 1.0f);
        float scaled = v * 255.0f;
        float clamped = std::clamp(scaled, min, max);
        return static_cast<uint8_t>(std::round(scaled));
    }

    static double normalize_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    inline datapod::Pose shift(const datapod::Pose &parent, const datapod::Pose &child) {
        // Extract yaw angles from quaternions
        double parent_yaw = get_yaw(parent);
        double child_yaw = get_yaw(child);

        // Precompute sin/cos of the parent yaw
        double cy = std::cos(parent_yaw);
        double sy = std::sin(parent_yaw);

        // Rotate the child's local XY offset into the parent's frame
        double off_x = child.point.x * cy - child.point.y * sy;
        double off_y = child.point.x * sy + child.point.y * cy;

        datapod::Pose result;

        // Translate into the parent's world position
        result.point.x = parent.point.x + off_x;
        result.point.y = parent.point.y + off_y;
        // Carry through any vertical (z) offset without rotation
        result.point.z = parent.point.z + child.point.z;

        // Compose the yaw angles (roll and pitch remain 0 for 2D)
        double result_yaw = normalize_angle(parent_yaw + child_yaw);
        set_yaw(result, result_yaw);

        return result;
    }

    inline datapod::Pose move(datapod::Pose from_origin, datapod::Pose trans_pose) {
        double trans_yaw = get_yaw(trans_pose);

        datapod::Pose rotated_offset;
        rotated_offset.point.x = from_origin.point.x * std::cos(trans_yaw) - from_origin.point.y * std::sin(trans_yaw);
        rotated_offset.point.y = from_origin.point.x * std::sin(trans_yaw) + from_origin.point.y * std::cos(trans_yaw);

        datapod::Pose new_pose;
        new_pose.point.x = trans_pose.point.x + rotated_offset.point.x;
        new_pose.point.y = trans_pose.point.y + rotated_offset.point.y;
        set_yaw(new_pose, trans_yaw);

        return new_pose;
    }

    inline float ackermann_scale(float angleRad, float trackWidth) {
        if (std::fabs(angleRad) < 1e-6f) return 1.0f;
        float R = trackWidth / std::tan(angleRad);
        return (R - (trackWidth * 0.5f)) / R;
    }

    inline muli::Transform pose_to_transform(const datapod::Pose &pose) {
        muli::Rotation rot(get_yaw(pose));
        muli::Vec2 pos;
        pos.x = pose.point.x;
        pos.y = pose.point.y;
        return muli::Transform{pos, rot};
    }

    inline datapod::Pose transform_to_pose(const muli::Transform &transform) {
        datapod::Pose pose;
        pose.point.x = transform.position.x;
        pose.point.y = transform.position.y;
        set_yaw(pose, transform.rotation.GetAngle());
        return pose;
    }

} // namespace utils
