#pragma once

#include "muli/muli.h"
#include "flatsim/types.hpp"
#include <algorithm>
#include <cmath>

namespace fs {
    namespace utils {
        inline float rad2deg(float rad) { return rad * 180.0f / M_PI; }
        inline float deg2rad(float deg) { return deg * M_PI / 180.0f; }

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

        inline concord::Pose shift(const concord::Pose &parent, const concord::Pose &child) {
            // Precompute sin/cos of the parent yaw
            double cy = std::cos(parent.angle.yaw);
            double sy = std::sin(parent.angle.yaw);

            // Rotate the child's local XY offset into the parent's frame
            double off_x = child.point.x * cy - child.point.y * sy;
            double off_y = child.point.x * sy + child.point.y * cy;

            concord::Pose result;

            // Translate into the parent's world position
            result.point.x = parent.point.x + off_x;
            result.point.y = parent.point.y + off_y;
            // Carry through any vertical (z) offset without rotation
            result.point.z = parent.point.z + child.point.z;

            // Compose the Euler angles: parent ⊕ child
            result.angle.roll = normalize_angle(parent.angle.roll + child.angle.roll);
            result.angle.pitch = normalize_angle(parent.angle.pitch + child.angle.pitch);
            result.angle.yaw = normalize_angle(parent.angle.yaw + child.angle.yaw);

            return result;
        }

        inline concord::Pose move(concord::Pose from_origin, concord::Pose trans_pose) {
            concord::Pose t;
            t.point.x = trans_pose.point.x;
            t.point.y = trans_pose.point.y;
            t.angle.yaw = trans_pose.angle.yaw;

            concord::Pose rotated_offset;
            rotated_offset.point.x =
                from_origin.point.x * std::cos(t.angle.yaw) - from_origin.point.y * std::sin(t.angle.yaw);
            rotated_offset.point.y =
                from_origin.point.x * std::sin(t.angle.yaw) + from_origin.point.y * std::cos(t.angle.yaw);

            concord::Pose new_pose;
            new_pose.point.x = trans_pose.point.x + rotated_offset.point.x;
            new_pose.point.y = trans_pose.point.y + rotated_offset.point.y;
            new_pose.angle.yaw = t.angle.yaw;

            return new_pose;
        }

        inline float ackermann_scale(float angleRad, float trackWidth) {
            if (std::fabs(angleRad) < 1e-6f)
                return 1.0f;
            float R = trackWidth / std::tan(angleRad);
            return (R - (trackWidth * 0.5f)) / R;
        }

        inline muli::Transform pose_to_transform(const concord::Pose &pose) {
            muli::Rotation rot(pose.angle.yaw);
            muli::Vec2 pos;
            pos.x = pose.point.x;
            pos.y = pose.point.y;
            return muli::Transform{pos, rot};
        }

        inline concord::Pose transform_to_pose(const muli::Transform &transform) {
            concord::Pose pose;
            pose.point.x = transform.position.x;
            pose.point.y = transform.position.y;
            pose.angle.yaw = transform.rotation.GetAngle();
            return pose;
        }

    } // namespace utils
} // namespace fs
