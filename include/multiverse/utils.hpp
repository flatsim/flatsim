#pragma once

#include "concord/types_basic.hpp"
#include "muli/muli.h"
#include <algorithm>
#include <cmath>

namespace mvs {
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
            double off_x = child.point.enu.x * cy - child.point.enu.y * sy;
            double off_y = child.point.enu.x * sy + child.point.enu.y * cy;

            concord::Pose result;

            // Translate into the parent's world position
            result.point.enu.x = parent.point.enu.x + off_x;
            result.point.enu.y = parent.point.enu.y + off_y;
            // Carry through any vertical (z) offset without rotation
            result.point.enu.z = parent.point.enu.z + child.point.enu.z;

            // Compose the Euler angles: parent ⊕ child
            result.angle.roll = normalize_angle(parent.angle.roll + child.angle.roll);
            result.angle.pitch = normalize_angle(parent.angle.pitch + child.angle.pitch);
            result.angle.yaw = normalize_angle(parent.angle.yaw + child.angle.yaw);

            return result;
        }

        inline muli::Transform pose_to_transform(const concord::Pose &pose) {
            muli::Rotation rot(pose.angle.yaw);
            muli::Vec2 pos;
            pos.x = pose.point.enu.x;
            pos.y = pose.point.enu.y;
            return muli::Transform{pos, rot};
        }

    } // namespace utils
} // namespace mvs
