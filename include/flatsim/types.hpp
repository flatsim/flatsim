#pragma once

#include <cista/serialization.h>
#include <string>

#include "concord/concord.hpp"
#include "pigment/pigment.hpp"

namespace types {

    // Main Chassis struct using concord/pigment types
    struct Chassis {
        std::string uuid;
        std::string name;
        concord::Pose pose;
        concord::Size size;
        pigment::RGB color;
    };

    // Serializable types for ZMQ transfer
    namespace ser {

        struct Vec2 {
            float x = 0.0f;
            float y = 0.0f;

            concord::Point to_concord() const { return concord::Point(x, y); }
            static Vec2 from_concord(const concord::Point &p) {
                return {static_cast<float>(p.x), static_cast<float>(p.y)};
            }
        };

        struct Pose {
            Vec2 position;
            float angle = 0.0f;

            concord::Pose to_concord() const {
                concord::Pose p;
                p.point = position.to_concord();
                p.angle.yaw = angle;
                return p;
            }
            static Pose from_concord(const concord::Pose &p) {
                return {Vec2::from_concord(p.point), static_cast<float>(p.angle.yaw)};
            }
        };

        struct Size {
            float width = 1.0f;
            float height = 2.0f;

            concord::Size to_concord() const { return concord::Size(width, height, 0.0); }
            static Size from_concord(const concord::Size &s) {
                return {static_cast<float>(s.x), static_cast<float>(s.y)};
            }
        };

        struct Color {
            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;

            pigment::RGB to_pigment() const { return pigment::RGB(r, g, b); }
            static Color from_pigment(const pigment::RGB &c) {
                return {static_cast<uint8_t>(c.r), static_cast<uint8_t>(c.g), static_cast<uint8_t>(c.b)};
            }
        };

        // Serializable Chassis for ZMQ
        struct Chassis {
            cista::raw::string uuid;
            cista::raw::string name;
            Pose pose;
            Size size;
            Color color;

            // Convert to main Chassis type
            types::Chassis to_chassis() const {
                return {std::string(uuid.view()), std::string(name.view()), pose.to_concord(), size.to_concord(),
                        color.to_pigment()};
            }

            // Convert from main Chassis type
            static Chassis from_chassis(const types::Chassis &c) {
                Chassis s;
                s.uuid = c.uuid;
                s.name = c.name;
                s.pose = Pose::from_concord(c.pose);
                s.size = Size::from_concord(c.size);
                s.color = Color::from_pigment(c.color);
                return s;
            }
        };

    } // namespace ser

} // namespace types
