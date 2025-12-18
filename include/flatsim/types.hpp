#pragma once

#include <cista/serialization.h>
#include <string>
#include <vector>

#include "concord/concord.hpp"
#include "pigment/pigment.hpp"

namespace types {

    // ============================================================================
    // Main Types (using concord/pigment)
    // ============================================================================

    struct Wheel {
        std::string name;
        concord::Pose pose; // LOCAL relative to machine (for definition)
        concord::Size size;
        pigment::RGB color;
        // Physics params (Simulator uses, Agent ignores)
        float steering_max = 0.0f; // Max steering angle (radians), 0 = fixed
        float throttle_max = 1.0f; // Max throttle multiplier
        float force = 300.0f;      // Thrust force
        float friction = 0.9f;     // Lateral friction
        float max_impulse = 10.0f; // Max friction impulse
        float brake = 5.0f;        // Brake force multiplier
        float drag = 0.5f;         // Rolling resistance
    };

    struct Section {
        std::string name;
        concord::Pose pose; // LOCAL relative to karosserie
        concord::Size size;
        pigment::RGB color;
        bool working = false;
    };

    struct Karosserie {
        std::string name;
        concord::Pose pose; // LOCAL relative to machine
        concord::Size size;
        pigment::RGB color;
        bool has_physics = true;
        std::vector<Section> sections;
    };

    struct Hitch {
        std::string name;
        concord::Pose pose; // LOCAL relative to machine
        concord::Size size;
        pigment::RGB color;
        bool is_master = true; // true = can pull, false = can be pulled
        bool hooked = false;
    };

    struct Machine {
        std::string uuid;
        std::string name;
        uint32_t group = 0; // Collision group (same group won't collide)
        concord::Pose pose; // WORLD pose
        concord::Size size;
        pigment::RGB color;
        std::vector<Wheel> wheels;
        std::vector<Karosserie> karosseries;
        std::vector<Hitch> hitches;
    };

    struct MachineControl {
        std::string uuid;
        std::vector<float> steering; // Per-wheel steering angles
        std::vector<float> throttle; // Per-wheel throttle values
        float brake = 0.0f;          // Brake force (0 = no brake)
    };

    // ============================================================================
    // Serializable Types for ZMQ (cista)
    // ============================================================================

    namespace ser {

        struct Vec2 {
            float x = 0.0f;
            float y = 0.0f;

            concord::Point to_point() const { return concord::Point(x, y); }
            concord::Size to_size() const { return concord::Size(x, y, 0.0); }
            static Vec2 from_point(const concord::Point &p) {
                return {static_cast<float>(p.x), static_cast<float>(p.y)};
            }
            static Vec2 from_size(const concord::Size &s) { return {static_cast<float>(s.x), static_cast<float>(s.y)}; }
        };

        struct Pose {
            Vec2 position;
            float angle = 0.0f;

            concord::Pose to_concord() const {
                concord::Pose p;
                p.point = position.to_point();
                p.angle.yaw = angle;
                return p;
            }
            static Pose from_concord(const concord::Pose &p) {
                return {Vec2::from_point(p.point), static_cast<float>(p.angle.yaw)};
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

        struct Wheel {
            cista::raw::string name;
            Pose pose;
            Vec2 size;
            Color color;
            float steering_max = 0.0f;
            float throttle_max = 1.0f;
            float force = 300.0f;
            float friction = 0.9f;
            float max_impulse = 10.0f;
            float brake = 5.0f;
            float drag = 0.5f;

            types::Wheel to_wheel() const {
                types::Wheel w;
                w.name = std::string(name.view());
                w.pose = pose.to_concord();
                w.size = size.to_size();
                w.color = color.to_pigment();
                w.steering_max = steering_max;
                w.throttle_max = throttle_max;
                w.force = force;
                w.friction = friction;
                w.max_impulse = max_impulse;
                w.brake = brake;
                w.drag = drag;
                return w;
            }

            static Wheel from_wheel(const types::Wheel &w) {
                Wheel s;
                s.name = w.name;
                s.pose = Pose::from_concord(w.pose);
                s.size = Vec2::from_size(w.size);
                s.color = Color::from_pigment(w.color);
                s.steering_max = w.steering_max;
                s.throttle_max = w.throttle_max;
                s.force = w.force;
                s.friction = w.friction;
                s.max_impulse = w.max_impulse;
                s.brake = w.brake;
                s.drag = w.drag;
                return s;
            }
        };

        struct Section {
            cista::raw::string name;
            Pose pose;
            Vec2 size;
            Color color;
            bool working = false;

            types::Section to_section() const {
                types::Section s;
                s.name = std::string(name.view());
                s.pose = pose.to_concord();
                s.size = size.to_size();
                s.color = color.to_pigment();
                s.working = working;
                return s;
            }

            static Section from_section(const types::Section &s) {
                Section r;
                r.name = s.name;
                r.pose = Pose::from_concord(s.pose);
                r.size = Vec2::from_size(s.size);
                r.color = Color::from_pigment(s.color);
                r.working = s.working;
                return r;
            }
        };

        struct Karosserie {
            cista::raw::string name;
            Pose pose;
            Vec2 size;
            Color color;
            bool has_physics = true;
            cista::raw::vector<Section> sections;

            types::Karosserie to_karosserie() const {
                types::Karosserie k;
                k.name = std::string(name.view());
                k.pose = pose.to_concord();
                k.size = size.to_size();
                k.color = color.to_pigment();
                k.has_physics = has_physics;
                for (const auto &s : sections) {
                    k.sections.push_back(s.to_section());
                }
                return k;
            }

            static Karosserie from_karosserie(const types::Karosserie &k) {
                Karosserie r;
                r.name = k.name;
                r.pose = Pose::from_concord(k.pose);
                r.size = Vec2::from_size(k.size);
                r.color = Color::from_pigment(k.color);
                r.has_physics = k.has_physics;
                for (const auto &s : k.sections) {
                    r.sections.push_back(Section::from_section(s));
                }
                return r;
            }
        };

        struct Hitch {
            cista::raw::string name;
            Pose pose;
            Vec2 size;
            Color color;
            bool is_master = true;
            bool hooked = false;

            types::Hitch to_hitch() const {
                types::Hitch h;
                h.name = std::string(name.view());
                h.pose = pose.to_concord();
                h.size = size.to_size();
                h.color = color.to_pigment();
                h.is_master = is_master;
                h.hooked = hooked;
                return h;
            }

            static Hitch from_hitch(const types::Hitch &h) {
                Hitch r;
                r.name = h.name;
                r.pose = Pose::from_concord(h.pose);
                r.size = Vec2::from_size(h.size);
                r.color = Color::from_pigment(h.color);
                r.is_master = h.is_master;
                r.hooked = h.hooked;
                return r;
            }
        };

        struct Machine {
            cista::raw::string uuid;
            cista::raw::string name;
            uint32_t group = 0;
            Pose pose;
            Vec2 size;
            Color color;
            cista::raw::vector<Wheel> wheels;
            cista::raw::vector<Karosserie> karosseries;
            cista::raw::vector<Hitch> hitches;

            types::Machine to_machine() const {
                types::Machine m;
                m.uuid = std::string(uuid.view());
                m.name = std::string(name.view());
                m.group = group;
                m.pose = pose.to_concord();
                m.size = size.to_size();
                m.color = color.to_pigment();
                for (const auto &w : wheels) {
                    m.wheels.push_back(w.to_wheel());
                }
                for (const auto &k : karosseries) {
                    m.karosseries.push_back(k.to_karosserie());
                }
                for (const auto &h : hitches) {
                    m.hitches.push_back(h.to_hitch());
                }
                return m;
            }

            static Machine from_machine(const types::Machine &m) {
                Machine r;
                r.uuid = m.uuid;
                r.name = m.name;
                r.group = m.group;
                r.pose = Pose::from_concord(m.pose);
                r.size = Vec2::from_size(m.size);
                r.color = Color::from_pigment(m.color);
                for (const auto &w : m.wheels) {
                    r.wheels.push_back(Wheel::from_wheel(w));
                }
                for (const auto &k : m.karosseries) {
                    r.karosseries.push_back(Karosserie::from_karosserie(k));
                }
                for (const auto &h : m.hitches) {
                    r.hitches.push_back(Hitch::from_hitch(h));
                }
                return r;
            }
        };

        struct MachineControl {
            cista::raw::string uuid;
            cista::raw::vector<float> steering;
            cista::raw::vector<float> throttle;
            float brake = 0.0f;

            types::MachineControl to_control() const {
                types::MachineControl c;
                c.uuid = std::string(uuid.view());
                for (const auto &s : steering) c.steering.push_back(s);
                for (const auto &t : throttle) c.throttle.push_back(t);
                c.brake = brake;
                return c;
            }

            static MachineControl from_control(const types::MachineControl &c) {
                MachineControl r;
                r.uuid = c.uuid;
                for (const auto &s : c.steering) r.steering.push_back(s);
                for (const auto &t : c.throttle) r.throttle.push_back(t);
                r.brake = c.brake;
                return r;
            }
        };

        // State feedback from simulator
        struct WheelState {
            Pose pose;     // World pose
            Vec2 velocity; // Linear velocity
            float angular_vel = 0.0f;
        };

        struct MachineState {
            cista::raw::string uuid;
            Pose pose;
            Vec2 velocity;
            float angular_vel = 0.0f;
            cista::raw::vector<WheelState> wheels;
        };

        struct WorldState {
            cista::raw::vector<MachineState> machines;
        };

        // Message types for ZMQ protocol
        enum class MsgType : uint8_t {
            SPAWN,
            DESPAWN,
            CONTROL,
            GET_STATE,
        };

        struct Request {
            MsgType type;
            Machine machine;         // For SPAWN
            MachineControl control;  // For CONTROL
            cista::raw::string uuid; // For DESPAWN
        };

        struct Response {
            bool success = false;
            WorldState state;
        };

    } // namespace ser

} // namespace types
