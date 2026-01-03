#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "pigment/pigment.hpp"
#include <datapod/datapod.hpp>

namespace types {

    // ============================================================================
    // Main Types (using datapod/pigment)
    // ============================================================================

    enum class PowerType { FUEL, BATTERY };
    enum class MachineRole { MASTER, FOLLOWER, SLAVE };

    struct Physics {
        float linear_damping = 0.2f;
        float angular_damping = 0.2f;
        float force = 30.0f; // Base force for typical wheel (0.2m radius)
        float torque = 10.0f;
        float friction = 1.5f;    // Base lateral friction (increased for better grip)
        float max_impulse = 2.0f; // Base max impulse for lateral friction
        float brake = 10.0f;
        float drag = 0.5f; // Velocity-dependent drag coefficient
    };

    struct MachineControls {
        std::vector<float> steerings_max;
        std::vector<float> throttles_max;
        std::vector<float> steerings_diff;
        std::vector<float> throttles_diff;
        std::vector<bool> left_side;
    };

    enum class ContainerType { HARVEST, WASTE };

    struct Container {
        std::string name;
        ContainerType type = ContainerType::HARVEST;
        float capacity;
        datapod::Box bound;
    };

    // Backwards-compatible name (legacy JSON key is still "tank")
    using Tank = Container;

    struct Power {
        std::string name;
        PowerType type;
        float capacity;
        float consumption_rate;
        float charge_rate = 0.0f;
    };

    struct Capability {
        std::vector<std::string> work_on;
        std::vector<std::string> connect_to;
        std::vector<std::string> unload_to;
    };

    struct Wheel {
        std::string name;
        datapod::Box bound;
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
        datapod::Box bound; // LOCAL relative to karosserie
        pigment::RGB color;
        bool working = false;
    };

    struct Karosserie {
        std::string name;
        datapod::Box bound; // LOCAL relative to machine
        pigment::RGB color;
        bool has_physics = true;
        std::vector<Section> sections;
    };

    struct Hitch {
        std::string name;
        datapod::Box bound; // LOCAL relative to machine
        pigment::RGB color;
        bool is_master = true; // true = can pull, false = can be pulled
        bool hooked = false;
    };

    // LIDAR sensor configuration (stored in Machine, used by simulator for raycasting)
    struct LidarConfig {
        bool enabled = false;        // If true, simulator will compute LIDAR data
        float min_range = 0.5f;      // Minimum detection range (meters)
        float max_range = 15.0f;     // Maximum detection range (meters)
        float fov_deg = 45.0f;       // Horizontal field of view (degrees)
        float resolution_deg = 3.0f; // Angular resolution (degrees)
    };

    enum class Role { MASTER, FOLLOWER, SLAVE };

    struct Machine {
        uint rci;
        uint group;
        bool slave = false;
        std::string name = "unnamed";
        std::string uuid = "none";
        std::string type = "none";
        std::vector<std::string> works_on;
        Capability capability;
        pigment::RGB color;
        datapod::Box bound;
        datapod::Polygon outline;
        std::vector<Wheel> wheels;
        MachineControls controls;
        std::unordered_map<std::string, Hitch> hitches;
        std::vector<Karosserie> karosseries;
        std::optional<Tank> tank;
        std::optional<Power> power_source;
        std::optional<LidarConfig> lidar; // LIDAR sensor configuration
        MachineRole role = MachineRole::MASTER;
        float turning_radius = 1.0f;
        std::string seqid = name;
    };

    // Velocity control command (agent -> simulator) - bicycle model
    struct BicycleControl {
        std::string uuid;
        float linear = 0.0f;  // Desired linear velocity [-1, 1] normalized
        float angular = 0.0f; // Desired angular velocity [-1, 1] normalized
        float brake = 0.0f;   // Brake force (0 = no brake)
    };

    // Per-wheel control command (low-level control)
    struct WheelControl {
        std::string uuid;
        std::vector<float> steering; // Per-wheel steering angles
        std::vector<float> throttle; // Per-wheel throttle values
        float brake = 0.0f;          // Brake force (0 = no brake)
    };

    enum class OP { IDLE, CHARGING, STOP, PAUSE, EMERGENCY, TRANSPORT, WORK };

    struct State {
        bool online = true;
        Role role = Role::MASTER;
        OP mode = OP::IDLE;
        bool turn_first = false;  // For diff/skid: rotate in place before translating
        bool allow_move = true;   // Allow movement (false = send zero velocity for collision avoidance)
        float speed_scale = 1.0f; // Scale factor for velocity commands (0.0 - 1.0)
    };

    // ============================================================================
    // Sensor Data Types - Raw data from simulator physics
    // ============================================================================

    // LIDAR scan data from physics raycasting
    struct LidarData {
        std::vector<float> ranges; // Distance measurements (meters)
        std::vector<float> angles; // Beam angles (radians, relative to heading)
        std::vector<bool> valid;   // True if beam hit something
        float min_range = 0.1f;
        float max_range = 30.0f;
    };

    // GPS data from ENU to WGS84 conversion
    struct GpsData {
        double latitude = 0.0;  // WGS84 degrees
        double longitude = 0.0; // WGS84 degrees
        double altitude = 0.0;  // Meters
        float heading = 0.0f;   // Radians
        float speed = 0.0f;     // m/s
    };

    // IMU data from physics velocities
    struct ImuData {
        float accel_x = 0.0f;  // m/s^2
        float accel_y = 0.0f;  // m/s^2
        float accel_z = 9.81f; // m/s^2 (gravity)
        float gyro_z = 0.0f;   // rad/s (yaw rate)
        float yaw = 0.0f;      // radians
    };

    // Combined sensor data for a machine
    struct SensorData {
        LidarData lidar;
        GpsData gps;
        ImuData imu;
        bool has_lidar = false;
        bool has_gps = false;
        bool has_imu = false;
    };

    // ============================================================================
    // World Types
    // ============================================================================

    struct WorldSettings {
        datapod::Size size = datapod::Size(100.0, 100.0, 0.0); // World bounds
        datapod::Geo datum;                                    // GPS reference point
    };

    struct StaticObstacle {
        size_t id = 0;
        datapod::Point position;
        double radius = 0.5;      // Obstacle radius (m)
        double uncertainty = 0.1; // Position uncertainty std dev (m)
    };

    struct DynamicObstacle {
        size_t id = 0;
        datapod::Point position;
        datapod::Point velocity;           // Velocity (m/s)
        double radius = 0.5;               // Obstacle radius (m)
        double uncertainty = 0.3;          // Position uncertainty std dev (m)
        double activation_distance = 10.0; // Distance to activate movement
        bool is_active = false;
    };

    // ============================================================================
    // Serializable Types for ZMQ (cista)
    // ============================================================================

    namespace ser {

        struct Vec2 {
            float x = 0.0f;
            float y = 0.0f;

            datapod::Point to_point() const { return datapod::Point(x, y); }
            datapod::Size to_size() const { return datapod::Size(x, y, 0.0); }
            static Vec2 from_point(const datapod::Point &p) {
                return {static_cast<float>(p.x), static_cast<float>(p.y)};
            }
            static Vec2 from_size(const datapod::Size &s) { return {static_cast<float>(s.x), static_cast<float>(s.y)}; }
        };

        struct Pose {
            Vec2 position;
            float angle = 0.0f;

            datapod::Pose to_datapod() const {
                datapod::Pose p;
                p.point = position.to_point();
                p.rotation = datapod::Quaternion::from_euler(datapod::Euler{0.0, 0.0, static_cast<double>(angle)});
                return p;
            }
            static Pose from_datapod(const datapod::Pose &p) {
                return {Vec2::from_point(p.point), static_cast<float>(p.rotation.to_euler().yaw)};
            }
        };

        struct Color {
            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;

            pigment::RGB to_pigment() const { return pigment::RGB(r, g, b); }
            static Color from_pigment(const pigment::RGB &c) { return {c.r(), c.g(), c.b()}; }
        };

        struct Bound {
            Pose pose;
            Vec2 size;

            datapod::Box to_box() const { return datapod::Box{pose.to_datapod(), size.to_size()}; }

            static Bound from_box(const datapod::Box &b) {
                return {Pose::from_datapod(b.pose), Vec2::from_size(b.size)};
            }
        };

        struct Wheel {
            datapod::String name;
            Bound bound;
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
                w.bound = bound.to_box();
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
                s.name = datapod::String(w.name);
                s.bound = Bound::from_box(w.bound);
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

            // Custom members() function for datapod serialization (struct has 11 fields)
            auto members() {
                return std::tie(name, bound, color, steering_max, throttle_max, force, friction, max_impulse, brake,
                                drag);
            }

            auto members() const {
                return std::tie(name, bound, color, steering_max, throttle_max, force, friction, max_impulse, brake,
                                drag);
            }
        };

        struct Section {
            datapod::String name;
            Bound bound;
            Color color;
            bool working = false;

            types::Section to_section() const {
                types::Section s;
                s.name = std::string(name.view());
                s.bound = bound.to_box();
                s.color = color.to_pigment();
                s.working = working;
                return s;
            }

            static Section from_section(const types::Section &s) {
                Section r;
                r.name = datapod::String(s.name);
                r.bound = Bound::from_box(s.bound);
                r.color = Color::from_pigment(s.color);
                r.working = s.working;
                return r;
            }
        };

        struct Karosserie {
            datapod::String name;
            Bound bound;
            Color color;
            bool has_physics = true;
            datapod::Vector<Section> sections;

            types::Karosserie to_karosserie() const {
                types::Karosserie k;
                k.name = std::string(name.view());
                k.bound = bound.to_box();
                k.color = color.to_pigment();
                k.has_physics = has_physics;
                for (const auto &s : sections) {
                    k.sections.push_back(s.to_section());
                }
                return k;
            }

            static Karosserie from_karosserie(const types::Karosserie &k) {
                Karosserie r;
                r.name = datapod::String(k.name);
                r.bound = Bound::from_box(k.bound);
                r.color = Color::from_pigment(k.color);
                r.has_physics = k.has_physics;
                for (const auto &s : k.sections) {
                    r.sections.push_back(Section::from_section(s));
                }
                return r;
            }
        };

        struct Hitch {
            datapod::String name;
            Bound bound;
            Color color;
            bool is_master = true;
            bool hooked = false;

            types::Hitch to_hitch() const {
                types::Hitch h;
                h.name = std::string(name.view());
                h.bound = bound.to_box();
                h.color = color.to_pigment();
                h.is_master = is_master;
                h.hooked = hooked;
                return h;
            }

            static Hitch from_hitch(const types::Hitch &h) {
                Hitch r;
                r.name = datapod::String(h.name);
                r.bound = Bound::from_box(h.bound);
                r.color = Color::from_pigment(h.color);
                r.is_master = h.is_master;
                r.hooked = h.hooked;
                return r;
            }
        };

        struct Capability {
            datapod::Vector<datapod::String> work_on;
            datapod::Vector<datapod::String> connect_to;
            datapod::Vector<datapod::String> unload_to;

            types::Capability to_capability() const {
                types::Capability c;
                for (const auto &s : work_on) c.work_on.push_back(std::string(s.view()));
                for (const auto &s : connect_to) c.connect_to.push_back(std::string(s.view()));
                for (const auto &s : unload_to) c.unload_to.push_back(std::string(s.view()));
                return c;
            }

            static Capability from_capability(const types::Capability &c) {
                Capability r;
                for (const auto &s : c.work_on) r.work_on.push_back(datapod::String(s));
                for (const auto &s : c.connect_to) r.connect_to.push_back(datapod::String(s));
                for (const auto &s : c.unload_to) r.unload_to.push_back(datapod::String(s));
                return r;
            }
        };

        struct MachineControls {
            datapod::Vector<float> steerings_max;
            datapod::Vector<float> throttles_max;
            datapod::Vector<float> steerings_diff;
            datapod::Vector<float> throttles_diff;
            datapod::Vector<bool> left_side;

            types::MachineControls to_controls() const {
                types::MachineControls c;
                for (const auto &v : steerings_max) c.steerings_max.push_back(v);
                for (const auto &v : throttles_max) c.throttles_max.push_back(v);
                for (const auto &v : steerings_diff) c.steerings_diff.push_back(v);
                for (const auto &v : throttles_diff) c.throttles_diff.push_back(v);
                for (const auto &v : left_side) c.left_side.push_back(v);
                return c;
            }

            static MachineControls from_controls(const types::MachineControls &c) {
                MachineControls r;
                for (const auto &v : c.steerings_max) r.steerings_max.push_back(v);
                for (const auto &v : c.throttles_max) r.throttles_max.push_back(v);
                for (const auto &v : c.steerings_diff) r.steerings_diff.push_back(v);
                for (const auto &v : c.throttles_diff) r.throttles_diff.push_back(v);
                for (const auto &v : c.left_side) r.left_side.push_back(v);
                return r;
            }
        };

        struct Polygon {
            datapod::Vector<Vec2> points;

            datapod::Polygon to_polygon() const {
                datapod::Vector<datapod::Point> pts;
                for (const auto &pt : points) pts.push_back(pt.to_point());
                return datapod::Polygon{pts};
            }

            static Polygon from_polygon(const datapod::Polygon &p) {
                Polygon r;
                for (const auto &pt : p.vertices) r.points.push_back(Vec2::from_point(pt));
                return r;
            }
        };

        struct Tank {
            datapod::String name;
            uint8_t type = 0; // 0=HARVEST, 1=WASTE
            float capacity = 0.0f;
            Bound bound;

            types::Tank to_tank() const {
                types::Tank t;
                t.name = std::string(name.view());
                t.type = static_cast<types::ContainerType>(type);
                t.capacity = capacity;
                t.bound = bound.to_box();
                return t;
            }

            static Tank from_tank(const types::Tank &t) {
                Tank r;
                r.name = datapod::String(t.name);
                r.type = static_cast<uint8_t>(t.type);
                r.capacity = t.capacity;
                r.bound = Bound::from_box(t.bound);
                return r;
            }
        };

        struct Power {
            datapod::String name;
            uint8_t type = 0; // 0=FUEL, 1=BATTERY
            float capacity = 0.0f;
            float consumption_rate = 0.0f;
            float charge_rate = 0.0f;

            types::Power to_power() const {
                types::Power p;
                p.name = std::string(name.view());
                p.type = static_cast<types::PowerType>(type);
                p.capacity = capacity;
                p.consumption_rate = consumption_rate;
                p.charge_rate = charge_rate;
                return p;
            }

            static Power from_power(const types::Power &p) {
                Power r;
                r.name = datapod::String(p.name);
                r.type = static_cast<uint8_t>(p.type);
                r.capacity = p.capacity;
                r.consumption_rate = p.consumption_rate;
                r.charge_rate = p.charge_rate;
                return r;
            }
        };

        struct Machine {
            uint32_t rci = 0;
            uint32_t group = 0;
            bool slave = false;
            datapod::String name;
            datapod::String uuid;
            datapod::String type;
            datapod::Vector<datapod::String> works_on;
            Capability capability;
            Color color;
            Bound bound;
            Polygon outline;
            datapod::Vector<Wheel> wheels;
            MachineControls controls;
            datapod::Vector<Hitch> hitches;
            datapod::Vector<Karosserie> karosseries;
            bool has_tank = false;
            Tank tank;
            bool has_power = false;
            Power power_source;
            uint8_t role = 0; // 0=MASTER, 1=FOLLOWER, 2=SLAVE
            float turning_radius = 1.0f;
            datapod::String seqid;

            types::Machine to_machine() const {
                types::Machine m;
                m.rci = rci;
                m.group = group;
                m.slave = slave;
                m.name = std::string(name.view());
                m.uuid = std::string(uuid.view());
                m.type = std::string(type.view());
                for (const auto &s : works_on) m.works_on.push_back(std::string(s.view()));
                m.capability = capability.to_capability();
                m.color = color.to_pigment();
                m.bound = bound.to_box();
                m.outline = outline.to_polygon();
                for (const auto &w : wheels) m.wheels.push_back(w.to_wheel());
                m.controls = controls.to_controls();
                for (const auto &h : hitches) {
                    auto hitch = h.to_hitch();
                    m.hitches[hitch.name] = hitch;
                }
                for (const auto &k : karosseries) m.karosseries.push_back(k.to_karosserie());
                if (has_tank) m.tank = tank.to_tank();
                if (has_power) m.power_source = power_source.to_power();
                m.role = static_cast<types::MachineRole>(role);
                m.turning_radius = turning_radius;
                m.seqid = std::string(seqid.view());
                return m;
            }

            static Machine from_machine(const types::Machine &m) {
                Machine r;
                r.rci = m.rci;
                r.group = m.group;
                r.slave = m.slave;
                r.name = datapod::String(m.name);
                r.uuid = datapod::String(m.uuid);
                r.type = datapod::String(m.type);
                for (const auto &s : m.works_on) r.works_on.push_back(datapod::String(s));
                r.capability = Capability::from_capability(m.capability);
                r.color = Color::from_pigment(m.color);
                r.bound = Bound::from_box(m.bound);
                r.outline = Polygon::from_polygon(m.outline);
                for (const auto &w : m.wheels) r.wheels.push_back(Wheel::from_wheel(w));
                r.controls = MachineControls::from_controls(m.controls);
                for (const auto &[name, h] : m.hitches) r.hitches.push_back(Hitch::from_hitch(h));
                for (const auto &k : m.karosseries) r.karosseries.push_back(Karosserie::from_karosserie(k));
                if (m.tank) {
                    r.has_tank = true;
                    r.tank = Tank::from_tank(*m.tank);
                }
                if (m.power_source) {
                    r.has_power = true;
                    r.power_source = Power::from_power(*m.power_source);
                }
                r.role = static_cast<uint8_t>(m.role);
                r.turning_radius = m.turning_radius;
                r.seqid = datapod::String(m.seqid);
                return r;
            }

            // Custom members() function for datapod serialization (struct has >10 fields)
            auto members() {
                return std::tie(rci, group, slave, name, uuid, type, works_on, capability, color, bound, outline,
                                wheels, controls, hitches, karosseries, has_tank, tank, has_power, power_source, role,
                                turning_radius, seqid);
            }

            auto members() const {
                return std::tie(rci, group, slave, name, uuid, type, works_on, capability, color, bound, outline,
                                wheels, controls, hitches, karosseries, has_tank, tank, has_power, power_source, role,
                                turning_radius, seqid);
            }
        };

        // Serializable bicycle control (velocity-based)
        struct BicycleControl {
            datapod::String uuid;
            float linear = 0.0f;
            float angular = 0.0f;
            float brake = 0.0f;

            types::BicycleControl to_control() const {
                types::BicycleControl c;
                c.uuid = std::string(uuid.view());
                c.linear = linear;
                c.angular = angular;
                c.brake = brake;
                return c;
            }

            static BicycleControl from_control(const types::BicycleControl &c) {
                BicycleControl r;
                r.uuid = datapod::String(c.uuid);
                r.linear = c.linear;
                r.angular = c.angular;
                r.brake = c.brake;
                return r;
            }
        };

        // Serializable per-wheel control
        struct WheelControl {
            datapod::String uuid;
            datapod::Vector<float> steering;
            datapod::Vector<float> throttle;
            float brake = 0.0f;

            types::WheelControl to_control() const {
                types::WheelControl c;
                c.uuid = std::string(uuid.view());
                for (const auto &s : steering) c.steering.push_back(s);
                for (const auto &t : throttle) c.throttle.push_back(t);
                c.brake = brake;
                return c;
            }

            static WheelControl from_control(const types::WheelControl &c) {
                WheelControl r;
                r.uuid = datapod::String(c.uuid);
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
            datapod::String uuid;
            uint64_t tick_seq = 0;
            Pose pose;
            Vec2 velocity;
            float angular_vel = 0.0f;
            datapod::Vector<WheelState> wheels;
        };

        // Sensor data from simulator (sent separately like MachineState)
        struct LidarData {
            datapod::Vector<float> ranges;
            datapod::Vector<float> angles;
            datapod::Vector<uint8_t> valid; // 0 or 1
            float min_range = 0.1f;
            float max_range = 30.0f;

            types::LidarData to_lidar() const {
                types::LidarData d;
                for (const auto &r : ranges) d.ranges.push_back(r);
                for (const auto &a : angles) d.angles.push_back(a);
                for (const auto &v : valid) d.valid.push_back(v != 0);
                d.min_range = min_range;
                d.max_range = max_range;
                return d;
            }

            static LidarData from_lidar(const types::LidarData &d) {
                LidarData r;
                for (const auto &v : d.ranges) r.ranges.push_back(v);
                for (const auto &v : d.angles) r.angles.push_back(v);
                for (const auto &v : d.valid) r.valid.push_back(v ? 1 : 0);
                r.min_range = d.min_range;
                r.max_range = d.max_range;
                return r;
            }
        };

        struct GpsData {
            double latitude = 0.0;
            double longitude = 0.0;
            double altitude = 0.0;
            float heading = 0.0f;
            float speed = 0.0f;

            types::GpsData to_gps() const { return {latitude, longitude, altitude, heading, speed}; }

            static GpsData from_gps(const types::GpsData &d) {
                return {d.latitude, d.longitude, d.altitude, d.heading, d.speed};
            }
        };

        struct ImuData {
            float accel_x = 0.0f;
            float accel_y = 0.0f;
            float accel_z = 9.81f;
            float gyro_z = 0.0f;
            float yaw = 0.0f;

            types::ImuData to_imu() const { return {accel_x, accel_y, accel_z, gyro_z, yaw}; }

            static ImuData from_imu(const types::ImuData &d) {
                return {d.accel_x, d.accel_y, d.accel_z, d.gyro_z, d.yaw};
            }
        };

        struct SensorState {
            datapod::String uuid;
            uint64_t tick_seq = 0;
            LidarData lidar;
            GpsData gps;
            ImuData imu;
            bool has_lidar = false;
            bool has_gps = false;
            bool has_imu = false;

            types::SensorData to_sensor_data() const {
                types::SensorData d;
                d.lidar = lidar.to_lidar();
                d.gps = gps.to_gps();
                d.imu = imu.to_imu();
                d.has_lidar = has_lidar;
                d.has_gps = has_gps;
                d.has_imu = has_imu;
                return d;
            }

            static SensorState from_sensor_data(const std::string &uuid, const types::SensorData &d) {
                SensorState s;
                s.uuid = datapod::String(uuid);
                s.lidar = LidarData::from_lidar(d.lidar);
                s.gps = GpsData::from_gps(d.gps);
                s.imu = ImuData::from_imu(d.imu);
                s.has_lidar = d.has_lidar;
                s.has_gps = d.has_gps;
                s.has_imu = d.has_imu;
                return s;
            }
        };

        // LIDAR configuration update from agent -> simulator
        struct LidarConfigMsg {
            datapod::String uuid;
            bool enabled = true;
            float min_range = 0.5f;
            float max_range = 15.0f;
            float fov_deg = 45.0f;
            float resolution_deg = 3.0f;

            types::LidarConfig to_config() const {
                types::LidarConfig c;
                c.enabled = enabled;
                c.min_range = min_range;
                c.max_range = max_range;
                c.fov_deg = fov_deg;
                c.resolution_deg = resolution_deg;
                return c;
            }

            static LidarConfigMsg from_config(const std::string &uuid, const types::LidarConfig &c) {
                LidarConfigMsg m;
                m.uuid = datapod::String(uuid);
                m.enabled = c.enabled;
                m.min_range = c.min_range;
                m.max_range = c.max_range;
                m.fov_deg = c.fov_deg;
                m.resolution_deg = c.resolution_deg;
                return m;
            }
        };

        struct WorldState {
            datapod::Vector<MachineState> machines;
        };

        // Message types for ZMQ protocol
        enum class MsgType : uint8_t {
            SPAWN,
            DESPAWN,
            CONTROL,
            HEARTBEAT,
            GET_STATE,
        };

        struct Request {
            MsgType type;
            Machine machine;      // For SPAWN
            WheelControl control; // For CONTROL (per-wheel)
            datapod::String uuid; // For DESPAWN and HEARTBEAT
        };

        struct RerunInfo {
            datapod::String grpc_address;   // e.g., "127.0.0.1:9876"
            datapod::String recording_id;   // Shared recording ID
            datapod::String application_id; // Application name
        };

        // ZMQ endpoints for agent <-> simulator communication
        // For IPC these are full `ipc://...` endpoints; for TCP full `tcp://host:port` endpoints.
        struct ZmqInfo {
            datapod::String uplink_endpoint;   // agent -> simulator (PUSH/PULL)
            datapod::String downlink_endpoint; // simulator -> agent (PUB/SUB)
        };

        struct Response {
            bool success = false;
            WorldState state;
            RerunInfo rerun;
            ZmqInfo zmq;
        };

        // Serializable State
        struct State {
            bool online = true;
            uint8_t role = 0; // 0=MASTER, 1=FOLLOWER, 2=SLAVE
            uint8_t mode = 0; // 0=IDLE, 1=CHARGING, 2=STOP, 3=PAUSE, 4=EMERGENCY, 5=TRANSPORT, 6=WORK
            bool turn_first = false;
            bool allow_move = true;
            float speed_scale = 1.0f;

            types::State to_state() const {
                types::State s;
                s.online = online;
                s.role = static_cast<types::Role>(role);
                s.mode = static_cast<types::OP>(mode);
                s.turn_first = turn_first;
                s.allow_move = allow_move;
                s.speed_scale = speed_scale;
                return s;
            }

            static State from_state(const types::State &s) {
                State r;
                r.online = s.online;
                r.role = static_cast<uint8_t>(s.role);
                r.mode = static_cast<uint8_t>(s.mode);
                r.turn_first = s.turn_first;
                r.allow_move = s.allow_move;
                r.speed_scale = s.speed_scale;
                return r;
            }
        };

    } // namespace ser

} // namespace types
