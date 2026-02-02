#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "pigment/pigment.hpp"
#include <datapod/datapod.hpp>

namespace types {

    // ============================================================================
    // Core Types
    // ============================================================================

    enum class PowerType { FUEL, BATTERY };
    enum class MachineRole { MASTER, FOLLOWER, SLAVE };
    enum class ContainerType { HARVEST, WASTE };
    enum class Role { MASTER, FOLLOWER, SLAVE };
    enum class OP { IDLE, CHARGING, STOP, PAUSE, EMERGENCY, TRANSPORT, WORK };

    // ============================================================================
    // Physics Configuration
    // ============================================================================

    struct Physics {
        float linear_damping = 0.2f;
        float angular_damping = 0.2f;
        float force = 30.0f;
        float torque = 10.0f;
        float friction = 1.5f;
        float max_impulse = 2.0f;
        float brake = 10.0f;
        float drag = 0.5f;
    };

    // ============================================================================
    // Machine Components
    // ============================================================================

    struct MachineControls {
        std::vector<float> steerings_max;
        std::vector<float> throttles_max;
        std::vector<float> steerings_diff;
        std::vector<float> throttles_diff;
        std::vector<bool> left_side;
    };

    struct Wheel {
        std::string name;
        datapod::Box bound;
        pigment::RGB color;
        float steering_max = 0.0f;
        float throttle_max = 1.0f;
        float force = 300.0f;
        float friction = 0.9f;
        float max_impulse = 10.0f;
        float brake = 5.0f;
        float drag = 0.5f;
    };

    struct Section {
        std::string name;
        datapod::Box bound;
        pigment::RGB color;
        bool working = false;
    };

    struct Karosserie {
        std::string name;
        datapod::Box bound;
        pigment::RGB color;
        bool has_physics = true;
        std::vector<Section> sections;
    };

    struct Hitch {
        std::string name;
        datapod::Box bound;
        pigment::RGB color;
        bool is_master = true;
        bool hooked = false;
    };

    struct Container {
        std::string name;
        ContainerType type = ContainerType::HARVEST;
        float capacity;
        datapod::Box bound;
    };
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

    struct LidarConfig {
        bool enabled = false;
        float min_range = 0.5f;
        float max_range = 15.0f;
        float fov_deg = 45.0f;
        float resolution_deg = 3.0f;
    };

    // ============================================================================
    // Machine Definition
    // ============================================================================

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
        std::optional<LidarConfig> lidar;
        MachineRole role = MachineRole::MASTER;
        float turning_radius = 1.0f;
        std::string seqid = name;
    };

    // ============================================================================
    // Control Commands
    // ============================================================================

    struct BicycleControl {
        std::string uuid;
        float linear = 0.0f;
        float angular = 0.0f;
        float brake = 0.0f;
    };

    struct WheelControl {
        std::string uuid;
        std::vector<float> steering;
        std::vector<float> throttle;
        float brake = 0.0f;
    };

    // ============================================================================
    // State
    // ============================================================================

    struct State {
        bool online = true;
        Role role = Role::MASTER;
        OP mode = OP::IDLE;
        bool turn_first = false;
        bool allow_move = true;
        float speed_scale = 1.0f;
    };

    // ============================================================================
    // Sensor Data
    // ============================================================================

    struct LidarData {
        std::vector<float> ranges;
        std::vector<float> angles;
        std::vector<bool> valid;
        float min_range = 0.1f;
        float max_range = 30.0f;
    };

    struct GpsData {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        float heading = 0.0f;
        float speed = 0.0f;
    };

    struct ImuData {
        float accel_x = 0.0f;
        float accel_y = 0.0f;
        float accel_z = 9.81f;
        float gyro_z = 0.0f;
        float yaw = 0.0f;
    };

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
        datapod::Size size = datapod::Size(100.0, 100.0, 0.0);
        datapod::Geo datum;
    };

    struct StaticObstacle {
        size_t id = 0;
        datapod::Point position;
        double radius = 0.5;
        double uncertainty = 0.1;
    };

    struct DynamicObstacle {
        size_t id = 0;
        datapod::Point position;
        datapod::Point velocity;
        double radius = 0.5;
        double uncertainty = 0.3;
        double activation_distance = 10.0;
        bool is_active = false;
    };

    // ============================================================================
    // Serializable Types for netpipe/agent47 communication
    // ============================================================================

    namespace ser {

        struct Vec2 {
            float x = 0.0f;
            float y = 0.0f;

            datapod::Point to_point() const { return datapod::Point(x, y); }
            static Vec2 from_point(const datapod::Point &p) {
                return {static_cast<float>(p.x), static_cast<float>(p.y)};
            }
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

        struct WheelState {
            Pose pose;
            Vec2 velocity;
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

        struct LidarData {
            datapod::Vector<float> ranges;
            datapod::Vector<float> angles;
            datapod::Vector<uint8_t> valid;
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

        struct WorldState {
            datapod::Vector<MachineState> machines;
        };

    } // namespace ser

} // namespace types
