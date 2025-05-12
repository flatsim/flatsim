#include "multiverse/robot.hpp"
#include "pigment/types_hsv.hpp"

namespace mvs {

    double mapValue(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group)
        : rec(rec), world(world), group(group) {
        filter.bit = 1 << group;
        filter.mask = ~(1 << group);
    }
    Robot::~Robot() {}

    void Robot::tick(float dt) {
        for (auto &sensor : sensors) {
            sensor->tick(dt, pose);
        }
        this->pose.point.enu.x = chassis->get_transform().position.x;
        this->pose.point.enu.y = chassis->get_transform().position.y;
        this->pose.point.wgs = this->pose.point.enu.toWGS(datum);
        chassis->tick(dt);
        chassis->update(steerings, throttles);

        visualize();
    }

    void Robot::init(concord::Datum datum, Robo robo) {
        spdlog::info("Initializing robot {}...", name);
        this->datum = datum;
        this->color = robo.color;
        this->name = robo.name;
        this->uuid = robo.uuid;
        this->size = robo.bound.size;
        this->pose = robo.bound.pose;
        this->spawn_position = robo.bound.pose;

        chassis = std::make_unique<Chasis>(world, rec, filter);
        chassis->init(robo.bound, color, name, robo.wheels, robo.karosserie);

        steerings.resize(robo.wheels.size(), 0.0f);
        steerings_max = robo.controlz.steerings_max;
        steerings_diff = robo.controlz.steerings_diff;
        throttles.resize(robo.wheels.size(), 0.0f);
        throttles_max = robo.controlz.throttles_max;
        throttles_diff = robo.controlz.throttles_diff;
    }

    void Robot::reset_controls() {
        for (uint i = 0; i < steerings.size(); ++i) {
            steerings[i] = 0.0f;
        }
        for (uint i = 0; i < throttles.size(); ++i) {
            throttles[i] = 0.0f;
        }
    }

    void Robot::set_angular(float angular) {
        constexpr float in_min = -1.0f, in_max = 1.0f;
        const float sign = (angular < 0.0f ? -1.0f : 1.0f);
        for (size_t i = 0; i < steerings.size(); ++i) {
            float o1 = steerings_max[i] - sign * steerings_diff[i];
            float o2 = -steerings_max[i] + sign * steerings_diff[i];
            steerings[i] = mapValue(angular, in_min, in_max, o1, o2);
        }
    }

    void Robot::set_linear(float linear) {
        for (uint i = 0; i < throttles.size(); ++i) {
            throttles[i] = mapValue(linear, -1.0f, 1.0f, throttles_max[i], -throttles_max[i]);
        }
    }

    void Robot::teleport(concord::Pose pose) {
        reset_controls();
        chassis->teleport(pose);
    }
    void Robot::respawn() {
        reset_controls();
        chassis->teleport(spawn_position);
    }

    void Robot::visualize() {
        chassis->visualize();

        auto x = this->pose.point.enu.x;
        auto y = this->pose.point.enu.y;

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(color.r, color.g, color.b));

        rec->log_static(this->name + "/pose", rerun::Points3D({{float(x), float(y), 0.1f}}).with_colors(colors));

        auto lat = float(this->pose.point.wgs.lat);
        auto lon = float(this->pose.point.wgs.lon);
        std::vector<rerun::LatLon> locators;
        locators.push_back(rerun::LatLon(lat, lon));
        rec->log_static(this->name + "/pose", rerun::GeoPoints(locators).with_colors(colors));

        Transform t = chassis->get_transform();
        const float arrowHeight = size.y * -0.03f; // How far the tip extends beyond the chassis
        const float arrowWidth = size.x * 0.5f;    // Width of arrow base

        std::array<Vec2, 3> arrow_head_offsets = {{
            {Vec2(0, size.y / 2 + arrowHeight)},   // tip of arrow (forward of the chassis)
            {Vec2(-arrowWidth / 2, size.y * 0.3)}, // left corner of arrow
            {Vec2(arrowWidth / 2, size.y * 0.3)}   // right corner of arrow
        }};

        std::array<Vec2, 3> arrow_world_points;
        for (int i = 0; i < 3; ++i) {
            Vec2 localOffset = arrow_head_offsets[i];
            Vec2 rotatedOffset;
            rotatedOffset.x = localOffset.x * t.rotation.c - localOffset.y * t.rotation.s;
            rotatedOffset.y = localOffset.x * t.rotation.s + localOffset.y * t.rotation.c;
            arrow_world_points[i].x = t.position.x + rotatedOffset.x;
            arrow_world_points[i].y = t.position.y + rotatedOffset.y;
        }
        const rerun::Position3D vertex_positions[3] = {{arrow_world_points[0].x, arrow_world_points[0].y, 0.1f},
                                                       {arrow_world_points[1].x, arrow_world_points[1].y, 0.1f},
                                                       {arrow_world_points[2].x, arrow_world_points[2].y, 0.1f}};
        const rerun::Color vertex_colors[3] = {
            {static_cast<uint8_t>(color.r), static_cast<uint8_t>(color.g), static_cast<uint8_t>(color.b)},
            {static_cast<uint8_t>(color.r), static_cast<uint8_t>(color.g), static_cast<uint8_t>(color.b)},
            {static_cast<uint8_t>(color.r), static_cast<uint8_t>(color.g), static_cast<uint8_t>(color.b)},
        };
        rec->log_static(this->name + "/heading", rerun::Mesh3D(vertex_positions)
                                                     .with_vertex_normals({{0.0, 0.0, 1.0}})
                                                     .with_vertex_colors(vertex_colors)
                                                     .with_triangle_indices({{2, 1, 0}}));
        visualize_pulse(std::max(size.x, size.y) * 3.0f);
    }

    void Robot::visualize_pulse(float p_s, float gps_mult, float inc) {
        concord::Point point(this->pose.point.enu.x, this->pose.point.enu.y, 0.0f, datum);
        if (!pulsining) {
            return;
        }

        // visualize local pulse
        std::vector<rerun::Vec3D> poi;
        auto pulse_enu_size = pulse_enu.getRadius() + inc;
        if (pulse_enu.getRadius() > p_s) {
            pulsining = false;
            pulse_enu_size = 0.0;
        }
        pulse_enu = concord::Circle(point, pulse_enu_size);
        auto pointss = pulse_enu.as_polygon(50, datum);
        for (auto &point : pointss) {
            poi.push_back({float(point.enu.x), float(point.enu.y), 0.0f});
        }
        poi.push_back({float(pointss[0].enu.x), float(pointss[0].enu.y), 0.0f});
        rec->log_static(
            this->name + "/pulse/enu",
            rerun::LineStrips3D({{poi}})
                .with_colors({{rerun::Color(color.r, color.g, color.b)}})
                .with_radii({{float(mapValue(pulse_enu_size, 0.0, std::max(size.x, size.y) * 3.0f, 0.03, 0.0005))}}));

        // visualize gps pulse
        std::vector<rerun::LatLon> locators;
        auto pulse_gps_size = pulse_gps.getRadius() + inc * gps_mult;
        if (pulse_gps.getRadius() > p_s * gps_mult) {
            pulsining = false;
            pulse_gps_size = 0.0;
        }
        pulse_gps = concord::Circle(point, pulse_gps_size);
        auto pointss_gps = pulse_gps.as_polygon(50, datum);
        for (auto &point : pointss_gps) {
            locators.push_back(rerun::LatLon(point.wgs.lat, point.wgs.lon));
        }
        locators.push_back(rerun::LatLon(pointss_gps[0].wgs.lat, pointss_gps[0].wgs.lon));
        auto linestr = rerun::components::GeoLineString::from_lat_lon(locators);
        rec->log_static(
            this->name + "/pulse/wgs",
            rerun::GeoLineStrings(linestr)
                .with_colors(rerun::Color(color.r, color.g, color.b))
                .with_radii({{float(mapValue(pulse_gps_size, 0.0, std::max(size.x, size.y) * 3.0f * gps_mult,
                                             0.03 * gps_mult, 0.0005 * gps_mult))}}));
    }
} // namespace mvs
