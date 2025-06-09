#include "multiverse/robot.hpp"

namespace mvs {

    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world, uint32_t group)
        : rec(rec), world(world) {
        filter.bit = 1 << group;
        filter.mask = ~(1 << group);
    }
    Robot::~Robot() {}

    void Robot::tick(float dt) {
        for (auto &sensor : sensors) {
            sensor->tick(dt, info.bound.pose);
        }
        this->info.bound.pose.point.enu.x = chassis->get_transform().position.x;
        this->info.bound.pose.point.enu.y = chassis->get_transform().position.y;
        this->info.bound.pose.point.wgs = this->info.bound.pose.point.enu.toWGS(datum);
        chassis->tick(dt);
        chassis->update(steerings, throttles);

        visualize();
    }

    void Robot::init(concord::Datum datum, RobotInfo robo) {
        spdlog::info("Initializing robot {}...", info.name);
        this->datum = datum;
        this->info = robo;
        this->spawn_position = robo.bound.pose;

        chassis = std::make_unique<Chasis>(world, rec, filter);
        chassis->init(robo);

        steerings.resize(robo.wheels.size(), 0.0f);
        steerings_max = robo.controlz.steerings_max;
        steerings_diff = robo.controlz.steerings_diff;
        throttles.resize(robo.wheels.size(), 0.0f);
        throttles_max = robo.controlz.throttles_max;
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
            steerings[i] = utils::mapper(angular, in_min, in_max, o1, o2);
        }
    }

    void Robot::set_linear(float linear) {
        constexpr float in_min = -1.0f, in_max = 1.0f;
        for (uint i = 0; i < throttles.size(); ++i) {
            auto lin_val = linear;
            if (steerings[i] > 0.0f && info.controlz.left_side[i]) {
                auto proportion = utils::ackermann_scale(steerings[i], info.bound.size.x);
                lin_val = linear * proportion;
            } else if (steerings[i] < 0.0f && !info.controlz.left_side[i]) {
                auto proportion = utils::ackermann_scale(steerings[i], info.bound.size.x);
                lin_val = linear * proportion;
            }
            throttles[i] = utils::mapper(lin_val, in_min, in_max, throttles_max[i], -throttles_max[i]);
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

        auto x = this->info.bound.pose.point.enu.x;
        auto y = this->info.bound.pose.point.enu.y;

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(info.color.r, info.color.g, info.color.b));

        rec->log_static(this->info.name + "/pose", rerun::Points3D({{float(x), float(y), 0.1f}}).with_colors(colors));

        auto lat = float(this->info.bound.pose.point.wgs.lat);
        auto lon = float(this->info.bound.pose.point.wgs.lon);
        std::vector<rerun::LatLon> locators;
        locators.push_back(rerun::LatLon(lat, lon));
        rec->log_static(this->info.name + "/pose", rerun::GeoPoints(locators).with_colors(colors));

        visualize_pulse(std::max(info.bound.size.x, info.bound.size.y) * 3.0f);
    }

    void Robot::visualize_pulse(float p_s, float gps_mult, float inc) {
        concord::Point point(this->info.bound.pose.point.enu.x, this->info.bound.pose.point.enu.y, 0.0f, datum);
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
            this->info.name + "/pulse/enu",
            rerun::LineStrips3D({{poi}})
                .with_colors({{rerun::Color(info.color.r, info.color.g, info.color.b)}})
                .with_radii({{float(utils::mapper(pulse_enu_size, 0.0, std::max(info.bound.size.x, info.bound.size.y) * 3.0f,
                                             0.03, 0.0005))}}));

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
        rec->log_static(this->info.name + "/pulse/wgs",
                        rerun::GeoLineStrings(linestr)
                            .with_colors(rerun::Color(info.color.r, info.color.g, info.color.b))
                            .with_radii({{float(utils::mapper(
                                pulse_gps_size, 0.0, std::max(info.bound.size.x, info.bound.size.y) * 3.0f * gps_mult,
                                0.03 * gps_mult, 0.0005 * gps_mult))}}));
    }
} // namespace mvs
