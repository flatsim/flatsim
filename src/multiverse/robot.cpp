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
            sensor->tick(dt, position);
        }
        this->position.point.enu.x = chassis->get_transform().position.x;
        this->position.point.enu.y = chassis->get_transform().position.y;
        this->position.point.enu.z = 0;
        this->position.point.wgs = this->position.point.enu.toWGS(datum);
        chassis->tick(dt);

        visualize();
    }

    void Robot::init(concord::Datum datum, concord::Pose pose, concord::Size size, pigment::RGB color, std::string name,
                     std::vector<concord::Size> wheel_sizes, std::vector<concord::Size> karosserie_sizes) {
        std::cout << "Initializing robot " << name << "...\n";
        this->color = color;
        this->name = name;
        this->size = size;
        this->spawn_position = pose;

        Transform t;
        t.position.x = pose.point.enu.x;
        t.position.y = pose.point.enu.y;
        t.rotation = pose.angle.yaw; // in radians

        concord::Bound bound;
        bound.size = size;
        bound.pose = pose;

        chassis = std::make_unique<Chasis>(world, rec, bound, color, name, wheel_sizes, filter);
        chassis->init(bound, color, name, wheel_sizes, filter);

        pulse = concord::Circle(this->position.point, 0.0);
    }

    void Robot::update(float steering[4], float throttle[4]) { chassis->update(steering, throttle); }
    void Robot::teleport(concord::Pose pose) { chassis->teleport(pose); }
    void Robot::respawn() { chassis->teleport(spawn_position); }

    void Robot::visualize() {
        chassis->visualize();

        auto x = this->position.point.enu.x;
        auto y = this->position.point.enu.y;

        std::vector<rerun::Color> colors;
        colors.push_back(rerun::Color(color.r, color.g, color.b));

        rec->log_static(this->name + "/pose", rerun::Points3D({{float(x), float(y), 0}}).with_colors(colors));

        auto lat = float(this->position.point.wgs.lat);
        auto lon = float(this->position.point.wgs.lon);
        std::vector<rerun::LatLon> locators;
        locators.push_back(rerun::LatLon(lat, lon));
        rec->log_static(this->name + "/pose", rerun::GeoPoints(locators).with_colors(colors));

        // for (auto &ks : karosserie) {
        //     auto k_x = ks->GetPosition().x;
        //     auto k_y = ks->GetPosition().y;
        //     auto k_th = ks->GetRotation().GetAngle();
        //     auto k_w = float(size.x * 1.3);
        //     auto k_h = float(size.y * 1.3);
        //     std::vector<rerun::Color> colors_a;
        //     colors_a.push_back(rerun::Color(color.r, color.g, color.b, 40));
        //     rec->log_static(this->name + "/karosserie",
        //                     rerun::Boxes3D::from_centers_and_half_sizes({{k_x, k_y, 0}}, {{k_w / 2, k_h / 2, 0.0f}})
        //                         .with_radii({{0.02f}})
        //                         // .with_fill_mode(rerun::FillMode::Solid)
        //                         .with_rotation_axis_angles(
        //                             {rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(k_th))})
        //                         .with_colors(colors_a));
        // }

        // Vec2 size = {this->size[0], this->size[1]};
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
            // Get local offset
            Vec2 localOffset = arrow_head_offsets[i];

            // Rotate the offset according to car's rotation
            Vec2 rotatedOffset;
            rotatedOffset.x = localOffset.x * t.rotation.c - localOffset.y * t.rotation.s;
            rotatedOffset.y = localOffset.x * t.rotation.s + localOffset.y * t.rotation.c;

            // Add the rotated offset to the car's position
            arrow_world_points[i].x = t.position.x + rotatedOffset.x;
            arrow_world_points[i].y = t.position.y + rotatedOffset.y;
        }

        const rerun::Position3D vertex_positions[3] = {{arrow_world_points[0].x, arrow_world_points[0].y, 0.0f},
                                                       {arrow_world_points[1].x, arrow_world_points[1].y, 0.0f},
                                                       {arrow_world_points[2].x, arrow_world_points[2].y, 0.0f}};

        pigment::HSV hsv = pigment::HSV::fromRGB(color);
        hsv.adjustBrightness(0.7f);
        auto c = hsv.toRGB();

        const rerun::Color vertex_colors[3] = {
            {static_cast<uint8_t>(c.r), static_cast<uint8_t>(c.g), static_cast<uint8_t>(c.b)},
            {static_cast<uint8_t>(c.r), static_cast<uint8_t>(c.g), static_cast<uint8_t>(c.b)},
            {static_cast<uint8_t>(c.r), static_cast<uint8_t>(c.g), static_cast<uint8_t>(c.b)},
        };
        rec->log_static(this->name + "/heading", rerun::Mesh3D(vertex_positions)
                                                     .with_vertex_normals({{0.0, 0.0, 1.0}})
                                                     .with_vertex_colors(vertex_colors)
                                                     .with_triangle_indices({{2, 1, 0}}));
        pulse_vis(std::max(size.x, size.y) * 3.0f);
    }

    void Robot::pulse_vis(float p_s) {
        auto pulse_size = pulse.getRadius() + 0.0015;
        auto hsv = pigment::HSV::fromRGB(color);
        hsv.adjustBrightness(0.5f);
        auto c = hsv.toRGB();
        auto this_c = rerun::Color(c.r, c.g, c.b);
        if (!pulsining) {
            return;
        } else if (pulse.getRadius() > p_s) {
            pulsining = false;
            pulse_size = 0.0;
        }
        auto x = this->position.point.enu.x;
        auto y = this->position.point.enu.y;
        concord::Point p;
        p.enu.x = x;
        p.enu.y = y;
        pulse = concord::Circle(p, pulse_size);
        auto pointss = pulse.as_polygon(50);
        std::vector<rerun::Vec3D> poi;
        for (auto &point : pointss) {
            poi.push_back({float(point.enu.x), float(point.enu.y), 0.0f});
        }
        poi.push_back({float(pointss[0].enu.x), float(pointss[0].enu.y), 0.0f});
        rec->log_static(
            this->name + "/pulse2",
            rerun::LineStrips3D({{poi}})
                .with_colors({{this_c}})
                .with_radii({{float(mapValue(pulse_size, 0.0, std::max(size.x, size.y) * 3.0f, 0.03, 0.0005))}}));
    }
} // namespace mvs
