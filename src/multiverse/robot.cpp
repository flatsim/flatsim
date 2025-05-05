#include "multiverse/robot.hpp"

namespace mvs {
    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<mvs::World> world, uint32_t collision_id)
        : rec(rec), world(world), collision_id(collision_id) {}
    Robot::~Robot() {}

    void Robot::tick(float dt) {
        for (auto &sensor : sensors) {
            sensor->tick(dt, position);
        }
        this->position.point.enu.x = chassis->body->GetPosition().x;
        this->position.point.enu.y = chassis->body->GetPosition().y;
        this->position.point.enu.z = 0;
        this->position.point.wgs = this->position.point.enu.toWGS(world->get_settings().get_datum());
        chassis->tick(dt);
        visualize();
    }

    void Robot::init(concord::Pose pose, pigment::RGB color, std::string name) {
        std::cout << "Initializing robot " << name << "...\n";
        this->color = color;
        this->name = name;
        this->size.x = 0.8f;
        this->size.y = 1.4f;
        this->spawn_position = pose;

        chassis = std::make_unique<Vehicle>(world->get_world().get(), rec, pose, size, color, name, collision_id);

        // shape.addPoint(pose.point.enu, world->get_settings().get_datum());
    }

    void Robot::update(float steering[4], float throttle[4]) { chassis->update(steering, throttle); }

    void Robot::visualize() {
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
        chassis->visualize();
    }

    void Robot::teleport(concord::Pose pose) { chassis->teleport(pose); }
    void Robot::respawn() { chassis->teleport(spawn_position); }
} // namespace mvs
