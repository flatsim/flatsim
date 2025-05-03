#include "multiverse/robot.hpp"

namespace mvs {
    Robot::Robot(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<mvs::World> world)
        : rec(rec), world(world) {}
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

    void Robot::init(concord::Pose pose, std::string name) {
        std::cout << "Initializing robot " << name << "...\n";
        this->name = name;
        this->size.x = 0.8f;
        this->size.y = 1.4f;

        chassis = std::make_unique<Vehicle>(world->get_world().get(), pose, size);
    }

    void Robot::visualize() {
        std::vector<rerun::Position3D> points;
        auto x = this->position.point.enu.x;
        auto y = this->position.point.enu.y;
        points.push_back(rerun::Position3D(x, y, 0));

        rec->log_static(this->name, rerun::Points3D(points));

        auto lat = float(this->position.point.wgs.lat);
        auto lon = float(this->position.point.wgs.lon);

        std::vector<rerun::LatLon> locators;
        locators.push_back(rerun::LatLon(lat, lon));
        rec->log_static(this->name, rerun::GeoPoints(locators));
    }

    // void Robot::teleport(float x, float y) { chassis->teleport({x, y}); }
} // namespace mvs
