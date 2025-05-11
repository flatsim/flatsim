#include "multiverse/world.hpp"

namespace mvs {
    namespace utl {
        std::vector<concord::ENU> build_corners(float width, float height) {
            std::vector<concord::ENU> corners;
            corners.push_back(concord::ENU(-width / 2.0f, -height / 2.0f, 0.0f));
            corners.push_back(concord::ENU(width / 2.0f, -height / 2.0f, 0.0f));
            corners.push_back(concord::ENU(width / 2.0f, height / 2.0f, 0.0f));
            corners.push_back(concord::ENU(-width / 2.0f, height / 2.0f, 0.0f));
            corners.push_back(concord::ENU(-width / 2.0f, -height / 2.0f, 0.0f));
            return corners;
        }
    } // namespace utl

    World::World(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}
    World::~World() { world.reset(); }

    void World::init(concord::Datum datum, concord::Size world_size, float grid_size) {

        settings.init(datum, world_size, grid_size);
        settings.apply_gravity = false;
        world = std::make_unique<muli::World>(settings);
        float width = settings.get_world_size().x;
        float height = settings.get_world_size().y;

        for (auto corner : utl::build_corners(width, height)) {
            float x = static_cast<float>(corner.x);
            float y = static_cast<float>(corner.y);
            float z = static_cast<float>(corner.z);
            enu_corners_.push_back({x, y, z});
            auto wgs_C = corner.toWGS(settings.get_datum());
            float lat = wgs_C.lat;
            float lon = wgs_C.lon;
            wgs_corners_.push_back({lat, lon});
        }

        int g_width = static_cast<int>(settings.get_world_size().x / settings.get_grid_size());
        int g_height = static_cast<int>(settings.get_world_size().y / settings.get_grid_size());
        grid = Layer(rec, g_width, g_height, settings.get_grid_size(), settings.get_datum());
    }
    void World::tick(float dt) {
        world->Step(dt);
        visualize();
    }
    //
    void World::visualize() {
        grid.visualize();

        auto border__ = rerun::components::LineStrip3D(enu_corners_);
        rec->log_static("border", rerun::LineStrips3D(border__).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

        auto linestring = rerun::components::GeoLineString::from_lat_lon(wgs_corners_);
        rec->log_static("border", rerun::GeoLineStrings(linestring).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));
    }

    void World::add_layer(std::string name, float inradius) {
        auto layer = std::make_shared<Layer>(rec, grid.getGrid().rows(), grid.getGrid().cols(), inradius);
        layers.push_back(layer);
    }

} // namespace mvs
