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

    void World::init(concord::Datum datum, Size world_size, Size grid_size) {
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

        for (float i = -width / 2; i < width / 2; i += settings.get_grid_size().x) {
            std::vector<Square> row;
            for (float j = -height / 2; j < height / 2; j += settings.get_grid_size().y) {
                float x = static_cast<float>(i) + settings.get_grid_size().x / 2.0f;
                float y = static_cast<float>(j) + settings.get_grid_size().y / 2.0f;
                auto enu = concord::ENU(x, y, 0.0f);
                auto wgs = enu.toWGS(settings.get_datum());
                row.push_back({{enu, wgs}, settings.get_grid_size().x});
                //--------------------------------------------------------------------------------
                // muli::RigidBody *body = world->CreateBox(grid_size / 2.0f);
                // body->Translate({x, y});
                // row.push_back(body);
            }
            grid.push_back(row);
        }

        for (auto &row : grid) {
            for (auto &square : row) {
                enu_grid_.push_back({float(square.getCenter().enu.x), float(square.getCenter().enu.y), 0});
            }
        }

        visualize_once();
    }
    void World::tick(float dt) { world->Step(dt); }

    void World::visualize_once() {
        auto border__ = rerun::components::LineStrip3D(enu_corners_);
        rec->log_static("border", rerun::LineStrips3D(border__).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

        auto linestring = rerun::components::GeoLineString::from_lat_lon(wgs_corners_);
        rec->log_static("border", rerun::GeoLineStrings(linestring).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

        auto gsx = float(settings.get_grid_size().x / 2);
        auto gsy = float(settings.get_grid_size().z / 2);
        rec->log("grid", rerun::Boxes3D::from_centers_and_half_sizes(enu_grid_, {{gsx, gsy, 0.0f}})
                             .with_colors({{200, 55, 155}})
                             .with_radii({{0.01f}}));
    }

} // namespace mvs
