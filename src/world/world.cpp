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

    World::World(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum, Size size)
        : settings(datum, size), rec(rec) {
        init(settings);
    }
    World::~World() { world.reset(); }

    void World::init(WorldSettings settings) {
        settings.apply_gravity = false;
        world = std::make_unique<muli::World>(settings);
        float width = settings.get_size().width;
        float height = settings.get_size().height;

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

        for (float i = -width / 2; i < width / 2; i += settings.get_size().grid_size) {
            std::vector<Square> row;
            // std::vector<muli::Shape *> shape_row;
            for (float j = -height / 2; j < height / 2; j += settings.get_size().grid_size) {
                auto grid_size = settings.get_size().grid_size;
                float x = static_cast<float>(i) + grid_size / 2.0f;
                float y = static_cast<float>(j) + grid_size / 2.0f;
                row.push_back({x, y, grid_size});
                //--------------------------------------------------------------------------------
                // std::unique_ptr<muli::Shape> shape;
                // shape.reset(new muli::Polygon(grid_size / 2.0f));
                // muli::RigidBody *body = world->CreateBox(grid_size / 2.0f);
                // body->Translate({x, y});
                // row.push_back(body);
            }
            grid.push_back(row);
        }

        for (auto &row : grid) {
            for (auto &square : row) {
                enu_grid_.push_back({square.x_center, square.y_center, 0});
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

        auto gs = settings.get_size().grid_size / 2;
        rec->log("grid", rerun::Boxes3D::from_centers_and_half_sizes(enu_grid_, {{gs, gs, 0.0f}})
                             .with_colors({{200, 55, 155}})
                             .with_radii({{0.01f}}));
    }

} // namespace mvs
