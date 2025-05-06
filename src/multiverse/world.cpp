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

    void World::init(concord::Datum datum, concord::Size world_size, concord::Size grid_size) {
        std::vector<std::array<float, 3>> enu_corners_;
        std::vector<rerun::LatLon> wgs_corners_;

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

        int g_width = static_cast<int>(settings.get_world_size().x / settings.get_grid_size().x);
        int g_height = static_cast<int>(settings.get_world_size().y / settings.get_grid_size().y);
        // the_grid = concord::Grid<pigment::RGB>(g_width, g_height, settings.get_grid_size().y);
        grid = Layer<pigment::RGB>(g_width, g_height, settings.get_grid_size().y);

        auto border__ = rerun::components::LineStrip3D(enu_corners_);
        rec->log_static("border", rerun::LineStrips3D(border__).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

        auto linestring = rerun::components::GeoLineString::from_lat_lon(wgs_corners_);
        rec->log_static("border", rerun::GeoLineStrings(linestring).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));
    }
    void World::tick(float dt) {
        world->Step(dt);
        visualize();
        grid.visualize(rec);
    }
    //
    void World::visualize() {
        // std::vector<std::array<float, 3>> enu_grid_;
        // std::vector<rerun::components::Color> enu_colors;
        //
        // for (std::size_t r = 0; r < the_grid.rows(); ++r) {
        //     for (std::size_t c = 0; c < the_grid.cols(); ++c) {
        //         enu_grid_.push_back({(float(the_grid(r, c).first.enu.x)), (float(the_grid(r, c).first.enu.y)), 0});
        //         auto &[pt, color] = the_grid(r, c); // now color is an RGB& directly
        //         color.r = 110;
        //         color.g = 90;
        //         color.b = 60;
        //         rerun::datatypes::Rgba32 a_color{uint8(the_grid(r, c).second.r), uint8(the_grid(r, c).second.g),
        //                                          uint8(the_grid(r, c).second.b), 255};
        //         enu_colors.push_back(a_color);
        //     }
        // }
        //
        // // visualize
        // auto gsx = float(settings.get_grid_size().x / 2);
        // auto gsy = float(settings.get_grid_size().z / 2);
        // rec->log_static("grid", rerun::Boxes3D::from_centers_and_half_sizes(enu_grid_, {{gsx, gsy, 0.0f}})
        //                             .with_colors(enu_colors)
        //                             .with_radii({{0.005f}}));
    }

} // namespace mvs
