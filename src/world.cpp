#include "flatsim/world.hpp"

namespace fs {
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
    World::~World() = default;

    void World::init(concord::Datum datum, concord::Size world_size) {
        settings.init(datum, world_size);
        settings.apply_gravity = false;
        world = std::make_unique<muli::World>(settings);
        float width = settings.get_world_size().x;
        float height = settings.get_world_size().y;

        world_bounds.from_pointvec({
            concord::Point(width / 2.0f, -height / 2.0f, 0.0f),
            concord::Point(-width / 2.0f, -height / 2.0f, 0.0f),
            concord::Point(width / 2.0f, height / 2.0f, 0.0f),
            concord::Point(-width / 2.0f, height / 2.0f, 0.0f),
        });
    }

    void World::tick(float dt) { world->Step(dt); }

    void World::adjust_word() {
        // No layers to adjust world bounds
    }

    void World::tock() {
        // World boundaries are static, so use thread-local cache to avoid repeated allocations
        static thread_local std::vector<std::array<float, 3>> enu_corners_;
        static thread_local std::vector<rerun::LatLon> wgs_corners_;
        static thread_local bool initialized = false;

        // Only compute corners once since world boundaries don't change
        if (!initialized) {
            enu_corners_.clear();
            wgs_corners_.clear();
            enu_corners_.reserve(5); // 4 corners + closing point
            wgs_corners_.reserve(5);

            for (auto corner : world_bounds.get_corners()) {
                float x = static_cast<float>(corner.x);
                float y = static_cast<float>(corner.y);
                float z = static_cast<float>(corner.z);
                enu_corners_.push_back({x, y, z});
                auto wgs_coords = corner.toWGS(settings.get_datum());
                float lat = static_cast<float>(wgs_coords.lat);
                float lon = static_cast<float>(wgs_coords.lon);
                wgs_corners_.push_back({lat, lon});
            }
            enu_corners_.push_back(enu_corners_[0]);
            wgs_corners_.push_back(wgs_corners_[0]);
            initialized = true;
        }

        auto border__ = rerun::components::LineStrip3D(enu_corners_);
        rec->log_static("border", rerun::LineStrips3D(border__).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

        auto linestring = rerun::components::GeoLineString::from_lat_lon(wgs_corners_);
        rec->log_static("border", rerun::GeoLineStrings(linestring).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

        // rec->log_static("grid", rerun::Boxes3D::from_centers_and_sizes(
        //                             grid.getGrid().flatten_points(),
        //                             {{float(grid.getGrid().inradius()), float(grid.getGrid().inradius()), 0.0f}})
        //                             .with_colors(rerun::Color(110, 90, 60))
        //                             .with_radii({{0.005f}}));
    }

} // namespace fs
