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

        // Visualize static obstacles (RED boxes)
        for (const auto &obs : static_obstacles) {
            std::string name = "obstacles/static_" + std::to_string(obs.id);
            rec->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                      {{float(obs.position.x), float(obs.position.y), 0.0f}},
                                      {{float(obs.radius), float(obs.radius), 0.3f}})
                                      .with_colors(rerun::Color(255, 0, 0)));
        }

        // Visualize dynamic obstacles (GREEN boxes)
        for (const auto &obs : dynamic_obstacles) {
            std::string name = "obstacles/dynamic_" + std::to_string(obs.id);
            rec->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                      {{float(obs.position.x), float(obs.position.y), 0.0f}},
                                      {{float(obs.radius), float(obs.radius), 0.3f}})
                                      .with_colors(rerun::Color(0, 255, 0)));
        }
    }

    // Obstacle management
    void World::add_obstacle(const StaticObstacle &obs) { static_obstacles.push_back(obs); }

    void World::add_obstacle(const DynamicObstacle &obs) { dynamic_obstacles.push_back(obs); }

    void World::clear_obstacles() {
        static_obstacles.clear();
        dynamic_obstacles.clear();
    }

    void World::update_obstacles(float dt, double robot_x, double robot_y) {
        for (auto &obs : dynamic_obstacles) {
            obs.update(dt, robot_x, robot_y);
        }
    }

    drivekit::WorldConstraints World::get_world_constraints(size_t horizon_steps, double dt) const {
        drivekit::WorldConstraints constraints;

        // Add static obstacles
        for (const auto &obs : static_obstacles) {
            drivekit::Obstacle dk_obs;
            dk_obs.id = 1000 + obs.id; // Offset to avoid ID collision with dynamic
            dk_obs.radius = obs.radius;

            drivekit::Obstacle::GaussianMode mode;
            mode.weight = 1.0;
            for (size_t t = 0; t <= horizon_steps; ++t) {
                // Static obstacles don't move
                mode.mean_x.push_back(obs.position.x);
                mode.mean_y.push_back(obs.position.y);
                mode.std_x.push_back(obs.uncertainty);
                mode.std_y.push_back(obs.uncertainty);
            }
            dk_obs.modes.push_back(mode);
            constraints.obstacles.push_back(dk_obs);
        }

        // Add dynamic obstacles with trajectory prediction
        for (const auto &obs : dynamic_obstacles) {
            drivekit::Obstacle dk_obs;
            dk_obs.id = obs.id;
            dk_obs.radius = obs.radius;

            drivekit::Obstacle::GaussianMode mode;
            mode.weight = 1.0;
            for (size_t t = 0; t <= horizon_steps; ++t) {
                double pred_time = t * dt;
                auto pred_pos = obs.predict(pred_time);
                mode.mean_x.push_back(pred_pos.x);
                mode.mean_y.push_back(pred_pos.y);
                mode.std_x.push_back(obs.uncertainty);
                mode.std_y.push_back(obs.uncertainty);
            }
            dk_obs.modes.push_back(mode);
            constraints.obstacles.push_back(dk_obs);
        }

        return constraints;
    }

} // namespace fs
