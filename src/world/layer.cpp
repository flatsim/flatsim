#include "multiverse/world/layer.hpp"

namespace mvs {
    Layer::Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum)
        : rec(rec), datum(datum), rnd(std::random_device()()) {}

    void Layer::init(LayerInfo info) {
        info.name = info.name == "unnamed" ? "unnamed" : info.name;
        info.uuid = info.uuid == "none" ? "unnamed" : info.uuid;

        this->info = info;
        auto field = info.bound;
        auto rows = static_cast<std::size_t>(field.size.x / info.resolution);
        auto cols = static_cast<std::size_t>(field.size.y / info.resolution);
        this->rows = rows;
        this->cols = cols;

        grid = concord::Grid<GridData>(rows, cols, info.resolution, datum, true, info.bound.pose);
        image.resize(grid.rows() * grid.cols() * 4, 0);
        for (auto &[p, gd] : grid) {
            gd.color.r = 50;
            gd.color.g = 50;
            gd.color.b = 50;
            gd.data = 0.0f;
        }

        auto corners = grid.corners();
        for (auto &p : corners) {
            enu_corners_.push_back({float(p.x), float(p.y), 0.0f});
            auto wgs_coords = p.toWGS(datum);
            wgs_corners_.push_back({float(wgs_coords.lat), float(wgs_coords.lon)});
        }
        enu_corners_.push_back(enu_corners_[0]);
        wgs_corners_.push_back(wgs_corners_[0]);

        for (auto &p : info.field) {
            auto wgs_coords = p.toWGS(datum);
            polygon_corners_wgs_.push_back({float(wgs_coords.lat), float(wgs_coords.lon)});
            polygon_corners_.push_back({float(p.x), float(p.y), 0.0f});
        }
        polygon_corners_wgs_.push_back(polygon_corners_wgs_[0]);
        polygon_corners_.push_back(polygon_corners_[0]);
    }

    void Layer::add_noise(bool in_polygon_only) {
        noise.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
        auto sz = std::max(info.bound.size.x, info.bound.size.y);
        noise.SetFrequency(sz / 300000.0f);
        noise.SetSeed(int(rnd()));
        for (std::size_t r = 0; r < grid.rows(); ++r) {
            for (std::size_t c = 0; c < grid.cols(); ++c) {
                auto &[pt, gd] = grid(r, c);
                if (in_polygon_only) {
                    if (pt.x > enu_corners_[0][0] && pt.x < enu_corners_[enu_corners_.size() - 1][0] &&
                        pt.y > enu_corners_[0][1] && pt.y < enu_corners_[enu_corners_.size() - 1][1]) {
                        gd.data = noise.GetNoise(float(r), float(c)) / 2;
                    }
                } else {
                    gd.data = noise.GetNoise(float(r), float(c)) / 2;
                }
            }
        }
        has_noise = true;
    }

    void Layer::color_field() {
        auto indices = grid.indices_within(info.field);
        for (auto idx : indices) {
            std::size_t r = idx / grid.cols();
            std::size_t c = idx % grid.cols();
            grid(r, c).second.color.r = float(info.color.r);
            grid(r, c).second.color.g = float(info.color.g);
            grid(r, c).second.color.b = float(info.color.b);
            if (has_noise) {
                auto val = utils::float_to_byte(grid(r, c).second.data);
                grid(r, c).second.color.r = utils::mapper(val, 0.0f, 255.0f, 100.0f, 155.0f);
            }
        }
    }

    void Layer::paint(pigment::RGB color, concord::Polygon brush) {
        auto indices = grid.indices_within(brush);
        for (auto idx : indices) {
            std::size_t r = idx / grid.cols();
            std::size_t c = idx % grid.cols();
            grid(r, c).second.color = color;
        }
    }

    void Layer::to_image(std::vector<uint8_t> &image) {
        const std::size_t rows = grid.rows();
        const std::size_t cols = grid.cols();
        // Make room for RGBA pixels
        image.resize(rows * cols * 4);
        for (std::size_t r = 0; r < rows; ++r) {
            for (std::size_t c = 0; c < cols; ++c) {
                auto &[pt, gd] = grid(r, c);
                // Compute rotated position
                std::size_t rotated_r = rows - 1 - r;
                std::size_t rotated_c = cols - 1 - c;
                // Index into the flat RGBA buffer
                std::size_t base = (rotated_r * cols + rotated_c) * 4;
                image[base + 0] = gd.color.r;
                image[base + 1] = gd.color.g;
                image[base + 2] = gd.color.b;
                image[base + 3] = gd.color.a;
            }
        }
    }

    void Layer::tick(float dt) {
        freq++;
        if (freq % 10 == 0) {
            visualize();
            freq = 0;
        }
    }

    concord::Grid<uint8_t> Layer::get_grid_data() {
        // Use the same size, resolution, datum and pose as the original grid:
        const auto rows = grid.rows();
        const auto cols = grid.cols();
        const auto res = info.resolution;  // meters per pixel
        const auto pose = info.bound.pose; // ENU shift & rotation

        // Construct a new Grid<uint8_t> with the same geo‐layout:
        concord::Grid<uint8_t> out(rows, cols, // dimensions
                                   res,        // cell size
                                   datum,      // reference datum
                                   /*centered=*/true, pose);

        // Copy over each cell’s “data” value, converting to byte:
        for (size_t r = 0; r < rows; ++r) {
            for (size_t c = 0; c < cols; ++c) {
                // grid(r,c).second.data is your float
                float val = grid(r, c).second.data;
                uint8_t b = utils::float_to_byte(val);
                out(r, c).second = b;
            }
        }

        return out;
    }

    void Layer::visualize() {
        rerun::Color colorz(this->info.color.r, this->info.color.g, this->info.color.b);
        to_image(image);
        rec->log_static(info.name + "/image",
                        rerun::Image::from_rgba32(image, {uint32_t(grid.cols()), uint32_t(grid.rows())}));

        auto border__ = rerun::components::LineStrip3D(enu_corners_);
        rec->log_static(info.name + "/border",
                        rerun::LineStrips3D(border__).with_colors({{colorz}}).with_radii({{0.1f}}));

        auto linestring = rerun::components::GeoLineString::from_lat_lon(wgs_corners_);
        rec->log_static(info.name + "/border",
                        rerun::GeoLineStrings(linestring).with_colors({{colorz}}).with_radii({{0.1f}}));

        auto polygon__ = rerun::components::LineStrip3D(polygon_corners_);
        rec->log_static(info.name + "/polygon",
                        rerun::LineStrips3D(polygon__).with_colors({{colorz}}).with_radii({{0.1f}}));

        auto polystr = rerun::components::GeoLineString::from_lat_lon(polygon_corners_wgs_);
        rec->log_static(info.name + "/polygon",
                        rerun::GeoLineStrings(polystr).with_colors({{colorz}}).with_radii({{0.1f}}));
    }

} // namespace mvs
