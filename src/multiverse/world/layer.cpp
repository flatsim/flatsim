#include "multiverse/world/layer.hpp"

namespace mvs {

    inline uint8_t floatToByte(float v) {
        v = std::clamp(v, 0.0f, 1.0f);
        float scaled = v * 255.0f;
        return static_cast<uint8_t>(std::round(scaled));
    }

    Layer::Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum)
        : rec(rec), datum(datum), rnd(std::random_device()()) {}

    void Layer::init(LayerInfo info) {
        this->info = info;
        auto field = info.bound;
        auto rows = static_cast<std::size_t>(field.size.x / info.resolution);
        auto cols = static_cast<std::size_t>(field.size.y / info.resolution);
        this->rows = rows;
        this->cols = cols;

        grid = concord::Grid<GridData>(rows, cols, info.resolution, datum, true, info.bound.pose);
        image.resize(grid.rows() * grid.cols() * 4, 0);
        for (auto &[p, gd] : grid) {
            gd.color.r = float(info.color.r);
            gd.color.g = float(info.color.g);
            gd.color.b = float(info.color.b);
            gd.data = 0.0f;
        }

        auto corners = grid.corners(datum);
        for (auto &p : corners) {
            enu_corners_.push_back({float(p.enu.x), float(p.enu.y), 0.0f});
            wgs_corners_.push_back({float(p.wgs.lat), float(p.wgs.lon)});
        }
        enu_corners_.push_back(enu_corners_[0]);
        wgs_corners_.push_back(wgs_corners_[0]);

        for (auto &p : info.field) {
            polygon_corners_wgs_.push_back({float(p.wgs.lat), float(p.wgs.lon)});
            polygon_corners_.push_back({float(p.enu.x), float(p.enu.y), 0.0f});
        }
        polygon_corners_wgs_.push_back(polygon_corners_wgs_[0]);
        polygon_corners_.push_back(polygon_corners_[0]);
    }

    void Layer::add_noise() {
        noise.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
        noise.SetSeed(int(rnd()));
        for (std::size_t r = 0; r < grid.rows(); ++r) {
            for (std::size_t c = 0; c < grid.cols(); ++c) {
                grid(r, c).second.data = noise.GetNoise(float(r), float(c)) / 2;
            }
        }
    }

    void Layer::to_image(std::vector<uint8_t> &image) {
        const std::size_t rows = grid.rows();
        const std::size_t cols = grid.cols();
        image.resize(rows * cols * 4);
        for (std::size_t r = 0; r < rows; ++r) {
            for (std::size_t c = 0; c < cols; ++c) {
                auto &[pt, gd] = grid(r, c);
                std::size_t base = (r * cols + c) * 4;
                image[base + 0] = gd.color.r;
                if (has_noise) {
                    image[base + 0] = floatToByte(gd.data);
                }
                image[base + 1] = gd.color.g;
                image[base + 2] = gd.color.b;
                image[base + 3] = gd.color.a;
            }
        }
    }

    std::vector<uint8_t> Layer::to_image() {
        const std::size_t rows = grid.rows();
        const std::size_t cols = grid.cols();
        std::vector<uint8_t> image(rows * cols * 4);
        for (std::size_t r = 0; r < rows; ++r) {
            for (std::size_t c = 0; c < cols; ++c) {
                auto &[pt, gd] = grid(r, c);

                std::size_t base = (r * cols + c) * 4;
                image[base + 0] = gd.color.r;
                image[base + 1] = gd.color.g;
                image[base + 2] = gd.color.b;
                image[base + 3] = gd.color.a;
            }
        }
        return image;
    }

    void Layer::tick(float dt) {
        freq++;
        if (freq % 1000 == 0) {
            visualize();
            freq = 0;
        }
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
