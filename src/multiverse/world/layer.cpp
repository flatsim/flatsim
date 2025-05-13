#include "multiverse/world/layer.hpp"

namespace mvs {

    inline uint8_t floatToByte(float v) {
        v = std::clamp(v, 0.0f, 1.0f);
        float scaled = v * 255.0f;
        return static_cast<uint8_t>(std::round(scaled));
    }

    Layer::Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum)
        : rec(rec), datum(datum), rnd(std::random_device()()) {}

    void Layer::init(std::string name, std::size_t rows, std::size_t cols, double inradius, bool centered) {
        this->name = name;
        this->inradius = inradius;
        this->rows = rows;
        this->cols = cols;

        grid = concord::Grid<GridData>(rows, cols, inradius, datum, centered);
        image.resize(grid.rows() * grid.cols() * 4, 0);
        for (auto &[p, gd] : grid) {
            gd.color.r = 70;
            gd.color.g = 70;
            gd.color.b = 50;
            gd.color.a = 255;
        }
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
                // image[base + 0] = gd.color.r;
                image[base + 0] = floatToByte(gd.data);
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
        to_image(image);
        rec->log_static(name, rerun::Image::from_rgba32(image, {uint32_t(grid.cols()), uint32_t(grid.rows())}));

        float g_w = float(grid.cols()) * inradius;
        float g_h = float(grid.rows()) * inradius;
        rerun::components::ImageBuffer buf(image);
        const rerun::Position3D vertex_positions[4] = {{-g_w / 2, -g_h / 2, -0.1f},
                                                       {g_w / 2, -g_h / 2, -0.1f},
                                                       {g_w / 2, g_h / 2, -0.1f},
                                                       {-g_w / 2, g_h / 2, -0.1f}};
        // rec->log_static(this->name + "/texture", rerun::Mesh3D(vertex_positions)
        //                                              .with_vertex_normals({{0.0, 0.0, 0.0}})
        //                                              .with_albedo_texture_buffer(buf)
        //                                              .with_triangle_indices({{0, 1, 2}, {0, 2, 3}}));

        // rec->log_static("grid", rerun::Boxes3D::from_centers_and_sizes(
        //                             grid.flatten_points(), {{float(grid.inradius()), float(grid.inradius()), 0.0f}})
        //                             .with_colors(rerun::Color(110, 90, 60))
        //                             .with_radii({{0.005f}}));
    }

} // namespace mvs
