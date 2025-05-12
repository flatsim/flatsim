#include "multiverse/world/layer.hpp"

namespace mvs {

    Layer::Layer(std::shared_ptr<rerun::RecordingStream> rec, std::string name, std::size_t rows, std::size_t cols,
                 double inradius, concord::Datum datum, bool centered)
        : rec(rec), name(name), inradius(inradius) {
        grid = concord::Grid<pigment::RGB>(rows, cols, inradius, datum, centered);
        image.resize(grid.rows() * grid.cols() * 3, 0);
        for (auto &[p, c] : grid) {
            c.r = 255;
            c.g = 255;
            c.b = 255;
            c.a = 255;
        }
    }

    void Layer::to_image(std::vector<uint8_t> &image) {
        const std::size_t rows = grid.rows();
        const std::size_t cols = grid.cols();
        image.resize(rows * cols * 4);
        for (std::size_t r = 0; r < rows; ++r) {
            for (std::size_t c = 0; c < cols; ++c) {
                auto &[pt, color] = grid(r, c);
                std::size_t base = (r * cols + c) * 4;
                image[base + 0] = color.r;
                image[base + 1] = color.g;
                image[base + 2] = color.b;
                image[base + 3] = color.a;
            }
        }
    }

    std::vector<uint8_t> Layer::to_image() {
        const std::size_t rows = grid.rows();
        const std::size_t cols = grid.cols();
        std::vector<uint8_t> image(rows * cols * 4);
        for (std::size_t r = 0; r < rows; ++r) {
            for (std::size_t c = 0; c < cols; ++c) {
                auto &[pt, color] = grid(r, c);

                std::size_t base = (r * cols + c) * 4;
                image[base + 0] = color.r;
                image[base + 1] = color.g;
                image[base + 2] = color.b;
                image[base + 3] = color.a;
            }
        }
        return image;
    }

    void Layer::visualize() {
        to_image(image);
        rec->log_static(name, rerun::Image::from_rgba32(image, {uint32_t(grid.cols()), uint32_t(grid.rows())}));

        float g_w = float(grid.cols()) * inradius;
        float g_h = float(grid.rows()) * inradius;

        // rerun::components::ImageBuffer buf(image);
        // const rerun::Position3D vertex_positions[4] = {
        //     {-g_w / 2, -g_h / 2, 0.0f}, {g_w / 2, -g_h / 2, 0.0f}, {g_w / 2, g_h / 2, 0.0f}, {-g_w / 2, g_h / 2,
        //     0.0f}};
        // rec->log_static(this->name + "/texture", rerun::Mesh3D(vertex_positions)
        //                                              .with_vertex_normals({{0.0, 0.0, 1.0}})
        //                                              .with_albedo_texture_buffer(buf)
        //                                              .with_triangle_indices({{0, 1, 2}, {0, 2, 3}}));
    }

} // namespace mvs
