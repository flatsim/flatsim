#include "multiverse/world/layer.hpp"

namespace mvs {

    Layer::Layer(std::shared_ptr<rerun::RecordingStream> rec, std::string name, std::size_t rows, std::size_t cols,
                 double inradius, concord::Datum datum, bool centered)
        : rec(rec), name(name) {
        grid = concord::Grid<pigment::RGB>(rows, cols, inradius, datum, centered);
        image.resize(grid.rows() * grid.cols() * 3);
    }

    void Layer::to_image(std::vector<uint8_t> &image) {
        for (std::size_t r = 0; r < grid.rows(); ++r) {
            for (std::size_t c = 0; c < grid.cols(); ++c) {
                auto &[pt, color] = grid(r, c); // now color is an RGB& directly
                image[r * grid.cols() * 3 + c * 3 + 0] = color.r;
                image[r * grid.cols() * 3 + c * 3 + 1] = color.g;
                image[r * grid.cols() * 3 + c * 3 + 2] = color.b;
            }
        }
    }

    std::vector<uint8_t> Layer::to_image() {
        std::vector<uint8_t> image(grid.rows() * grid.cols() * 3);
        for (std::size_t r = 0; r < grid.rows(); ++r) {
            for (std::size_t c = 0; c < grid.cols(); ++c) {
                auto &[pt, color] = grid(r, c); // now color is an RGB& directly
                image[r * grid.cols() * 3 + c * 3 + 0] = color.r;
                image[r * grid.cols() * 3 + c * 3 + 1] = color.g;
                image[r * grid.cols() * 3 + c * 3 + 2] = color.b;
            }
        }
        return image;
    }

    void Layer::visualize() {
        to_image(image);
        rec->log_static(name, rerun::Image::from_rgb24(image, {uint32_t(grid.cols()), uint32_t(grid.rows())}));
        // auto gs = float(grid.inradius() / 2);
        // rec->log_static("grid", rerun::Boxes3D::from_centers_and_half_sizes(grid.flatten_points(), {{gs, gs, 0.0f}})
        //                             .with_colors(rerun::Color(110, 90, 60))
        //                             .with_radii({{0.005f}}));
    }

} // namespace mvs
