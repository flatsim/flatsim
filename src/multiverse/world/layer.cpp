#include "multiverse/world/layer.hpp"

namespace mvs {
    template class Layer<pigment::RGB>;
    template <typename T> Layer<T>::Layer(std::size_t rows, std::size_t cols, double inradius, bool centered) {
        grid = concord::Grid<T>(rows, cols, inradius, centered);
    }

    template <typename T>
    Layer<T>::Layer(std::size_t rows, std::size_t cols, double inradius, concord::Datum datum, bool centered) {
        grid = concord::Grid<T>(rows, cols, inradius, datum, centered);
    }

    template <typename T> template <typename U, typename> void Layer<T>::to_image(std::vector<uint8_t> &image) {
        for (std::size_t r = 0; r < grid.rows(); ++r) {
            for (std::size_t c = 0; c < grid.cols(); ++c) {
                auto &[pt, color] = grid(r, c); // now color is an RGB& directly
                image[r * grid.cols() * 3 + c * 3 + 0] = color.r;
                image[r * grid.cols() * 3 + c * 3 + 1] = color.g;
                image[r * grid.cols() * 3 + c * 3 + 2] = color.b;
            }
        }
    }

    template <typename T>
    template <typename U, typename>
    void Layer<T>::visualize(std::shared_ptr<rerun::RecordingStream> rec, concord::Size world_size,
                             concord::Size grid_size) {
        std::vector<std::array<float, 3>> enu_grid_;
        for (std::size_t r = 0; r < grid.getGrid().rows(); ++r) {
            for (std::size_t c = 0; c < grid.getGrid().cols(); ++c) {
                enu_grid_.push_back(
                    {(float(grid.getGrid()(r, c).first.enu.x)), (float(grid.getGrid()(r, c).first.enu.y)), 0});
            }
        }

        auto gsy = float(grid_size.y / 2);
        auto gsx = float(world_size.x / 2);
        rec->log_static("grid", rerun::Boxes3D::from_centers_and_half_sizes(enu_grid_, {{gsx, gsy, 0.0f}})
                                    .with_colors(rerun::Color(110, 90, 60))
                                    .with_radii({{0.005f}}));
    }

} // namespace mvs
