#include "multiverse/world/layer.hpp"

namespace mvs {
    template class Layer<pigment::RGB>;
    template <typename T>
    Layer<T>::Layer(std::shared_ptr<rerun::RecordingStream> rec, std::size_t rows, std::size_t cols, double inradius,
                    bool centered)
        : rec(rec) {
        grid = concord::Grid<T>(rows, cols, inradius, centered);
    }

    template <typename T>
    Layer<T>::Layer(std::shared_ptr<rerun::RecordingStream> rec, std::size_t rows, std::size_t cols, double inradius,
                    concord::Datum datum, bool centered)
        : rec(rec) {
        grid = concord::Grid<T>(rows, cols, inradius, datum, centered);
    }

    template <typename T>
    Layer<T>::Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Size world_size, double inradius) : rec(rec) {
        auto width = world_size.x / inradius;
        auto height = world_size.y / inradius;
        grid = concord::Grid<T>(width, height, inradius);
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

    template <typename T> void Layer<T>::visualize() {
        auto gs = float(grid.inradius() / 2);
        rec->log_static("grid", rerun::Boxes3D::from_centers_and_half_sizes(grid.flatten_points(), {{gs, gs, 0.0f}})
                                    .with_colors(rerun::Color(110, 90, 60))
                                    .with_radii({{0.005f}}));
    }

} // namespace mvs
