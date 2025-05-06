#include "multiverse/world/layer.hpp"

namespace mvs {
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

    template class Layer<pigment::RGB>;
} // namespace mvs
