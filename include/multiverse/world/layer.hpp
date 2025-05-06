#pragma once

#include "concord/types_basic.hpp"
#include "concord/types_grid.hpp"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

#include <type_traits>
#include <vector>

namespace mvs {
    template <typename T> class Layer {
        friend class concord::Grid<T>;

      private:
        std::vector<std::array<float, 3>> enu_grid_;

      public:
        Layer() = default;
        Layer(std::size_t rows, std::size_t cols, double inradius, bool centered = true);
        Layer(std::size_t rows, std::size_t cols, double inradius, concord::Datum datum, bool centered = true);

        concord::Grid<T> &getGrid() { return grid; }
        const concord::Grid<T> &getGrid() const { return grid; }

        template <typename U = T, typename = std::enable_if_t<std::is_same_v<U, pigment::RGB>>>
        void to_image(std::vector<uint8_t> &image);

        template <typename U = T, typename = std::enable_if_t<std::is_same_v<U, pigment::RGB>>>
        void visualize(std::shared_ptr<rerun::RecordingStream> rec, concord::Size world_size, concord::Size grid_size);

      private:
        concord::Grid<T> grid;
    };
} // namespace mvs
