#pragma once

#include "concord/types_basic.hpp"
#include "concord/types_grid.hpp"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"

#include <type_traits>
#include <vector>

namespace mvs {
    class Layer {
      public:
        std::string name;

      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        std::vector<uint8_t> image;

      public:
        Layer() = default;
        Layer(std::shared_ptr<rerun::RecordingStream> rec, std::string name, std::size_t rows, std::size_t cols, double inradius,
              concord::Datum datum = concord::Datum(), bool centered = true);

        concord::Grid<pigment::RGB> &getGrid() { return grid; }

        void to_image(std::vector<uint8_t> &image);
        std::vector<uint8_t> to_image();

        void visualize();

      private:
        concord::Grid<pigment::RGB> grid;
    };
} // namespace mvs
