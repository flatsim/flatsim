#pragma once

#include "concord/types_basic.hpp"
#include "concord/types_grid.hpp"
#include "pigment/types_basic.hpp"
#include "rerun.hpp"
#include "spdlog/spdlog.h"

#include <type_traits>
#include <vector>

namespace mvs {
    class Layer {
      public:
        std::string name;
        float inradius;

      private:
        std::shared_ptr<rerun::RecordingStream> rec;
        concord::Datum datum;
        std::vector<uint8_t> image;
        uint freq = 0;

      public:
        Layer() = default;
        Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum);

        void init(std::string name, std::size_t rows, std::size_t cols, double inradius, bool centered = true);
        void tick(float dt);

        concord::Grid<pigment::RGB> &getGrid() { return grid; }

        void to_image(std::vector<uint8_t> &image);
        std::vector<uint8_t> to_image();

        void visualize();

      private:
        concord::Grid<pigment::RGB> grid;
    };
} // namespace mvs
