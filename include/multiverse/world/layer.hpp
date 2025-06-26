#pragma once

#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"

#include <any>
#include <random>
#include <type_traits>
#include <vector>

namespace mvs {

    struct GridData {
        pigment::RGB color;
        float data;
    };

    class Layer {
      public:
        LayerInfo info;

      private:
        concord::Grid<GridData> grid;
        std::shared_ptr<rerun::RecordingStream> rec;
        entropy::NoiseGen noise;
        concord::Datum datum;
        std::vector<uint8_t> image;
        std::vector<uint8_t> data_img;
        size_t rows, cols;
        uint freq = 0;
        bool has_noise = false;

        std::mt19937 rnd;
        std::vector<std::array<float, 3>> enu_corners_;
        std::vector<std::array<float, 3>> polygon_corners_;
        std::vector<rerun::LatLon> wgs_corners_;
        std::vector<rerun::LatLon> polygon_corners_wgs_;

      public:
        Layer() = default;
        Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum);

        void init(LayerInfo info);
        void tick(float dt);
        void add_noise(bool in_polygon_only = false);
        void paint(pigment::RGB color, concord::Polygon brush);
        void color_field();
        void to_image(std::vector<uint8_t> &image);
        void visualize();
        concord::Grid<uint8_t> get_grid_data() const;

        concord::Point at(uint x, uint y) const { return grid.at(x, y).first; }
        GridData data_at(uint x, uint y) const { return grid.at(x, y).second; }
        concord::Grid<GridData> &getGrid() { return grid; }
        const concord::Grid<GridData> &getGrid() const { return grid; }
    };
} // namespace mvs
