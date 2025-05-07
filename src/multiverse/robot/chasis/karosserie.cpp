#include "multiverse/robot/karosserie.hpp"

namespace mvs {
    Karosserie::Karosserie() {}
    Karosserie::Karosserie(concord::Bound box) : box(box) {}

    void Karosserie::tick(float dt) {}
    void Karosserie::visualize() {}
} // namespace mvs
