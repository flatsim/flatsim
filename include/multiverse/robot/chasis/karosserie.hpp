#pragma once

#include "concord/types_basic.hpp"

namespace mvs {
    class Karosserie {
      private:
        concord::Bound box;

      public:
        Karosserie();
        Karosserie(concord::Bound box);

        void tick(float dt);
        void visualize();
    };
} // namespace mvs
