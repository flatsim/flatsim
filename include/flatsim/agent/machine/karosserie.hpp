#pragma once

#include "flatsim/types.hpp"

namespace agent {

    class Karosserie {
      private:
        types::Karosserie config_;

      public:
        Karosserie() = default;
        Karosserie(const types::Karosserie &config);

        // Accessors
        const types::Karosserie &config() const { return config_; }
    };

} // namespace agent
