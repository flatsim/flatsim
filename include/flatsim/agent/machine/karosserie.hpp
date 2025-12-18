#pragma once

#include <memory>
#include <rerun.hpp>

#include "flatsim/types.hpp"

namespace agent {

    class Karosserie {
      private:
        types::Karosserie config_;
        types::Machine machine_config_; // Parent machine config
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        Karosserie() = default;
        Karosserie(const types::Karosserie &config, const types::Machine &machine_config,
                   std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Set rerun for visualization
        void set_rerun(std::shared_ptr<rerun::RecordingStream> rec) { rec_ = rec; }

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Accessors
        const types::Karosserie &config() const { return config_; }
    };

} // namespace agent
