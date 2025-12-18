#pragma once

#include <memory>
#include <rerun.hpp>

#include "flatsim/types.hpp"

namespace agent {

    // Forward declaration
    class Machine;

    class Hitch {
      private:
        types::Hitch config_;
        types::Machine machine_config_; // Parent machine config
        Machine *connected_machine_ = nullptr;
        std::shared_ptr<rerun::RecordingStream> rec_;

      public:
        Hitch() = default;
        Hitch(const types::Hitch &config, const types::Machine &machine_config,
              std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Track connection (no physics, just state)
        void set_connected(Machine *machine) { connected_machine_ = machine; }
        void disconnect() { connected_machine_ = nullptr; }

        // Check if connected
        bool is_connected() const { return connected_machine_ != nullptr; }

        // Set rerun for visualization
        void set_rerun(std::shared_ptr<rerun::RecordingStream> rec) { rec_ = rec; }

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Accessors
        const types::Hitch &config() const { return config_; }
        Machine *connected_machine() const { return connected_machine_; }
    };

} // namespace agent
