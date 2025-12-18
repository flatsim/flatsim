#pragma once

#include "flatsim/types.hpp"

namespace agent {

    // Forward declaration
    class Machine;

    class Hitch {
      private:
        types::Hitch config_;
        Machine *connected_machine_ = nullptr;

      public:
        Hitch() = default;
        Hitch(const types::Hitch &config);

        // Track connection (no physics, just state)
        void set_connected(Machine *machine) { connected_machine_ = machine; }
        void disconnect() { connected_machine_ = nullptr; }

        // Check if connected
        bool is_connected() const { return connected_machine_ != nullptr; }

        // Tick/tock pattern
        void tick(float dt);
        void tock();

        // Accessors
        const types::Hitch &config() const { return config_; }
        Machine *connected_machine() const { return connected_machine_; }
    };

} // namespace agent
