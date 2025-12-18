#pragma once

#include "flatsim/types.hpp"
#include "muli/world.h"

namespace simulator {

    // Forward declaration
    class Machine;

    class Hitch {
      private:
        muli::RevoluteJoint *joint_ = nullptr;
        types::Hitch config_;
        Machine *connected_machine_ = nullptr;

      public:
        Hitch() = default;
        Hitch(const types::Hitch &config);

        // Connect this hitch to another machine's hitch (creates revolute joint)
        bool connect(muli::World &world, muli::RigidBody *this_body, Machine *other_machine,
                     const std::string &other_hitch_name);

        // Disconnect
        void disconnect(muli::World &world);

        // Check if connected
        bool is_connected() const { return connected_machine_ != nullptr; }

        // Accessors
        muli::RevoluteJoint *joint() const { return joint_; }
        const types::Hitch &config() const { return config_; }
        Machine *connected_machine() const { return connected_machine_; }
    };

} // namespace simulator
