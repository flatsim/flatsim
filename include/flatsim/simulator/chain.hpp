#pragma once

#include <string>
#include <vector>

namespace simulator {

    class Machine;

    struct ChainLink {
        std::string master_uuid;
        std::string follower_uuid;
        std::string master_hitch;
        std::string follower_hitch;
    };

    class Chain {
      public:
        // Connect a follower to a master (physics joint creation is TODO; for now we only toggle hitch flags).
        bool connect(Machine &master, const std::string &master_hitch, Machine &follower,
                     const std::string &follower_hitch, float min_overlap_percent = 50.0f);

        // Disconnect a follower from a master (no-op if not connected).
        bool disconnect(Machine &master, Machine &follower);

        // Disconnect all followers from this master.
        void disconnect_all(Machine &master);

        const std::vector<ChainLink> &links() const { return links_; }

      private:
        std::vector<ChainLink> links_;
    };

} // namespace simulator
