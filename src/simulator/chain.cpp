#include "flatsim/simulator/chain.hpp"

#include "flatsim/simulator/machine.hpp"
#include <algorithm>

namespace simulator {

    static float calculate_hitch_overlap_percentage(const Hitch &hitch1, const Hitch &hitch2) {
        auto corners1 = hitch1.get_corners();
        auto corners2 = hitch2.get_corners();

        float min_x1 = static_cast<float>(corners1[0].x), max_x1 = static_cast<float>(corners1[0].x);
        float min_y1 = static_cast<float>(corners1[0].y), max_y1 = static_cast<float>(corners1[0].y);
        float min_x2 = static_cast<float>(corners2[0].x), max_x2 = static_cast<float>(corners2[0].x);
        float min_y2 = static_cast<float>(corners2[0].y), max_y2 = static_cast<float>(corners2[0].y);

        for (const auto &corner : corners1) {
            min_x1 = std::min(min_x1, static_cast<float>(corner.x));
            max_x1 = std::max(max_x1, static_cast<float>(corner.x));
            min_y1 = std::min(min_y1, static_cast<float>(corner.y));
            max_y1 = std::max(max_y1, static_cast<float>(corner.y));
        }

        for (const auto &corner : corners2) {
            min_x2 = std::min(min_x2, static_cast<float>(corner.x));
            max_x2 = std::max(max_x2, static_cast<float>(corner.x));
            min_y2 = std::min(min_y2, static_cast<float>(corner.y));
            max_y2 = std::max(max_y2, static_cast<float>(corner.y));
        }

        float intersection_x1 = std::max(min_x1, min_x2);
        float intersection_y1 = std::max(min_y1, min_y2);
        float intersection_x2 = std::min(max_x1, max_x2);
        float intersection_y2 = std::min(max_y1, max_y2);

        if (intersection_x1 >= intersection_x2 || intersection_y1 >= intersection_y2) return 0.0f;

        float intersection_area = (intersection_x2 - intersection_x1) * (intersection_y2 - intersection_y1);
        float area1 = (max_x1 - min_x1) * (max_y1 - min_y1);
        float area2 = (max_x2 - min_x2) * (max_y2 - min_y2);
        float smaller_area = std::min(area1, area2);
        if (smaller_area <= 0.0f) return 0.0f;

        return (intersection_area / smaller_area) * 100.0f;
    }

    bool Chain::connect(Machine &master, const std::string &master_hitch_name, Machine &follower,
                        const std::string &follower_hitch_name, float min_overlap_percent) {
        auto *master_hitch = master.find_hitch(master_hitch_name);
        auto *follower_hitch = follower.find_hitch(follower_hitch_name);
        if (!master_hitch || !follower_hitch) return false;

        // Only connect master hitch -> follower hitch.
        if (!master_hitch->is_master || follower_hitch->is_master) return false;

        float overlap_percentage = calculate_hitch_overlap_percentage(*master_hitch, *follower_hitch);
        if (overlap_percentage < min_overlap_percent) return false;

        // TODO: create physics joint(s) between bodies; for now just toggle hooked state.
        master_hitch->hooked = true;
        follower_hitch->hooked = true;

        // Avoid duplicates.
        links_.erase(std::remove_if(links_.begin(), links_.end(),
                                    [&](const ChainLink &l) {
                                        return l.master_uuid == master.uuid() && l.follower_uuid == follower.uuid();
                                    }),
                     links_.end());

        links_.push_back({.master_uuid = master.uuid(),
                          .follower_uuid = follower.uuid(),
                          .master_hitch = master_hitch_name,
                          .follower_hitch = follower_hitch_name});
        return true;
    }

    bool Chain::disconnect(Machine &master, Machine &follower) {
        auto it = std::find_if(links_.begin(), links_.end(), [&](const ChainLink &l) {
            return l.master_uuid == master.uuid() && l.follower_uuid == follower.uuid();
        });
        if (it == links_.end()) return false;

        if (auto *mh = master.find_hitch(it->master_hitch)) mh->hooked = false;
        if (auto *fh = follower.find_hitch(it->follower_hitch)) fh->hooked = false;

        links_.erase(it);
        return true;
    }

    void Chain::disconnect_all(Machine &master) {
        // TODO: if/when this is used, pass access to other machines so we can also unhook followers.
        for (auto &link : links_) {
            if (link.master_uuid != master.uuid()) continue;
            if (auto *mh = master.find_hitch(link.master_hitch)) mh->hooked = false;
        }

        links_.erase(std::remove_if(links_.begin(), links_.end(),
                                    [&](const ChainLink &l) { return l.master_uuid == master.uuid(); }),
                     links_.end());
    }

} // namespace simulator
