#include "flatsim/agent/container_manager.hpp"

namespace fs {

    void ContainerManager::init(const std::string &name, Container::Type type, float capacity, float current_level,
                                float fill_rate) {
        container.emplace(name, type, capacity, fill_rate, /*empty_rate=*/0.0f);
        container->fill(current_level);
    }

} // namespace fs
