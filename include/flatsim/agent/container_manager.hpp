#pragma once

#include "flatsim/agent/container/container.hpp"
#include "flatsim/types.hpp"
#include <optional>

namespace fs {

    /**
     * @brief Manager for optional container system
     *
     * Provides null-safe interface to container operations.
     * Automatically handles cases where container doesn't exist.
     */
    class ContainerManager {
      private:
        std::optional<Container> container;

      public:
        ContainerManager() = default;
        ~ContainerManager() = default;

        // Allow copying and moving
        ContainerManager(const ContainerManager &) = default;
        ContainerManager &operator=(const ContainerManager &) = default;
        ContainerManager(ContainerManager &&) = default;
        ContainerManager &operator=(ContainerManager &&) = default;

        /**
         * @brief Initialize container with parameters
         */
        void init(const std::string &name, Container::Type type, float capacity, float current_level, float fill_rate);

        /**
         * @brief Initialize from shared machine config (`types::Container` / legacy `types::Tank`)
         */
        void init(const types::Container &container_info, const pigment::RGB &color, const std::string &parent_name) {
            Container::Type type = (container_info.type == types::ContainerType::WASTE) ? Container::Type::WASTE
                                                                                        : Container::Type::HARVEST;

            // fill/empty rates are currently not part of shared config; keep harmless defaults
            container.emplace(container_info.name, type, container_info.capacity, /*fill_rate=*/0.0f,
                              /*empty_rate=*/0.0f);
            container->init(color, parent_name, container_info.bound);
        }

        /**
         * @brief Check if container exists
         */
        bool exists() const { return container.has_value(); }

        /**
         * @brief Get direct access to container (use with caution)
         * @return Optional reference to container
         */
        std::optional<Container> &get_optional() { return container; }
        const std::optional<Container> &get_optional() const { return container; }

        /**
         * @brief Fill container with amount
         */
        void fill(float amount) {
            if (container) container->fill(amount);
        }

        /**
         * @brief Empty entire container
         */
        void empty_all() {
            if (container) container->empty_all();
        }

        /**
         * @brief Empty specific amount from container
         */
        void empty(float amount) {
            if (container) container->empty(amount);
        }

        /**
         * @brief Get current container amount
         */
        float get_current_amount() const { return container ? container->get_current_amount() : 0.0f; }

        /**
         * @brief Get container capacity
         */
        float get_capacity() const { return container ? container->get_capacity() : 0.0f; }

        /**
         * @brief Get container fill percentage
         */
        float get_percentage() const { return container ? container->get_percentage() : 0.0f; }

        /**
         * @brief Check if container is empty
         */
        bool is_empty() const { return container ? container->is_empty() : true; }

        /**
         * @brief Check if container is full
         */
        bool is_full() const { return container ? container->is_full() : false; }

        /**
         * @brief Update container (for visualization)
         */
        void tick(double dt, const datapod::Pose &pose) {
            if (container) container->tick(dt, pose);
        }

        /**
         * @brief Visualize container
         */
        void tock(std::shared_ptr<rerun::RecordingStream> rec) {
            if (container) container->tock(rec);
        }
    };

} // namespace fs
