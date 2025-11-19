#pragma once

#include "flatsim/robot/power/power.hpp"
#include "flatsim/robot/types.hpp"
#include <memory>
#include <optional>

namespace fs {

    /**
     * @brief Manager for optional power system
     *
     * Provides null-safe interface to power operations.
     * Automatically handles cases where power system doesn't exist.
     */
    class PowerManager {
      private:
        std::optional<std::unique_ptr<Power>> power;

      public:
        PowerManager() = default;
        ~PowerManager() = default;

        // Prevent copying, allow moving
        PowerManager(const PowerManager &) = delete;
        PowerManager &operator=(const PowerManager &) = delete;
        PowerManager(PowerManager &&) = default;
        PowerManager &operator=(PowerManager &&) = default;

        /**
         * @brief Initialize power system with parameters
         */
        void init(const std::string &name, Power::Type type, float capacity, float consumption_rate, float charge_rate);

        /**
         * @brief Initialize from PowerInfo
         */
        void init(const PowerInfo &power_info);

        /**
         * @brief Check if power system exists
         */
        bool exists() const { return power.has_value() && power->get() != nullptr; }

        /**
         * @brief Get raw pointer to power (for direct access)
         */
        Power *get() const { return (power && *power) ? power->get() : nullptr; }

        /**
         * @brief Check if power system is available and not empty
         */
        bool is_powered() const { return exists() && !(*power)->is_empty(); }

        /**
         * @brief Get power percentage
         */
        float get_percentage() const { return exists() ? (*power)->get_percentage() : 0.0f; }

        /**
         * @brief Get current power amount
         */
        float get_current_amount() const { return exists() ? (*power)->get_current_amount() : 0.0f; }

        /**
         * @brief Get power capacity
         */
        float get_capacity() const { return exists() ? (*power)->get_capacity() : 0.0f; }

        /**
         * @brief Check if power is empty
         */
        bool is_empty() const { return !exists() || (*power)->is_empty(); }

        /**
         * @brief Check if power is full
         */
        bool is_full() const { return exists() && (*power)->is_full(); }

        /**
         * @brief Refuel/recharge power
         */
        void refuel(float amount) {
            if (exists()) (*power)->refuel(amount);
        }

        /**
         * @brief Charge power
         */
        void charge(float dt) {
            if (exists()) (*power)->charge(dt);
        }

        /**
         * @brief Update power system
         */
        void update(float dt, float consumption_multiplier = 1.0f) {
            if (exists()) (*power)->update(dt, consumption_multiplier);
        }

        /**
         * @brief Fill power to maximum
         */
        void refuel_full() {
            if (exists()) (*power)->refuel_full();
        }

        /**
         * @brief Check if power is low (< 15%)
         */
        bool is_low() const { return exists() && (*power)->is_low(); }
    };

} // namespace fs
