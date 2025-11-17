#pragma once

#include "flatsim/robot/tank/tank.hpp"
#include <optional>

namespace fs {

    /**
     * @brief Manager for optional tank system
     *
     * Provides null-safe interface to tank operations.
     * Automatically handles cases where tank doesn't exist.
     */
    class TankManager {
      private:
        std::optional<Tank> tank;

      public:
        TankManager() = default;
        ~TankManager() = default;

        // Allow copying and moving
        TankManager(const TankManager &) = default;
        TankManager &operator=(const TankManager &) = default;
        TankManager(TankManager &&) = default;
        TankManager &operator=(TankManager &&) = default;

        /**
         * @brief Initialize tank with parameters
         */
        void init(const std::string &name, Tank::Type type, float capacity, float current_level, float fill_rate);

        /**
         * @brief Initialize from TankInfo
         */
        void init(const TankInfo &tank_info, const pigment::RGB &color, const std::string &seqid);

        /**
         * @brief Check if tank exists
         */
        bool exists() const { return tank.has_value(); }

        /**
         * @brief Get direct access to tank (use with caution)
         * @return Optional reference to tank
         */
        std::optional<Tank> &get_optional() { return tank; }
        const std::optional<Tank> &get_optional() const { return tank; }

        /**
         * @brief Fill tank with amount
         */
        void fill(float amount) {
            if (tank) tank->fill(amount);
        }

        /**
         * @brief Empty entire tank
         */
        void empty_all() {
            if (tank) tank->empty_all();
        }

        /**
         * @brief Empty specific amount from tank
         */
        void empty(float amount) {
            if (tank) tank->empty(amount);
        }

        /**
         * @brief Get current tank amount
         */
        float get_current_amount() const { return tank ? tank->get_current_amount() : 0.0f; }

        /**
         * @brief Get tank capacity
         */
        float get_capacity() const { return tank ? tank->get_capacity() : 0.0f; }

        /**
         * @brief Get tank fill percentage
         */
        float get_percentage() const { return tank ? tank->get_percentage() : 0.0f; }

        /**
         * @brief Check if tank is empty
         */
        bool is_empty() const { return tank ? tank->is_empty() : true; }

        /**
         * @brief Check if tank is full
         */
        bool is_full() const { return tank ? tank->is_full() : false; }

        /**
         * @brief Update tank (for visualization)
         */
        void tick(double dt, const concord::Pose &pose) {
            if (tank) tank->tick(dt, pose);
        }

        /**
         * @brief Visualize tank
         */
        void tock(std::shared_ptr<rerun::RecordingStream> rec) {
            if (tank) tank->tock(rec);
        }
    };

} // namespace fs
