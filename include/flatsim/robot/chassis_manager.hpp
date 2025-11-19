#pragma once

#include "flatsim/robot/chassis/chassis.hpp"
#include "flatsim/robot/types.hpp"
#include <memory>
#include <vector>

namespace fs {

    // Forward declaration
    class Robot;

    /**
     * @brief Manager for robot chassis and its components
     *
     * Provides unified interface for chassis management including:
     * - Karosserie/section operations
     * - Wheel damping control
     * - Teleportation
     * - Color updates
     * - Physics body access
     */
    class ChassisManager {
      private:
        Robot *robot = nullptr;
        std::unique_ptr<Chassis> chassis;
        std::shared_ptr<rerun::RecordingStream> rec;
        std::shared_ptr<muli::World> world;
        muli::CollisionFilter filter;

      public:
        ChassisManager() = default;
        ~ChassisManager() = default;

        // Prevent copying, allow moving
        ChassisManager(const ChassisManager &) = delete;
        ChassisManager &operator=(const ChassisManager &) = delete;
        ChassisManager(ChassisManager &&) = default;
        ChassisManager &operator=(ChassisManager &&) = default;

        /**
         * @brief Initialize the chassis manager
         * @param r Pointer to parent robot
         * @param rec Rerun recording stream
         * @param world Physics world
         * @param filter Collision filter
         * @param robo Robot configuration info
         */
        void init(Robot *r, std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world,
                  muli::CollisionFilter filter, RobotInfo &robo);

        /**
         * @brief Physics tick update
         * @param dt Time delta in seconds
         */
        void tick(float dt);

        /**
         * @brief Visualization tick update
         * @param label Label for visualization
         */
        void tock(const std::string &label);

        /**
         * @brief Update wheel steering and throttle
         * @param steering Steering values per wheel
         * @param throttle Throttle values per wheel
         * @param dt Time delta in seconds
         */
        void update(const std::vector<float> &steering, const std::vector<float> &throttle, float dt);

        // Karosserie/Section operations
        /**
         * @brief Toggle work state for a specific section
         * @param karosserie_name Name of the karosserie
         * @param section_id Section ID to toggle
         */
        void toggle_section_work(const std::string &karosserie_name, int section_id);

        /**
         * @brief Toggle work state for all sections
         * @param karosserie_name Name of the karosserie
         */
        void toggle_all_sections_work(const std::string &karosserie_name);

        /**
         * @brief Toggle work state for all sections except one
         * @param karosserie_name Name of the karosserie
         * @param except_section_id Section ID to exclude
         */
        void toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id);

        // Wheel operations
        /**
         * @brief Set wheel damping parameters
         * @param linear_damping Linear damping coefficient
         * @param angular_damping Angular damping coefficient
         */
        void set_wheel_damping(float linear_damping, float angular_damping);

        // Transform operations
        /**
         * @brief Teleport chassis to a new pose
         * @param pose Target pose
         */
        void teleport(concord::Pose pose);

        /**
         * @brief Update chassis and component colors
         * @param new_color New color to apply
         */
        void update_color(const pigment::RGB &new_color);

        // Accessors
        /**
         * @brief Get current chassis pose
         * @return Current pose
         */
        concord::Pose get_pose() const;

        /**
         * @brief Get physics transform
         * @return Physics transform
         */
        muli::Transform get_transform() const;

        /**
         * @brief Get physics body
         * @return Pointer to rigid body
         */
        muli::RigidBody *get_body();

        /**
         * @brief Get karosseries
         * @return Pointer to karosseries vector
         */
        std::vector<Karosserie> *get_karosseries();

        /**
         * @brief Get hitches
         * @return Pointer to hitches vector
         */
        std::vector<Hitch> *get_hitches();

        /**
         * @brief Check if chassis exists
         * @return true if chassis is initialized
         */
        bool exists() const { return chassis != nullptr; }

        /**
         * @brief Get chassis bound
         * @return Chassis bound
         */
        const concord::Bound &get_bound() const;
    };

} // namespace fs
