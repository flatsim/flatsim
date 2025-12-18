#pragma once

#include <map>
#include <memory>
#include <vector>
#include <zmq.hpp>

#include "flatsim/types.hpp"
#include "muli/world.h"

namespace simulator {

    enum class Conn { TCP, IPC };

    struct WorldSettings {
        float width = 100.0f;
        float height = 100.0f;
    };

    // Internal wheel physics state
    struct WheelPhysics {
        muli::RigidBody *body = nullptr;
        muli::MotorJoint *motor_joint = nullptr;
        muli::AngleJoint *angle_joint = nullptr;
        types::Wheel config;
        // Rate-limited current values
        float current_steering = 0.0f;
        float current_throttle = 0.0f;
        float steering_rate = 0.52f; // rad/s
        float throttle_rate = 2.5f;  // 1/s
    };

    // Internal machine physics state
    struct MachinePhysics {
        muli::RigidBody *body = nullptr;
        std::vector<WheelPhysics> wheels;
        types::Machine config;
        muli::CollisionFilter filter;
    };

    class Simulator {
      private:
        // ZMQ
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        Conn conn_;
        std::string address_;

        // Physics
        std::unique_ptr<muli::World> world_;
        WorldSettings world_settings_;

        // Machines: uuid -> physics
        std::map<std::string, MachinePhysics> machines_;

        // Next collision group
        uint32_t next_group_ = 1;

        // Helper to compute shifted pose
        static concord::Pose shift_pose(const concord::Pose &parent, const concord::Pose &child);

      public:
        Simulator(Conn conn, const std::string &address = "", const WorldSettings &settings = {});
        ~Simulator();

        void tick(float dt);
        void tock();

        // Create machine with wheels, karosseries, etc.
        void create_machine(const types::Machine &machine);

        // Apply control to a machine
        void apply_control(const types::MachineControl &control, float dt);

        // Destroy a machine
        bool destroy_machine(const std::string &uuid);

        // Get world state for feedback
        types::ser::WorldState get_world_state() const;

        muli::World &get_world() { return *world_; }
    };

} // namespace simulator
