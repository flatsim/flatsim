#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <rerun.hpp>
#include <vector>
#include <zmq.hpp>

#include "flatsim/simulator/machine.hpp"
#include "flatsim/simulator/world.hpp"
#include "flatsim/types.hpp"

// Forward declaration for Agent (avoid circular include)
namespace agent {
    class Agent;
}

namespace simulator {

    enum class Conn {
        LOCAL, // No networking - agents owned by simulator, single process
        IPC,   // Inter-process communication via Unix sockets
        TCP    // Network communication via TCP
    };

    // Simple settings for Simulator constructor (not to be confused with simulator::WorldSettings)
    struct SimulatorSettings {
        float width;
        float height;
        concord::Datum datum;

        // Force users to specify all parameters
        SimulatorSettings(float w, float h, concord::Datum d) : width(w), height(h), datum(d) {}
    };

    class Simulator {
      private:
        // Connection mode
        Conn conn_;

        // ZMQ (only used in IPC/TCP modes)
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> spawn_socket_;                           // REP - for spawn/despawn requests
        std::unique_ptr<zmq::socket_t> heartbeat_socket_;                       // PULL - for heartbeat messages
        std::map<std::string, std::unique_ptr<zmq::socket_t>> control_sockets_; // PULL per-robot
        std::map<std::string, std::unique_ptr<zmq::socket_t>> state_sockets_;   // PUB per-robot
        std::string address_;
        int next_tcp_port_ = 5600;

        // Physics world with obstacle management
        std::unique_ptr<World> world_;
        SimulatorSettings sim_settings_;

        // Machines: uuid -> Machine (physics bodies)
        std::map<std::string, Machine> machines_;

        // Local agents (only used in LOCAL mode)
        std::vector<std::unique_ptr<agent::Agent>> local_agents_;

        // Heartbeat tracking: uuid -> last heartbeat time (IPC/TCP only)
        std::map<std::string, std::chrono::steady_clock::time_point> last_heartbeat_;

        // Next collision group
        uint32_t next_group_ = 1;

        // Rerun visualization
        std::shared_ptr<rerun::RecordingStream> rec_;
        std::string rerun_grpc_addr_ = "rerun+http://127.0.0.1:9876/proxy";
        std::string recording_id_ = "flatsim";
        std::string application_id_ = "flatsim";

        // Private helpers
        void init_rerun();

        // Transport abstraction - handles LOCAL vs IPC/TCP
        void send_state(const std::string &uuid, const types::ser::MachineState &state);
        std::optional<types::WheelControl> recv_control(const std::string &uuid, int timeout_ms);

        // IPC/TCP only - spawn/despawn and connection management
        void process_spawn_requests();
        void process_heartbeats();
        void cleanup_stale_connections();

      public:
        // Constructor for LOCAL mode (no networking)
        // NOTE: datum is REQUIRED - GPS coordinates won't work without it
        Simulator(float width, float height, concord::Datum datum,
                  std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Constructor for IPC/TCP mode with explicit parameters
        Simulator(Conn conn, const std::string &address, float width, float height, concord::Datum datum,
                  std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        // Constructor for IPC/TCP mode with settings struct
        Simulator(Conn conn, const std::string &address, const SimulatorSettings &settings,
                  std::shared_ptr<rerun::RecordingStream> rec = nullptr);

        ~Simulator();

        // Main loop methods
        void tick(float dt);
        void tock();

        // LOCAL mode: Spawn agent directly (returns reference)
        agent::Agent &spawn_agent(const std::filesystem::path &json_path, concord::Pose spawn_pose,
                                  std::optional<pigment::RGB> color = std::nullopt);

        // LOCAL mode: Get agent by uuid
        agent::Agent *get_agent(const std::string &uuid);

        // LOCAL mode: Get all agents
        const std::vector<std::unique_ptr<agent::Agent>> &agents() const { return local_agents_; }

        // Get connection mode
        Conn connection_mode() const { return conn_; }

        // Create machine with wheels, karosseries, etc.
        void create_machine(const types::Machine &machine);

        // Get machine by uuid
        Machine *get_machine(const std::string &uuid);

        // Apply control to a machine
        void apply_control(const types::WheelControl &control, float dt);

        // Destroy a machine
        bool destroy_machine(const std::string &uuid);

        // Get world state for feedback
        types::ser::WorldState get_world_state() const;

        // Access to physics world and world wrapper
        muli::World &get_world() { return world_->physics(); }
        World &world() { return *world_; }

        // Rerun access
        std::shared_ptr<rerun::RecordingStream> rec() const { return rec_; }
    };

} // namespace simulator
