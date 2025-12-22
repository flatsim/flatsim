#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <rerun.hpp>
#include <thread>
#include <vector>
#include <zmq.hpp>

#include "flatsim/simulator/data.hpp"
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

        // Sensor data helper
        Data sensor_data_;

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
        void send_sensor_state(const std::string &uuid, const types::ser::SensorState &state);
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
                                  std::optional<std::string> uuid = std::nullopt,
                                  std::optional<pigment::RGB> color = std::nullopt);

        // LOCAL mode: Get agent by uuid
        agent::Agent *get_agent(const std::string &uuid);

        // LOCAL mode: Get agent by index
        agent::Agent &get_agent(size_t index);

        // LOCAL mode: Get all agents
        const std::vector<std::unique_ptr<agent::Agent>> &agents() const { return local_agents_; }

        // LOCAL mode: Number of agents
        size_t num_agents() const { return local_agents_.size(); }

        // Get connection mode
        Conn connection_mode() const { return conn_; }

        // Convenience loop: combines tick/tock with user callback
        // Runs physics at full speed, visualization at viz_fps
        // user_loop receives dt and returns false to exit
        template <typename UserLoop> void ticktock(UserLoop user_loop, int viz_fps = 30);

        // Create machine with wheels, karosseries, etc.
        void create_machine(const types::Machine &machine);

        // Get machine by uuid
        Machine *get_machine(const std::string &uuid);

        // Apply control to a machine
        void apply_control(const types::WheelControl &control, float dt);

        // Teleport a machine to a new pose
        void teleport_machine(const std::string &uuid, const concord::Pose &pose);

        // Destroy a machine
        bool destroy_machine(const std::string &uuid);

        // Get world state for feedback
        types::ser::WorldState get_world_state() const;

        // Access to physics world and world wrapper
        muli::World &get_world() { return world_->physics(); }
        World &world() { return *world_; }

        // Rerun access
        std::shared_ptr<rerun::RecordingStream> rec() const { return rec_; }

        // Datum access
        concord::Datum get_datum() const { return sim_settings_.datum; }

        // Sensor data access
        const types::SensorData &get_sensor_data(const std::string &uuid) const;

        // LIDAR scan for a specific machine (on-demand)
        types::LidarData scan_lidar(const std::string &uuid, float min_range, float max_range, float fov_deg,
                                    float resolution_deg);

        // Configure LIDAR for a machine (enables automatic LIDAR computation in tick())
        void set_lidar_config(const std::string &uuid, const types::LidarConfig &config);
    };

    // ============================================================================
    // Template Implementation
    // ============================================================================

    template <typename UserLoop> void Simulator::ticktock(UserLoop user_loop, int viz_fps) {
        std::atomic<bool> running{true};
        const auto viz_interval = std::chrono::milliseconds(1000 / viz_fps);

        // Background visualization thread
        std::thread viz_thread([this, &running, viz_interval]() {
            while (running.load()) {
                auto viz_start = std::chrono::steady_clock::now();
                this->tock();

                auto viz_end = std::chrono::steady_clock::now();
                auto elapsed = viz_end - viz_start;
                if (elapsed < viz_interval) {
                    std::this_thread::sleep_for(viz_interval - elapsed);
                }
            }
        });

        // Main physics loop
        auto last_time = std::chrono::steady_clock::now();

        try {
            while (true) {
                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<float> dt_dur = now - last_time;
                float dt = dt_dur.count();
                last_time = now;

                // Physics tick
                this->tick(dt);

                // User callback - return false to exit
                if (!user_loop(dt)) {
                    break;
                }

                // Small sleep to cap CPU usage
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (...) {
            running.store(false);
            if (viz_thread.joinable()) {
                viz_thread.join();
            }
            throw;
        }

        // Clean shutdown
        running.store(false);
        if (viz_thread.joinable()) {
            viz_thread.join();
        }
    }

} // namespace simulator
