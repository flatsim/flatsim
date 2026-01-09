// Test netpipe RemotePeer for bidirectional RPC on a single channel
// This demonstrates TRUE bidirectional communication where both sides
// can register handlers AND make calls simultaneously

#include <atomic>
#include <chrono>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <thread>

// Message types
enum class MsgType : uint32_t {
    // Agent -> Simulator
    SPAWN = 1,
    DESPAWN = 2,
    HEARTBEAT = 3,
    CONTROL = 4,
    LIDAR_CFG = 5,

    // Simulator -> Agent (bidirectional!)
    STATE = 10,
    SENSORS = 11,
    TELEPORT = 12,
};

// Simple message structures
struct SpawnRequest {
    std::string agent_name;

    netpipe::Message serialize() const {
        netpipe::Message msg;
        msg.insert(msg.end(), agent_name.begin(), agent_name.end());
        return msg;
    }

    static SpawnRequest deserialize(const netpipe::Message &msg) { return {std::string(msg.begin(), msg.end())}; }
};

struct SpawnResponse {
    bool success;
    std::string uuid;

    netpipe::Message serialize() const {
        netpipe::Message msg;
        msg.push_back(success ? 1 : 0);
        msg.insert(msg.end(), uuid.begin(), uuid.end());
        return msg;
    }

    static SpawnResponse deserialize(const netpipe::Message &msg) {
        if (msg.empty()) return {false, ""};
        bool success = msg[0] == 1;
        std::string uuid(msg.begin() + 1, msg.end());
        return {success, uuid};
    }
};

struct StateUpdate {
    float x, y, theta;

    netpipe::Message serialize() const {
        netpipe::Message msg(sizeof(float) * 3);
        std::memcpy(msg.data(), &x, sizeof(float));
        std::memcpy(msg.data() + sizeof(float), &y, sizeof(float));
        std::memcpy(msg.data() + sizeof(float) * 2, &theta, sizeof(float));
        return msg;
    }

    static StateUpdate deserialize(const netpipe::Message &msg) {
        StateUpdate state;
        std::memcpy(&state.x, msg.data(), sizeof(float));
        std::memcpy(&state.y, msg.data() + sizeof(float), sizeof(float));
        std::memcpy(&state.theta, msg.data() + sizeof(float) * 2, sizeof(float));
        return state;
    }
};

struct ControlCommand {
    float linear, angular;

    netpipe::Message serialize() const {
        netpipe::Message msg(sizeof(float) * 2);
        std::memcpy(msg.data(), &linear, sizeof(float));
        std::memcpy(msg.data() + sizeof(float), &angular, sizeof(float));
        return msg;
    }

    static ControlCommand deserialize(const netpipe::Message &msg) {
        ControlCommand cmd;
        std::memcpy(&cmd.linear, msg.data(), sizeof(float));
        std::memcpy(&cmd.angular, msg.data() + sizeof(float), sizeof(float));
        return cmd;
    }
};

// ============================================================================
// Server (Simulator)
// ============================================================================
void run_simulator() {
    std::cout << "\n=== Simulator (Server) Starting ===\n";

    // Listen for connections
    netpipe::TcpStream listen_stream;
    netpipe::TcpEndpoint endpoint{"0.0.0.0", 7447};

    auto listen_res = listen_stream.listen(endpoint);
    if (listen_res.is_err()) {
        std::cerr << "Listen failed: " << listen_res.error().message.c_str() << "\n";
        return;
    }
    std::cout << "Simulator listening on 0.0.0.0:7447\n";

    // Accept agent connection
    auto client_res = listen_stream.accept();
    if (client_res.is_err()) {
        std::cerr << "Accept failed: " << client_res.error().message.c_str() << "\n";
        return;
    }
    auto client_stream = std::move(client_res.value());
    std::cout << "Agent connected!\n";

    // Create RemotePeer for bidirectional communication
    netpipe::Remote<netpipe::Bidirect> peer(*client_stream, 100, true); // max 100 concurrent, metrics enabled

    // Track spawned agents
    std::string spawned_uuid;
    std::atomic<bool> agent_spawned{false};
    std::atomic<bool> running{true};

    // ========================================================================
    // Register handlers for incoming requests from Agent
    // ========================================================================

    // SPAWN handler
    peer.register_method(
        static_cast<uint32_t>(MsgType::SPAWN), [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
            auto spawn_req = SpawnRequest::deserialize(req);
            std::cout << "[SPAWN] Request from: " << spawn_req.agent_name << "\n";

            // Generate UUID
            spawned_uuid = "agent_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
            agent_spawned = true;

            SpawnResponse resp{true, spawned_uuid};
            std::cout << "[SPAWN] Success, UUID: " << spawned_uuid << "\n";
            return dp::result::ok(resp.serialize());
        });

    // DESPAWN handler
    peer.register_method(static_cast<uint32_t>(MsgType::DESPAWN),
                         [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                             std::string uuid(req.begin(), req.end());
                             std::cout << "[DESPAWN] Request for: " << uuid << "\n";

                             if (uuid == spawned_uuid) {
                                 agent_spawned = false;
                                 running = false;
                                 std::cout << "[DESPAWN] Success\n";
                                 netpipe::Message resp{1}; // Success
                                 return dp::result::ok(resp);
                             }

                             netpipe::Message resp{0}; // Failure
                             return dp::result::ok(resp);
                         });

    // HEARTBEAT handler
    peer.register_method(static_cast<uint32_t>(MsgType::HEARTBEAT),
                         [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                             std::string uuid(req.begin(), req.end());
                             std::cout << "[HEARTBEAT] From: " << uuid << "\n";
                             netpipe::Message resp{1}; // ACK
                             return dp::result::ok(resp);
                         });

    // CONTROL handler
    peer.register_method(static_cast<uint32_t>(MsgType::CONTROL),
                         [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                             auto cmd = ControlCommand::deserialize(req);
                             std::cout << "[CONTROL] linear=" << cmd.linear << ", angular=" << cmd.angular << "\n";
                             netpipe::Message resp{1}; // ACK
                             return dp::result::ok(resp);
                         });

    // LIDAR_CFG handler
    peer.register_method(static_cast<uint32_t>(MsgType::LIDAR_CFG),
                         [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                             std::cout << "[LIDAR_CFG] Received config, size=" << req.size() << " bytes\n";
                             netpipe::Message resp{1}; // ACK
                             return dp::result::ok(resp);
                         });

    // ========================================================================
    // Simulator main loop - sends STATE updates to Agent
    // ========================================================================

    std::cout << "Simulator ready, waiting for spawn...\n";

    // Wait for spawn
    while (!agent_spawned && running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Main simulation loop
    int tick = 0;
    while (running && agent_spawned) {
        // Simulate physics state
        StateUpdate state{static_cast<float>(tick) * 0.1f, static_cast<float>(tick) * 0.05f,
                          static_cast<float>(tick) * 0.01f};

        // Send STATE update to Agent (bidirectional call!)
        std::cout << "[STATE] Sending update (tick=" << tick << ")\n";
        auto state_res = peer.call(static_cast<uint32_t>(MsgType::STATE), state.serialize(), 1000);

        if (state_res.is_err()) {
            std::cerr << "[STATE] Failed to send: " << state_res.error().message.c_str() << "\n";
            break;
        }

        tick++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (tick >= 20) {
            std::cout << "[Simulator] Stopping after 20 ticks\n";
            break;
        }
    }

    // Print metrics
    std::cout << "\n=== Simulator Metrics ===\n";
    const auto &client_metrics = peer.get_client_metrics();
    const auto &server_metrics = peer.get_server_metrics();
    std::cout << "Outgoing calls (to Agent):\n";
    std::cout << "  Total: " << client_metrics.total_requests.load() << "\n";
    std::cout << "  Success: " << client_metrics.successful_requests.load() << "\n";
    std::cout << "  Failed: " << client_metrics.failed_requests.load() << "\n";
    std::cout << "Incoming requests (from Agent):\n";
    std::cout << "  Total: " << server_metrics.total_requests.load() << "\n";
    std::cout << "  Success: " << server_metrics.successful_requests.load() << "\n";
    std::cout << "  Failed: " << server_metrics.failed_requests.load() << "\n";

    client_stream->close();
    std::cout << "Simulator stopped\n";
}

// ============================================================================
// Client (Agent)
// ============================================================================
void run_agent() {
    std::cout << "\n=== Agent (Client) Starting ===\n";

    // Wait for server to start
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Connect to simulator
    netpipe::TcpStream stream;
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 7447};

    auto connect_res = stream.connect(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "Connect failed: " << connect_res.error().message.c_str() << "\n";
        return;
    }
    std::cout << "Connected to simulator!\n";

    // Create RemotePeer for bidirectional communication
    netpipe::Remote<netpipe::Bidirect> peer(stream, 100, true); // max 100 concurrent, metrics enabled

    std::atomic<int> state_updates_received{0};

    // ========================================================================
    // Register handlers for incoming calls from Simulator
    // ========================================================================

    // STATE handler (receives state updates from simulator)
    peer.register_method(static_cast<uint32_t>(MsgType::STATE),
                         [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                             auto state = StateUpdate::deserialize(req);
                             state_updates_received++;
                             std::cout << "[STATE] Received update: x=" << state.x << ", y=" << state.y
                                       << ", theta=" << state.theta << "\n";
                             netpipe::Message resp{1}; // ACK
                             return dp::result::ok(resp);
                         });

    // SENSORS handler (receives sensor data from simulator)
    peer.register_method(static_cast<uint32_t>(MsgType::SENSORS),
                         [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                             std::cout << "[SENSORS] Received sensor data, size=" << req.size() << " bytes\n";
                             netpipe::Message resp{1}; // ACK
                             return dp::result::ok(resp);
                         });

    // TELEPORT handler (receives teleport commands from simulator)
    peer.register_method(static_cast<uint32_t>(MsgType::TELEPORT),
                         [&](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                             std::cout << "[TELEPORT] Received teleport command\n";
                             netpipe::Message resp{1}; // ACK
                             return dp::result::ok(resp);
                         });

    // ========================================================================
    // Agent main loop - sends commands to Simulator
    // ========================================================================

    // SPAWN
    std::cout << "\n--- Testing SPAWN ---\n";
    SpawnRequest spawn_req{"TestAgent"};
    auto spawn_res = peer.call(static_cast<uint32_t>(MsgType::SPAWN), spawn_req.serialize(), 5000);

    if (spawn_res.is_err()) {
        std::cerr << "SPAWN failed: " << spawn_res.error().message.c_str() << "\n";
        return;
    }

    auto spawn_resp = SpawnResponse::deserialize(spawn_res.value());
    if (!spawn_resp.success) {
        std::cerr << "SPAWN rejected by simulator\n";
        return;
    }

    std::cout << "SPAWN successful! UUID: " << spawn_resp.uuid << "\n";
    std::string my_uuid = spawn_resp.uuid;

    // Send HEARTBEAT
    std::cout << "\n--- Testing HEARTBEAT ---\n";
    netpipe::Message hb_msg(my_uuid.begin(), my_uuid.end());
    auto hb_res = peer.call(static_cast<uint32_t>(MsgType::HEARTBEAT), hb_msg, 5000);
    if (hb_res.is_ok()) {
        std::cout << "HEARTBEAT acknowledged\n";
    }

    // Send CONTROL commands
    std::cout << "\n--- Testing CONTROL ---\n";
    for (int i = 0; i < 10; i++) {
        ControlCommand cmd{static_cast<float>(i) * 0.5f, static_cast<float>(i) * 0.2f};
        auto ctrl_res = peer.call(static_cast<uint32_t>(MsgType::CONTROL), cmd.serialize(), 5000);
        if (ctrl_res.is_ok()) {
            std::cout << "[Agent] CONTROL sent (linear=" << cmd.linear << ", angular=" << cmd.angular << ")\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Send LIDAR_CFG
    std::cout << "\n--- Testing LIDAR_CFG ---\n";
    netpipe::Message lidar_cfg{0x01, 0x02, 0x03, 0x04}; // Dummy config
    auto lidar_res = peer.call(static_cast<uint32_t>(MsgType::LIDAR_CFG), lidar_cfg, 5000);
    if (lidar_res.is_ok()) {
        std::cout << "LIDAR_CFG sent\n";
    }

    // Wait for more state updates
    std::cout << "\n--- Waiting for STATE updates ---\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // DESPAWN
    std::cout << "\n--- Testing DESPAWN ---\n";
    netpipe::Message despawn_msg(my_uuid.begin(), my_uuid.end());
    auto despawn_res = peer.call(static_cast<uint32_t>(MsgType::DESPAWN), despawn_msg, 5000);
    if (despawn_res.is_ok() && despawn_res.value()[0] == 1) {
        std::cout << "DESPAWN successful\n";
    }

    // Print metrics
    std::cout << "\n=== Agent Metrics ===\n";
    const auto &client_metrics = peer.get_client_metrics();
    const auto &server_metrics = peer.get_server_metrics();
    std::cout << "Outgoing calls (to Simulator):\n";
    std::cout << "  Total: " << client_metrics.total_requests.load() << "\n";
    std::cout << "  Success: " << client_metrics.successful_requests.load() << "\n";
    std::cout << "  Failed: " << client_metrics.failed_requests.load() << "\n";
    std::cout << "Incoming requests (from Simulator):\n";
    std::cout << "  Total: " << server_metrics.total_requests.load() << "\n";
    std::cout << "  Success: " << server_metrics.successful_requests.load() << "\n";
    std::cout << "  Failed: " << server_metrics.failed_requests.load() << "\n";
    std::cout << "STATE updates received: " << state_updates_received.load() << "\n";

    stream.close();
    std::cout << "Agent stopped\n";
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char **argv) {
    std::cout << "==============================================\n";
    std::cout << "Netpipe Bidirectional Test (RemotePeer)\n";
    std::cout << "Single channel, both sides call each other\n";
    std::cout << "==============================================\n";

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " [server|client|both]\n";
        return 1;
    }

    std::string mode(argv[1]);

    if (mode == "server") {
        run_simulator();
    } else if (mode == "client") {
        run_agent();
    } else if (mode == "both") {
        // Run both in separate threads
        std::thread server_thread(run_simulator);
        std::thread client_thread(run_agent);

        client_thread.join();
        server_thread.join();
    } else {
        std::cerr << "Unknown mode: " << mode << "\n";
        return 1;
    }

    std::cout << "\n✓ Test completed\n";
    return 0;
}
