#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <random>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <net/if.h>

enum class Protocol { NONE = -1, DDS_RTPS = 0, ZENOH = 1, MQTT = 2 };

enum class Medium { WIFI_5GHZ = 1, CELLULAR_5G = 2 };

struct GeoPoint {
    double latitude;
    double longitude;
    double altitude;
};

struct AgentMessage {
    uint64_t timestamp;
    char public_key[64];
    char uuid[37];
    bool orchestrator;
    GeoPoint zero_ref;
    char participant_uuids[10][37];
    int32_t capability_index;
    uint32_t medium;
    uint32_t protocol;
    char ipv6_addresses[3][46];
    uint32_t robot_id;
    char robot_name[32];
    
    void serialize(char *buffer) const { memcpy(buffer, this, sizeof(AgentMessage)); }
    
    static AgentMessage deserialize(const char *buffer) {
        AgentMessage msg;
        memcpy(&msg, buffer, sizeof(AgentMessage));
        return msg;
    }
};

class ARISRobot {
  private:
    std::string name;
    std::string uuid;
    uint32_t robot_id;
    Protocol chosen_protocol;
    int32_t capability_index;

    int multicast_fd;
    std::thread discovery_thread;
    std::atomic<bool> running;

    std::map<std::string, AgentMessage> known_robots;
    mutable std::mutex robots_mutex;

    static constexpr uint16_t ARIS_PORT = 7447;

    std::atomic<int> tokens;
    std::chrono::steady_clock::time_point last_token_update;

  public:
    ARISRobot(const std::string &name, uint32_t id, int32_t capability = 75)
        : name(name), robot_id(id), chosen_protocol(Protocol::NONE),
          capability_index(capability), multicast_fd(-1), running(false), tokens(1000) {

        uuid = generate_uuid();
        last_token_update = std::chrono::steady_clock::now();
    }

    ~ARISRobot() { stop(); }

    bool start() {
        multicast_fd = socket(AF_INET6, SOCK_DGRAM, 0);
        if (multicast_fd < 0) {
            std::cerr << name << ": Failed to create socket" << std::endl;
            return false;
        }

        int reuse = 1;
        setsockopt(multicast_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct sockaddr_in6 addr = {};
        addr.sin6_family = AF_INET6;
        addr.sin6_port = htons(ARIS_PORT);
        addr.sin6_addr = in6addr_any;

        if (bind(multicast_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            std::cerr << name << ": Failed to bind to port " << ARIS_PORT << std::endl;
            close(multicast_fd);
            return false;
        }

        struct ipv6_mreq mreq = {};
        inet_pton(AF_INET6, "ff02::1234", &mreq.ipv6mr_multiaddr); // Use simpler multicast address
        mreq.ipv6mr_interface = if_nametoindex("eno2"); // Use active ethernet interface
        if (mreq.ipv6mr_interface == 0) {
            mreq.ipv6mr_interface = if_nametoindex("lo"); // Fallback to loopback
        }

        if (setsockopt(multicast_fd, IPPROTO_IPV6, IPV6_JOIN_GROUP, &mreq, sizeof(mreq)) < 0) {
            std::cerr << name << ": Failed to join multicast group" << std::endl;
            close(multicast_fd);
            return false;
        }

        std::cout << name << " (" << uuid << ") joined ARIS multicast group" << std::endl;

        running = true;
        discovery_thread = std::thread(&ARISRobot::discovery_loop, this);

        return true;
    }

    void stop() {
        running = false;
        if (multicast_fd >= 0) {
            close(multicast_fd);
            multicast_fd = -1;
        }
        if (discovery_thread.joinable()) {
            discovery_thread.join();
        }
    }

    Protocol get_protocol() const { return chosen_protocol; }
    int32_t get_capability() const { return capability_index; }

    void print_status() const {
        std::lock_guard<std::mutex> lock(robots_mutex);
        std::cout << "\n" << name << " Status:" << std::endl;
        std::cout << "  UUID: " << uuid << std::endl;
        std::cout << "  Protocol: " << protocol_to_string(chosen_protocol) << std::endl;
        std::cout << "  Capability: " << capability_index << "/100" << std::endl;
        std::cout << "  Tokens: " << tokens.load() << std::endl;
        std::cout << "  Known robots: " << known_robots.size() << std::endl;
        for (const auto &[id, agent] : known_robots) {
            std::cout << "    - " << agent.robot_name << " (" << id << ") cap:" << agent.capability_index << std::endl;
        }
    }

  private:
    void discovery_loop() {
        // Listen for existing network
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(5, 15);
        auto listen_duration = std::chrono::seconds(dis(gen));
        auto listen_start = std::chrono::steady_clock::now();

        std::cout << name << " listening for " << listen_duration.count() << " seconds..." << std::endl;

        bool heard_network = false;

        while (running && (std::chrono::steady_clock::now() - listen_start) < listen_duration) {
            if (receive_message()) {
                heard_network = true;
                break;
            }
            update_tokens();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!running) return;

        if (!heard_network) {
            // First robot - select and announce protocol at 1Hz
            chosen_protocol = select_protocol();
            std::cout << name << " is first robot, selected protocol: " << protocol_to_string(chosen_protocol) << std::endl;
            std::cout << name << " announcing protocol at 1Hz until others join..." << std::endl;
            
            // Announce protocol at 1Hz until others join
            while (running && known_robots.empty()) {
                if (consume_tokens(30)) {
                    send_agent_message();
                }
                
                // Listen for responses for 1 second
                for (int i = 0; i < 10 && running; i++) {
                    receive_message();
                    update_tokens();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            
            if (!known_robots.empty()) {
                std::cout << name << " detected other robots, network established!" << std::endl;
            }
        }

        // Operating mode with heartbeats every 2 seconds
        while (running) {
            if (consume_tokens(10)) {
                send_agent_message();
            }

            // Listen and update for 2 seconds (heartbeat interval)
            for (int i = 0; i < 20 && running; i++) {
                receive_message();
                update_tokens();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    Protocol select_protocol() {
        if (capability_index >= 90) {
            return Protocol::DDS_RTPS;
        } else if (capability_index >= 60) {
            return Protocol::ZENOH;
        } else {
            return Protocol::MQTT;
        }
    }

    void send_agent_message() {
        AgentMessage msg = {};
        msg.timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();

        strncpy(msg.public_key, "ed25519_public_key_placeholder", sizeof(msg.public_key) - 1);
        strncpy(msg.uuid, uuid.c_str(), sizeof(msg.uuid) - 1);
        msg.orchestrator = false;

        msg.zero_ref = {40.7128, -74.0060, 0.0};

        msg.capability_index = capability_index;
        msg.medium = static_cast<uint32_t>(Medium::WIFI_5GHZ);
        msg.protocol = static_cast<uint32_t>(chosen_protocol);

        strncpy(msg.ipv6_addresses[0], "fe80::1", 45);
        strncpy(msg.ipv6_addresses[1], "fd00::1", 45);
        strncpy(msg.ipv6_addresses[2], "", 45);

        msg.robot_id = robot_id;
        strncpy(msg.robot_name, name.c_str(), sizeof(msg.robot_name) - 1);

        char buffer[sizeof(AgentMessage)];
        msg.serialize(buffer);

        struct sockaddr_in6 dest = {};
        dest.sin6_family = AF_INET6;
        dest.sin6_port = htons(ARIS_PORT);
        inet_pton(AF_INET6, "ff02::1234", &dest.sin6_addr); // Use same multicast address
        dest.sin6_scope_id = if_nametoindex("eno2"); // Set scope for active interface
        if (dest.sin6_scope_id == 0) {
            dest.sin6_scope_id = if_nametoindex("lo"); // Fallback to loopback
        }

        sendto(multicast_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&dest, sizeof(dest));
    }

    bool receive_message() {
        char buffer[1024];
        struct sockaddr_in6 from;
        socklen_t from_len = sizeof(from);

        ssize_t received =
            recvfrom(multicast_fd, buffer, sizeof(buffer), MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);

        if (received == sizeof(AgentMessage)) {
            AgentMessage msg = AgentMessage::deserialize(buffer);

            if (std::string(msg.uuid) == uuid) {
                return false;
            }

            if (chosen_protocol == Protocol::NONE) {
                chosen_protocol = static_cast<Protocol>(msg.protocol);
                std::cout << name << " adopted protocol: " << protocol_to_string(chosen_protocol) << std::endl;
            }

            if (should_share_info_with(msg.capability_index)) {
                std::lock_guard<std::mutex> lock(robots_mutex);
                std::string robot_uuid(msg.uuid);
                if (known_robots.find(robot_uuid) == known_robots.end()) {
                    std::cout << name << " discovered: " << msg.robot_name << " (" << robot_uuid
                              << ") cap:" << msg.capability_index << std::endl;
                }
                known_robots[robot_uuid] = msg;
            }

            return true;
        }

        return false;
    }

    bool should_share_info_with(int32_t other_capability) {
        if (capability_index >= 90 || other_capability >= 90) {
            return true;
        } else if (capability_index >= 60 && other_capability >= 60) {
            return true;
        } else if (capability_index >= 50 && other_capability >= 50) {
            return true;
        }
        return capability_index >= 25 && other_capability >= 25;
    }

    void update_tokens() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_token_update);

        if (elapsed.count() > 100) {
            int bandwidth_mbps = 10;
            int new_tokens = (bandwidth_mbps * elapsed.count()) / 10;

            tokens = std::min(tokens.load() + new_tokens, 1000);
            last_token_update = now;
        }
    }

    bool consume_tokens(int count) {
        int current = tokens.load();
        if (current >= count) {
            tokens -= count;
            return true;
        }
        return false;
    }

    std::string generate_uuid() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 15);

        char uuid_str[37];
        snprintf(uuid_str, sizeof(uuid_str), "%08x-%04x-%04x-%04x-%012lx", robot_id, 4096, 16384,
                 dis(gen) * 4096 + dis(gen) * 256 + dis(gen) * 16 + dis(gen),
                 (unsigned long)(std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count() &
                                 0xFFFFFFFFFFFF));

        return std::string(uuid_str);
    }

    std::string protocol_to_string(Protocol p) const {
        switch (p) {
        case Protocol::NONE:
            return "NONE";
        case Protocol::DDS_RTPS:
            return "DDS/RTPS";
        case Protocol::ZENOH:
            return "ZENOH";
        case Protocol::MQTT:
            return "MQTT";
        default:
            return "UNKNOWN";
        }
    }
};

class ARISNetwork {
  private:
    std::vector<std::unique_ptr<ARISRobot>> robots;

  public:
    void add_robot(const std::string &name, uint32_t id, int32_t capability = 75) {
        auto robot = std::make_unique<ARISRobot>(name, id, capability);
        if (robot->start()) {
            robots.push_back(std::move(robot));
        }
    }

    void print_network_status() {
        std::cout << "\n=== ARIS P2P Network Status ===" << std::endl;
        for (const auto &robot : robots) {
            robot->print_status();
        }
        std::cout << std::endl;
    }

    void wait_for_discovery() {
        std::cout << "\nWaiting for P2P discovery..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(25));
    }
};

int main() {
    std::cout << "=== ARIS P2P Discovery Demo ===" << std::endl;
    std::cout << "True peer-to-peer - no leaders, just protocol adoption" << std::endl;
    std::cout << "Using IPv6 multicast group ff02::1234 on interface eno2\n" << std::endl;

    ARISNetwork network;

    // Add Robot 1 - it will listen then create network
    std::cout << "Adding Robot 1 (Tractor-Alpha)..." << std::endl;
    network.add_robot("Tractor-Alpha", 1001, 95);
    
    std::cout << "Waiting for Robot 1 to establish network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(20)); // Wait for robot 1 to finish listening and start announcing
    
    // Add Robot 2 - it will detect existing network and join
    std::cout << "\nAdding Robot 2 (Harvester-Beta)..." << std::endl;
    network.add_robot("Harvester-Beta", 2002, 80);
    
    std::cout << "Waiting for Robot 2 to join network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10)); // Wait for robot 2 to hear and join
    
    // Add Robot 3 - it will detect existing network and join
    std::cout << "\nAdding Robot 3 (Sprayer-Gamma)..." << std::endl;
    network.add_robot("Sprayer-Gamma", 3003, 60);
    
    std::cout << "Waiting for Robot 3 to join network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10)); // Wait for robot 3 to hear and join
    
    // Add Robot 4 - it will detect existing network and join
    std::cout << "\nAdding Robot 4 (Feeder-Delta)..." << std::endl;
    network.add_robot("Feeder-Delta", 4004, 40);
    
    std::cout << "Waiting for Robot 4 to join network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10)); // Wait for robot 4 to hear and join

    network.print_network_status();

    std::cout << "\nP2P discovery complete! All robots are equal peers." << std::endl;
    std::cout << "Press Enter to shutdown..." << std::endl;
    std::cin.get();

    return 0;
}