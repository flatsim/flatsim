#include "flatsim/robot/network/lan.hpp"
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <net/if.h>
#include <netinet/in.h>
#include <random>
#include <sys/socket.h>
#include <unistd.h>

enum class Protocol : int32_t { NONE = -1, DDS_RTPS = 0, ZENOH = 1, MQTT = 2 };

enum class Medium : uint32_t { WIFI_5GHZ = 1, CELLULAR_5G = 2 };

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

    void serialize(char *buffer) const;
    static AgentMessage deserialize(const char *buffer);
};

class ARISRobot {
  private:
    std::string name_;
    std::string uuid_;
    uint32_t robot_id_;
    Protocol chosen_protocol_;
    int32_t capability_index_;
    std::unique_ptr<LanInterface> lan_interface_;

    int multicast_fd_;
    std::thread discovery_thread_;
    std::atomic<bool> running_;

    std::map<std::string, AgentMessage> known_robots_;
    mutable std::mutex robots_mutex_;

    static constexpr uint16_t ARIS_PORT = 7447;

    std::atomic<int> tokens_;
    std::chrono::steady_clock::time_point last_token_update_;

    void discovery_loop();
    Protocol select_protocol();
    void send_agent_message();
    bool receive_message();
    bool should_share_info_with(int32_t other_capability);
    void update_tokens();
    bool consume_tokens(int count);
    std::string generate_uuid();
    std::string protocol_to_string(Protocol p) const;

  public:
    ARISRobot(const std::string &name, uint32_t id, int32_t capability = 75, const std::string &interface = "");
    ~ARISRobot();

    bool start();
    void stop();

    Protocol get_protocol() const;
    int32_t get_capability() const;
    void print_status() const;
};

class ARISNetwork {
  private:
    std::vector<std::unique_ptr<ARISRobot>> robots_;

  public:
    void add_robot(const std::string &name, uint32_t id, int32_t capability = 75, const std::string &interface = "");
    void print_network_status();
};

// AgentMessage Implementation
void AgentMessage::serialize(char *buffer) const {
    memcpy(buffer, this, sizeof(AgentMessage));
}

AgentMessage AgentMessage::deserialize(const char *buffer) {
    AgentMessage msg;
    memcpy(&msg, buffer, sizeof(AgentMessage));
    return msg;
}

// ARISRobot Implementation
ARISRobot::ARISRobot(const std::string &name, uint32_t id, int32_t capability, const std::string &interface)
    : name_(name), robot_id_(id), chosen_protocol_(Protocol::NONE), capability_index_(capability),
      multicast_fd_(-1), running_(false), tokens_(1000) {

    lan_interface_ = std::make_unique<LanInterface>(name + "_lan", id, interface);
    uuid_ = generate_uuid();
    last_token_update_ = std::chrono::steady_clock::now();
}

ARISRobot::~ARISRobot() {
    stop();
}

bool ARISRobot::start() {
    if (!lan_interface_->start()) {
        return false;
    }

    multicast_fd_ = socket(AF_INET6, SOCK_DGRAM, 0);
    if (multicast_fd_ < 0) {
        std::cerr << name_ << ": Failed to create ARIS socket" << std::endl;
        return false;
    }

    int reuse = 1;
    setsockopt(multicast_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in6 addr = {};
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(ARIS_PORT);
    addr.sin6_addr = in6addr_any;

    if (bind(multicast_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << name_ << ": Failed to bind to ARIS port " << ARIS_PORT << std::endl;
        close(multicast_fd_);
        return false;
    }

    struct ipv6_mreq mreq = {};
    inet_pton(AF_INET6, "ff02::1234", &mreq.ipv6mr_multiaddr);
    mreq.ipv6mr_interface = if_nametoindex(lan_interface_->get_interface().c_str());
    if (mreq.ipv6mr_interface == 0) {
        mreq.ipv6mr_interface = if_nametoindex("lo");
    }

    if (setsockopt(multicast_fd_, IPPROTO_IPV6, IPV6_JOIN_GROUP, &mreq, sizeof(mreq)) < 0) {
        std::cerr << name_ << ": Failed to join ARIS multicast group" << std::endl;
        close(multicast_fd_);
        return false;
    }

    std::cout << name_ << " (" << uuid_ << ") joined ARIS multicast group on " << lan_interface_->get_interface() << std::endl;

    running_ = true;
    discovery_thread_ = std::thread(&ARISRobot::discovery_loop, this);

    return true;
}

void ARISRobot::stop() {
    running_ = false;
    if (multicast_fd_ >= 0) {
        close(multicast_fd_);
        multicast_fd_ = -1;
    }
    if (discovery_thread_.joinable()) {
        discovery_thread_.join();
    }
    if (lan_interface_) {
        lan_interface_->stop();
    }
}

Protocol ARISRobot::get_protocol() const {
    return chosen_protocol_;
}

int32_t ARISRobot::get_capability() const {
    return capability_index_;
}

void ARISRobot::print_status() const {
    std::lock_guard<std::mutex> lock(robots_mutex_);
    std::cout << "\n" << name_ << " Status:" << std::endl;
    std::cout << "  UUID: " << uuid_ << std::endl;
    std::cout << "  Interface: " << lan_interface_->get_interface() << std::endl;
    std::cout << "  IPv6: " << lan_interface_->get_ipv6() << std::endl;
    std::cout << "  Protocol: " << protocol_to_string(chosen_protocol_) << std::endl;
    std::cout << "  Capability: " << capability_index_ << "/100" << std::endl;
    std::cout << "  Tokens: " << tokens_.load() << std::endl;
    std::cout << "  Known robots: " << known_robots_.size() << std::endl;
    for (const auto &[id, agent] : known_robots_) {
        std::cout << "    - " << agent.robot_name << " (" << id << ") cap:" << agent.capability_index << std::endl;
    }
}

void ARISRobot::discovery_loop() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(5, 15);
    auto listen_duration = std::chrono::seconds(dis(gen));
    auto listen_start = std::chrono::steady_clock::now();

    std::cout << name_ << " listening for " << listen_duration.count() << " seconds..." << std::endl;

    bool heard_network = false;

    while (running_ && (std::chrono::steady_clock::now() - listen_start) < listen_duration) {
        if (receive_message()) {
            heard_network = true;
            break;
        }
        update_tokens();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!running_) return;

    if (!heard_network) {
        chosen_protocol_ = select_protocol();
        std::cout << name_ << " is first robot, selected protocol: " << protocol_to_string(chosen_protocol_) << std::endl;

        while (running_ && known_robots_.empty()) {
            if (consume_tokens(30)) {
                send_agent_message();
            }

            for (int i = 0; i < 10 && running_; i++) {
                receive_message();
                update_tokens();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        if (!known_robots_.empty()) {
            std::cout << name_ << " detected other robots, network established!" << std::endl;
        }
    }

    while (running_) {
        if (consume_tokens(10)) {
            send_agent_message();
        }

        for (int i = 0; i < 20 && running_; i++) {
            receive_message();
            update_tokens();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

Protocol ARISRobot::select_protocol() {
    if (capability_index_ >= 90) {
        return Protocol::DDS_RTPS;
    } else if (capability_index_ >= 60) {
        return Protocol::ZENOH;
    } else {
        return Protocol::MQTT;
    }
}

void ARISRobot::send_agent_message() {
    AgentMessage msg = {};
    msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    strncpy(msg.public_key, "ed25519_public_key_placeholder", sizeof(msg.public_key) - 1);
    strncpy(msg.uuid, uuid_.c_str(), sizeof(msg.uuid) - 1);
    msg.orchestrator = false;

    msg.zero_ref = {40.7128, -74.0060, 0.0};

    msg.capability_index = capability_index_;
    msg.medium = static_cast<uint32_t>(Medium::WIFI_5GHZ);
    msg.protocol = static_cast<uint32_t>(chosen_protocol_);

    strncpy(msg.ipv6_addresses[0], lan_interface_->get_ipv6().c_str(), 45);
    strncpy(msg.ipv6_addresses[1], "", 45);
    strncpy(msg.ipv6_addresses[2], "", 45);

    msg.robot_id = robot_id_;
    strncpy(msg.robot_name, name_.c_str(), sizeof(msg.robot_name) - 1);

    char buffer[sizeof(AgentMessage)];
    msg.serialize(buffer);

    struct sockaddr_in6 dest = {};
    dest.sin6_family = AF_INET6;
    dest.sin6_port = htons(ARIS_PORT);
    inet_pton(AF_INET6, "ff02::1234", &dest.sin6_addr);
    dest.sin6_scope_id = if_nametoindex(lan_interface_->get_interface().c_str());
    if (dest.sin6_scope_id == 0) {
        dest.sin6_scope_id = if_nametoindex("lo");
    }

    sendto(multicast_fd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&dest, sizeof(dest));
}

bool ARISRobot::receive_message() {
    char buffer[1024];
    struct sockaddr_in6 from;
    socklen_t from_len = sizeof(from);

    ssize_t received = recvfrom(multicast_fd_, buffer, sizeof(buffer), MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);

    if (received == sizeof(AgentMessage)) {
        AgentMessage msg = AgentMessage::deserialize(buffer);

        if (std::string(msg.uuid) == uuid_) {
            return false;
        }

        if (chosen_protocol_ == Protocol::NONE) {
            chosen_protocol_ = static_cast<Protocol>(msg.protocol);
            std::cout << name_ << " adopted protocol: " << protocol_to_string(chosen_protocol_) << std::endl;
        }

        if (should_share_info_with(msg.capability_index)) {
            std::lock_guard<std::mutex> lock(robots_mutex_);
            std::string robot_uuid(msg.uuid);
            if (known_robots_.find(robot_uuid) == known_robots_.end()) {
                std::cout << name_ << " discovered: " << msg.robot_name << " (" << robot_uuid
                          << ") cap:" << msg.capability_index << std::endl;
            }
            known_robots_[robot_uuid] = msg;
        }

        return true;
    }

    return false;
}

bool ARISRobot::should_share_info_with(int32_t other_capability) {
    if (capability_index_ >= 90 || other_capability >= 90) {
        return true;
    } else if (capability_index_ >= 60 && other_capability >= 60) {
        return true;
    } else if (capability_index_ >= 50 && other_capability >= 50) {
        return true;
    }
    return capability_index_ >= 25 && other_capability >= 25;
}

void ARISRobot::update_tokens() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_token_update_);

    if (elapsed.count() > 100) {
        int bandwidth_mbps = 10;
        int new_tokens = (bandwidth_mbps * elapsed.count()) / 10;

        tokens_ = std::min(tokens_.load() + new_tokens, 1000);
        last_token_update_ = now;
    }
}

bool ARISRobot::consume_tokens(int count) {
    int current = tokens_.load();
    if (current >= count) {
        tokens_ -= count;
        return true;
    }
    return false;
}

std::string ARISRobot::generate_uuid() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);

    char uuid_str[37];
    snprintf(uuid_str, sizeof(uuid_str), "%08x-%04x-%04x-%04x-%012lx", robot_id_, 4096, 16384,
             dis(gen) * 4096 + dis(gen) * 256 + dis(gen) * 16 + dis(gen),
             (unsigned long)(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count() &
                             0xFFFFFFFFFFFF));

    return std::string(uuid_str);
}

std::string ARISRobot::protocol_to_string(Protocol p) const {
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

// ARISNetwork Implementation
void ARISNetwork::add_robot(const std::string &name, uint32_t id, int32_t capability, const std::string &interface) {
    auto robot = std::make_unique<ARISRobot>(name, id, capability, interface);
    if (robot->start()) {
        robots_.push_back(std::move(robot));
    }
}

void ARISNetwork::print_network_status() {
    std::cout << "\n=== ARIS P2P Network Status ===" << std::endl;
    for (const auto &robot : robots_) {
        robot->print_status();
    }
    std::cout << std::endl;
}

int main() {
    std::cout << "=== ARIS P2P Discovery with LAN Demo ===" << std::endl;
    std::cout << "True peer-to-peer with automatic interface creation" << std::endl;
    std::cout << "Using IPv6 multicast group ff02::1234\n" << std::endl;

    ARISNetwork network;

    std::cout << "Adding Robot 1 (Tractor-Alpha)..." << std::endl;
    network.add_robot("Tractor-Alpha", 1001, 95);

    std::cout << "Waiting for Robot 1 to establish network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(20));

    std::cout << "\nAdding Robot 2 (Harvester-Beta)..." << std::endl;
    network.add_robot("Harvester-Beta", 2002, 80);

    std::cout << "Waiting for Robot 2 to join network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "\nAdding Robot 3 (Sprayer-Gamma)..." << std::endl;
    network.add_robot("Sprayer-Gamma", 3003, 60);

    std::cout << "Waiting for Robot 3 to join network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "\nAdding Robot 4 (Feeder-Delta)..." << std::endl;
    network.add_robot("Feeder-Delta", 4004, 40);

    std::cout << "Waiting for Robot 4 to join network..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    network.print_network_status();

    std::cout << "\nP2P discovery complete! All robots are equal peers." << std::endl;
    std::cout << "Press Enter to shutdown..." << std::endl;
    std::cin.get();

    return 0;
}