#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

// Simulated Robot with IPv6 networking
class NetworkedRobot {
  private:
    std::string name;
    std::string ipv6_address;
    uint16_t port;
    int socket_fd;
    std::thread receive_thread;
    bool running;

  public:
    NetworkedRobot(const std::string &name, int id) : name(name), port(8000 + id), socket_fd(-1), running(false) {
        // Generate unique IPv6 address
        char addr_buf[64];
        snprintf(addr_buf, sizeof(addr_buf), "fd00:dead:beef::%x", id);
        ipv6_address = addr_buf;
    }

    ~NetworkedRobot() { stop(); }

    bool start() {
        // Add IPv6 address to default interface (requires sudo)
        std::string interface = "lo"; // Using loopback for demo
        std::string cmd = "ip -6 addr add " + ipv6_address + "/128 dev " + interface + " 2>/dev/null";
        if (system(cmd.c_str()) != 0) {
            std::cerr << name << ": Failed to add IPv6 address (try with sudo)" << std::endl;
            // Continue anyway - will use existing addresses
        } else {
            std::cout << name << ": Added IPv6 address " << ipv6_address << " to " << interface << std::endl;
        }

        // Create IPv6 UDP socket
        socket_fd = socket(AF_INET6, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            std::cerr << name << ": Failed to create socket" << std::endl;
            return false;
        }

        // Allow IPv6-only socket
        int v6only = 1;
        setsockopt(socket_fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));

        // Bind to our specific IPv6 address
        struct sockaddr_in6 addr = {};
        addr.sin6_family = AF_INET6;
        addr.sin6_port = htons(port);

        // Try to bind to our specific address first
        if (inet_pton(AF_INET6, ipv6_address.c_str(), &addr.sin6_addr) == 1) {
            if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
                std::cout << name << " bound to [" << ipv6_address << "]:" << port << std::endl;
            } else {
                // Fall back to any address
                addr.sin6_addr = in6addr_any;
                if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                    std::cerr << name << ": Failed to bind to port " << port << std::endl;
                    close(socket_fd);
                    return false;
                }
                std::cout << name << " bound to [::] (any):" << port << std::endl;
            }
        }

        // Start receive thread
        running = true;
        receive_thread = std::thread(&NetworkedRobot::receive_loop, this);

        return true;
    }

    void stop() {
        running = false;
        if (socket_fd >= 0) {
            close(socket_fd);
            socket_fd = -1;
        }
        if (receive_thread.joinable()) {
            receive_thread.join();
        }

        // Remove IPv6 address
        std::string interface = "lo";
        std::string cmd = "ip -6 addr del " + ipv6_address + "/128 dev " + interface + " 2>/dev/null";
        system(cmd.c_str());
    }

    void send_message(const std::string &dest_addr, uint16_t dest_port, const std::string &msg) {
        if (socket_fd < 0) return;

        struct sockaddr_in6 dest = {};
        dest.sin6_family = AF_INET6;
        dest.sin6_port = htons(dest_port);

        if (inet_pton(AF_INET6, dest_addr.c_str(), &dest.sin6_addr) != 1) {
            std::cerr << name << ": Invalid destination address" << std::endl;
            return;
        }

        ssize_t sent = sendto(socket_fd, msg.c_str(), msg.length(), 0, (struct sockaddr *)&dest, sizeof(dest));

        if (sent > 0) {
            std::cout << name << " sent: \"" << msg << "\" to [" << dest_addr << "]:" << dest_port << std::endl;
        }
    }

    const std::string &get_ipv6() const { return ipv6_address; }
    uint16_t get_port() const { return port; }

  private:
    void receive_loop() {
        char buffer[1024];
        struct sockaddr_in6 from;
        socklen_t from_len;

        while (running) {
            from_len = sizeof(from);
            ssize_t received =
                recvfrom(socket_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);

            if (received > 0) {
                buffer[received] = '\0';
                char addr_str[INET6_ADDRSTRLEN];
                inet_ntop(AF_INET6, &from.sin6_addr, addr_str, sizeof(addr_str));

                std::cout << name << " received: \"" << buffer << "\" from [" << addr_str
                          << "]:" << ntohs(from.sin6_port) << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

// Robot Network Manager
class RobotNetworkManager {
  private:
    std::vector<std::unique_ptr<NetworkedRobot>> robots;

  public:
    void add_robot(const std::string &name) {
        int id = robots.size() + 1;
        auto robot = std::make_unique<NetworkedRobot>(name, id);

        if (robot->start()) {
            robots.push_back(std::move(robot));
        }
    }

    void broadcast_message(const std::string &msg) {
        if (robots.size() < 2) return;

        // Each robot sends to all others using their actual IPv6 addresses
        for (size_t i = 0; i < robots.size(); i++) {
            for (size_t j = 0; j < robots.size(); j++) {
                if (i != j) {
                    robots[i]->send_message(robots[j]->get_ipv6(), robots[j]->get_port(), msg);
                }
            }
        }
    }

    void list_robots() {
        std::cout << "\nActive robots:" << std::endl;
        for (const auto &robot : robots) {
            std::cout << "  - IPv6: [" << robot->get_ipv6() << "]:" << robot->get_port() << std::endl;
        }
    }
};

int main() {
    std::cout << "=== Robot IPv6 Network Demo ===" << std::endl;
    std::cout << "NOTE: Run with sudo to assign IPv6 addresses\n" << std::endl;

    RobotNetworkManager manager;

    // Create multiple robots with unique IPv6 addresses
    manager.add_robot("Tractor-1");
    manager.add_robot("Harvester-1");
    manager.add_robot("Sprayer-1");

    manager.list_robots();

    std::cout << "\nCheck assigned IPv6 addresses with: ip -6 addr show dev lo" << std::endl;
    std::cout << "Press 'c' to continue with communication test, 'q' to quit: ";

    std::string input;
    while (std::getline(std::cin, input)) {
        if (input == "q") {
            std::cout << "\nShutting down..." << std::endl;
            return 0;
        } else if (input == "c") {
            break;
        }
        std::cout << "Press 'c' to continue, 'q' to quit: ";
    }

    // Let them communicate
    std::cout << "\nBroadcasting messages between robots..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    manager.broadcast_message("Hello from robot network!");

    // Keep running for a bit to see messages
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "\nPress 'q' to quit: ";
    while (std::getline(std::cin, input) && input != "q") {
        std::cout << "Press 'q' to quit: ";
    }

    std::cout << "\nShutting down..." << std::endl;
    return 0;
}
