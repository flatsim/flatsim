#include "flatsim/robot/network/lan.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/if_tun.h>
#include <net/if.h>
#include <netinet/in.h>
#include <random>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

// LanInterface Implementation
LanInterface::LanInterface(const std::string &interface, uint16_t port, const std::string &ipv6_addr)
    : socket_fd_(-1), running_(false), owns_interface_(false), port_(port) {

    if (interface.empty()) {
        interface_name_ = "robot_auto";
        owns_interface_ = true;
    } else {
        interface_name_ = interface;
        // Check if interface already exists
        owns_interface_ = (if_nametoindex(interface.c_str()) == 0);
    }

    if (ipv6_addr.empty()) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1, 65535);
        int robot_id = dis(gen);
        ipv6_address_ = generate_robot_ipv6(robot_id);
    } else {
        ipv6_address_ = ipv6_addr;
    }
}

LanInterface::~LanInterface() { stop(); }

int LanInterface::create_tun_interface(const std::string &name) {
    int fd = open("/dev/net/tun", O_RDWR);
    if (fd < 0) {
        std::cerr << "Error opening /dev/net/tun: " << strerror(errno) << std::endl;
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI;
    strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);

    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        std::cerr << "Error creating TUN interface " << name << ": " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    if (ioctl(fd, TUNSETPERSIST, 1) < 0) {
        std::cerr << "Error making TUN persistent: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    std::cout << "Created persistent TUN interface: " << ifr.ifr_name << std::endl;

    std::string cmd = "ip link set " + std::string(ifr.ifr_name) + " up";
    if (system(cmd.c_str()) != 0) {
        std::cerr << "Failed to bring interface up" << std::endl;
    }

    return fd;
}

std::string LanInterface::generate_robot_ipv6(int robot_id) {
    char ipv6_buf[INET6_ADDRSTRLEN];
    snprintf(ipv6_buf, sizeof(ipv6_buf), "fd00:dead:beef::%04x", robot_id);
    return std::string(ipv6_buf);
}

bool LanInterface::setup_interface() {
    if (owns_interface_) {
        int tun_fd = create_tun_interface(interface_name_);
        if (tun_fd < 0) {
            std::cerr << "Failed to create interface, falling back to loopback" << std::endl;
            interface_name_ = "lo";
            owns_interface_ = false;
        } else {
            close(tun_fd);
        }
    }

    std::string cmd = "ip -6 addr add " + ipv6_address_ + "/64 dev " + interface_name_ + " 2>/dev/null";
    if (system(cmd.c_str()) != 0) {
        std::cerr << "Failed to add IPv6 address (try with sudo)" << std::endl;
    } else {
        std::cout << "Added IPv6 address " << ipv6_address_ << " to " << interface_name_ << std::endl;
    }

    return true;
}

bool LanInterface::start() {
    if (!setup_interface()) {
        return false;
    }

    socket_fd_ = socket(AF_INET6, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << ipv6_address_ << ": Failed to create socket" << std::endl;
        return false;
    }

    int v6only = 1;
    setsockopt(socket_fd_, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));
    
    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in6 addr = {};
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(port_);

    if (inet_pton(AF_INET6, ipv6_address_.c_str(), &addr.sin6_addr) == 1) {
        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
            std::cout << ipv6_address_ << " bound to [" << ipv6_address_ << "]:" << port_ << std::endl;
        } else {
            addr.sin6_addr = in6addr_any;
            if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                std::cerr << ipv6_address_ << ": Failed to bind to port " << port_ << std::endl;
                close(socket_fd_);
                return false;
            }
            std::cout << ipv6_address_ << " bound to [::] (any):" << port_ << std::endl;
        }
    }

    running_ = true;
    receive_thread_ = std::thread(&LanInterface::receive_loop, this);

    return true;
}

void LanInterface::stop() {
    running_ = false;
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (owns_interface_) {
        std::string cmd = "ip link del " + interface_name_ + " 2>/dev/null";
        system(cmd.c_str());
    } else {
        std::string cmd = "ip -6 addr del " + ipv6_address_ + "/64 dev " + interface_name_ + " 2>/dev/null";
        system(cmd.c_str());
    }
}

void LanInterface::send_message(const std::string &dest_addr, uint16_t dest_port, const std::string &msg) {
    if (socket_fd_ < 0) return;

    struct sockaddr_in6 dest = {};
    dest.sin6_family = AF_INET6;
    dest.sin6_port = htons(dest_port);

    if (inet_pton(AF_INET6, dest_addr.c_str(), &dest.sin6_addr) != 1) {
        std::cerr << ipv6_address_ << ": Invalid destination address" << std::endl;
        return;
    }

    ssize_t sent = sendto(socket_fd_, msg.c_str(), msg.length(), 0, (struct sockaddr *)&dest, sizeof(dest));

    if (sent > 0) {
        std::cout << ipv6_address_ << " sent: \"" << msg << "\" to [" << dest_addr << "]:" << dest_port << std::endl;
    }
}

void LanInterface::multicast_message(const std::string &msg) {
    if (socket_fd_ < 0) return;

    // Use IPv6 multicast address ff02::1 (all nodes multicast)
    struct sockaddr_in6 dest = {};
    dest.sin6_family = AF_INET6;
    dest.sin6_port = htons(port_);

    if (inet_pton(AF_INET6, "ff02::1", &dest.sin6_addr) != 1) {
        std::cerr << ipv6_address_ << ": Failed to set multicast address" << std::endl;
        return;
    }

    ssize_t sent = sendto(socket_fd_, msg.c_str(), msg.length(), 0, (struct sockaddr *)&dest, sizeof(dest));

    if (sent > 0) {
        std::cout << ipv6_address_ << " multicast: \"" << msg << "\" to all nodes" << std::endl;
    }
}

void LanInterface::multicast_to_group(const std::vector<std::string> &dest_addrs, uint16_t dest_port, const std::string &msg) {
    if (socket_fd_ < 0) return;

    std::cout << ipv6_address_ << " multicasting: \"" << msg << "\" to group [";
    for (size_t i = 0; i < dest_addrs.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << dest_addrs[i];
    }
    std::cout << "]" << std::endl;

    for (const auto &dest_addr : dest_addrs) {
        struct sockaddr_in6 dest = {};
        dest.sin6_family = AF_INET6;
        dest.sin6_port = htons(dest_port);

        if (inet_pton(AF_INET6, dest_addr.c_str(), &dest.sin6_addr) != 1) {
            std::cerr << ipv6_address_ << ": Invalid destination address " << dest_addr << std::endl;
            continue;
        }

        sendto(socket_fd_, msg.c_str(), msg.length(), 0, (struct sockaddr *)&dest, sizeof(dest));
    }
}

const std::string &LanInterface::get_ipv6() const { return ipv6_address_; }

uint16_t LanInterface::get_port() const { return port_; }

const std::string &LanInterface::get_interface() const { return interface_name_; }

void LanInterface::receive_loop() {
    char buffer[1024];
    struct sockaddr_in6 from;
    socklen_t from_len;

    while (running_) {
        from_len = sizeof(from);
        ssize_t received =
            recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);

        if (received > 0) {
            buffer[received] = '\0';
            char addr_str[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, &from.sin6_addr, addr_str, sizeof(addr_str));

            std::cout << ipv6_address_ << " received: \"" << buffer << "\" from [" << addr_str << "]:" << ntohs(from.sin6_port)
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}