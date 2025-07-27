#include <cstring>
#include <iostream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
// #include <linux/if.h>  // Conflicts with net/if.h
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <net/if.h>
#include <netinet/in.h>
// #include <linux/ipv6.h>  // Conflicts with netinet headers

// Method 1: Create TUN/TAP virtual network interface (PERSISTENT)
int create_tun_interface(const std::string &name) {
    int fd = open("/dev/net/tun", O_RDWR);
    if (fd < 0) {
        std::cerr << "Error opening /dev/net/tun: " << strerror(errno) << std::endl;
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI; // TUN device, no packet info
    strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ);

    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        std::cerr << "Error creating TUN interface: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // Make it persistent
    if (ioctl(fd, TUNSETPERSIST, 1) < 0) {
        std::cerr << "Error making TUN persistent: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    std::cout << "Created persistent TUN interface: " << ifr.ifr_name << std::endl;

    // Bring interface up
    std::string cmd = "ip link set " + std::string(ifr.ifr_name) + " up";
    system(cmd.c_str());

    // Add IPv6 address
    cmd = "ip -6 addr add fd00:robot::1/64 dev " + std::string(ifr.ifr_name);
    system(cmd.c_str());

    std::cout << "Interface is UP with IPv6 address fd00:robot::1/64" << std::endl;

    return fd;
}

// Method 2: Create virtual ethernet pair (requires root)
bool create_veth_pair(const std::string &veth1, const std::string &veth2) {
    // This would typically use netlink sockets (libnl) or system() calls
    std::string cmd = "ip link add " + veth1 + " type veth peer name " + veth2;
    int result = system(cmd.c_str());
    return result == 0;
}

// Method 3: Assign IPv6 address to interface
bool assign_ipv6_address(const std::string &interface, const std::string &ipv6_addr) {
    int sock = socket(AF_INET6, SOCK_DGRAM, 0);
    if (sock < 0) return false;

    struct ifreq ifr;
    struct sockaddr_in6 sai;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);

    memset(&sai, 0, sizeof(sai));
    sai.sin6_family = AF_INET6;
    sai.sin6_port = 0;

    // Convert IPv6 string to binary form
    if (inet_pton(AF_INET6, ipv6_addr.c_str(), &sai.sin6_addr) != 1) {
        close(sock);
        return false;
    }

    memcpy(&ifr.ifr_addr, &sai, sizeof(sai));

    // This would need SIOCSIFADDR ioctl (requires root)
    // For demo purposes, we'll use system command
    close(sock);

    std::string cmd = "ip -6 addr add " + ipv6_addr + "/64 dev " + interface;
    return system(cmd.c_str()) == 0;
}

// Method 4: Create IPv6 socket bound to specific address
int create_ipv6_socket(const std::string &ipv6_addr, uint16_t port) {
    int sock = socket(AF_INET6, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
        return -1;
    }

    struct sockaddr_in6 addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(port);

    if (inet_pton(AF_INET6, ipv6_addr.c_str(), &addr.sin6_addr) != 1) {
        std::cerr << "Invalid IPv6 address: " << ipv6_addr << std::endl;
        close(sock);
        return -1;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind to " << ipv6_addr << ":" << port << " - " << strerror(errno) << std::endl;
        close(sock);
        return -1;
    }

    std::cout << "Socket bound to [" << ipv6_addr << "]:" << port << std::endl;
    return sock;
}

// Generate unique IPv6 addresses for robots
std::string generate_robot_ipv6(int robot_id) {
    // Using link-local addresses (fe80::/10) or unique local addresses (fd00::/8)
    char ipv6_buf[INET6_ADDRSTRLEN];
    snprintf(ipv6_buf, sizeof(ipv6_buf), "fd00:dead:beef::%04x", robot_id);
    return std::string(ipv6_buf);
}

int main() {
    std::cout << "=== IPv6 Network Interface Demo ===" << std::endl;

    // Example 1: Generate IPv6 addresses for robots
    std::cout << "\n1. Generating IPv6 addresses for robots:" << std::endl;
    for (int i = 1; i <= 5; i++) {
        std::string ipv6 = generate_robot_ipv6(i);
        std::cout << "   Robot " << i << ": " << ipv6 << std::endl;
    }

    // Example 2: Create TUN interface (requires /dev/net/tun access)
    std::cout << "\n2. Creating TUN interface:" << std::endl;
    int tun_fd = create_tun_interface("robot_tun0");
    if (tun_fd >= 0) {
        std::cout << "   Success! File descriptor: " << tun_fd << std::endl;
        std::cout << "   Check with: ip addr show robot_tun0" << std::endl;

        std::cout << "\nPress 'q' then Enter to remove interface and exit: ";
        std::string input;
        while (std::getline(std::cin, input) && input != "q") {
            std::cout << "Type 'q' to quit: ";
        }

        // Remove persistence before closing
        ioctl(tun_fd, TUNSETPERSIST, 0);
        close(tun_fd);
    }

    // Example 3: Create IPv6 socket (no root required)
    std::cout << "\n3. Creating IPv6 sockets:" << std::endl;
    int sock = create_ipv6_socket("::1", 8080); // localhost IPv6
    if (sock >= 0) {
        std::cout << "   Success! Socket fd: " << sock << std::endl;
        close(sock);
    }


    return 0;
}
