#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

class LanInterface {
  private:
    std::string interface_name_;
    std::string ipv6_address_;
    uint16_t port_;
    int socket_fd_;
    std::thread receive_thread_;
    bool running_;
    bool owns_interface_;

    int create_tun_interface(const std::string &name);
    std::string generate_robot_ipv6(int robot_id);
    bool setup_interface();

  public:
    LanInterface(const std::string &interface = "", uint16_t port = 8000, const std::string &ipv6_addr = "");
    ~LanInterface();

    bool start();
    void stop();
    void send_message(const std::string &dest_addr, uint16_t dest_port, const std::string &msg);
    void multicast_message(const std::string &msg);
    void multicast_to_group(const std::vector<std::string> &dest_addrs, uint16_t dest_port, const std::string &msg);
    const std::string &get_ipv6() const;
    uint16_t get_port() const;
    const std::string &get_interface() const;

  private:
    void receive_loop();
};