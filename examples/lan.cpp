#include "flatsim/robot/network/lan.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

// Global flag for graceful shutdown
std::atomic<bool> should_exit{false};

void signal_handler(int signal) {
    std::cout << "\nCaught signal " << signal << ", requesting shutdown..." << std::endl;
    should_exit = true;
}

int main() {
    std::cout << "=== LAN Interface P2P & Multicast Demo ===\n" << std::endl;

    // Setup signal handler for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    LanInterface robot1("flat1", 8000);
    LanInterface robot2("flat2", 8000);
    LanInterface robot3("flat3", 8000);
    LanInterface robot4("flat4", 8000);

    if (!robot1.start()) {
        std::cerr << "Failed to start robot1" << std::endl;
        return 1;
    }
    if (!robot2.start()) {
        std::cerr << "Failed to start robot2" << std::endl;
        return 1;
    }
    if (!robot3.start()) {
        std::cerr << "Failed to start robot3" << std::endl;
        return 1;
    }
    if (!robot4.start()) {
        std::cerr << "Failed to start robot4" << std::endl;
        return 1;
    }

    std::cout << "\nActive robots:" << std::endl;
    std::cout << "  - " << robot1.get_interface() << " IPv6: [" << robot1.get_ipv6() << "]:" << robot1.get_port()
              << std::endl;
    std::cout << "  - " << robot2.get_interface() << " IPv6: [" << robot2.get_ipv6() << "]:" << robot2.get_port()
              << std::endl;
    std::cout << "  - " << robot3.get_interface() << " IPv6: [" << robot3.get_ipv6() << "]:" << robot3.get_port()
              << std::endl;
    std::cout << "  - " << robot4.get_interface() << " IPv6: [" << robot4.get_ipv6() << "]:" << robot4.get_port()
              << std::endl;

    std::cout << "\nPress 'c' to continue with multicast demo, 'q' to quit: ";

    std::string input;
    while (std::getline(std::cin, input) && !should_exit) {
        if (input == "q") {
            break;
        } else if (input == "c") {
            break;
        }
        std::cout << "Press 'c' to continue, 'q' to quit: ";
    }

    if (should_exit) {
        std::cout << "\nShutting down..." << std::endl;
        return 0;
    }

    std::cout << "\n=== Multicast Communication Demo ===" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Demo 1: Robot1 multicasts to all robots using IPv6 multicast
    std::cout << "\n--- Demo 1: Global Multicast ---" << std::endl;
    robot1.multicast_message("Global announcement from Tractor-1!");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Demo 2: Robot1 multicasts to specific group (robot2 and robot3)
    std::cout << "\n--- Demo 2: Group Multicast ---" << std::endl;
    std::vector<std::string> group_harvest = {robot2.get_ipv6(), robot3.get_ipv6()};
    robot1.multicast_to_group(group_harvest, robot2.get_port(), "Harvest coordination from Tractor-1!");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Demo 3: Robot2 multicasts to other robots (robot3 and robot4)
    std::cout << "\n--- Demo 3: Another Group Multicast ---" << std::endl;
    std::vector<std::string> group_support = {robot3.get_ipv6(), robot4.get_ipv6()};
    robot2.multicast_to_group(group_support, robot3.get_port(), "Support request from Harvester-1!");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Demo 4: Traditional P2P messages
    std::cout << "\n--- Demo 4: Traditional P2P Messages ---" << std::endl;
    robot3.send_message(robot4.get_ipv6(), robot4.get_port(), "Direct message from Sprayer-1!");
    robot4.send_message(robot1.get_ipv6(), robot1.get_port(), "Task complete from Feeder-1!");

    // Keep running until signal or timeout
    auto start_time = std::chrono::steady_clock::now();
    while (!should_exit && std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!should_exit) {
        std::cout << "\nPress 'q' to quit: ";
        while (std::getline(std::cin, input) && input != "q" && !should_exit) {
            std::cout << "Press 'q' to quit: ";
        }
    }

    std::cout << "\nShutting down..." << std::endl;
    return 0;
}
