#include "flatsim/network.hpp"
#include <stdexcept>

namespace fs {

    Network::Network() : initialized(false) {}

    Network::~Network() { cleanup(); }

    void Network::add_interface(std::unique_ptr<network::Interface> interface) {
        if (!interface) {
            throw std::invalid_argument("Cannot add null interface");
        }

        interfaces.push_back(std::move(interface));
    }

    void Network::remove_interface(const std::string &interface_type) {
        auto it = std::remove_if(interfaces.begin(), interfaces.end(),
                                 [&interface_type](const std::unique_ptr<network::Interface> &iface) {
                                     return iface && iface->get_type() == interface_type;
                                 });
        interfaces.erase(it, interfaces.end());
    }

    network::Interface *Network::get_interface(const std::string &interface_type) {
        for (auto &iface : interfaces) {
            if (iface && iface->get_type() == interface_type) {
                return iface.get();
            }
        }
        return nullptr;
    }

    bool Network::init(const std::string &robot_uuid) {
        if (initialized) {
            return false; // Already initialized
        }

        this->robot_uuid = robot_uuid;

        // Initialize all interfaces
        bool all_success = true;
        for (auto &iface : interfaces) {
            if (iface && !iface->init(robot_uuid)) {
                all_success = false;
            }
        }

        initialized = all_success;
        return initialized;
    }

    void Network::tick(float dt) {
        if (!initialized) {
            return;
        }

        // Update all interfaces
        for (auto &iface : interfaces) {
            if (iface) {
                iface->tick(dt);
            }
        }
    }

    void Network::cleanup() {
        if (!initialized) {
            return;
        }

        // Cleanup all interfaces
        for (auto &iface : interfaces) {
            if (iface) {
                iface->cleanup();
            }
        }

        interfaces.clear();
        initialized = false;
    }

    bool Network::is_ready() const {
        if (!initialized) {
            return false;
        }

        // Check if all interfaces are ready
        for (const auto &iface : interfaces) {
            if (iface && !iface->is_ready()) {
                return false;
            }
        }

        return true;
    }

    std::vector<std::string> Network::get_interface_types() const {
        std::vector<std::string> types;
        for (const auto &iface : interfaces) {
            if (iface) {
                types.push_back(iface->get_type());
            }
        }
        return types;
    }

    size_t Network::get_interface_count() const { return interfaces.size(); }

    bool Network::connect_to_peer(const std::string &peer_uuid) {
        if (!initialized) {
            return false;
        }

        bool any_success = false;
        for (auto &iface : interfaces) {
            if (iface && iface->connect_to_peer(peer_uuid)) {
                any_success = true;
            }
        }

        return any_success;
    }

    void Network::disconnect_from_peer(const std::string &peer_uuid) {
        if (!initialized) {
            return;
        }

        for (auto &iface : interfaces) {
            if (iface) {
                iface->disconnect_from_peer(peer_uuid);
            }
        }
    }

    std::vector<std::string> Network::get_connected_peers() const {
        std::vector<std::string> all_peers;

        if (!initialized) {
            return all_peers;
        }

        // Collect unique peers from all interfaces
        for (const auto &iface : interfaces) {
            if (iface) {
                auto iface_peers = iface->get_connected_peers();
                for (const auto &peer : iface_peers) {
                    // Add only if not already present
                    if (std::find(all_peers.begin(), all_peers.end(), peer) == all_peers.end()) {
                        all_peers.push_back(peer);
                    }
                }
            }
        }

        return all_peers;
    }

    void Network::send_all(const std::vector<uint8_t> &data) {
        if (!initialized) {
            return;
        }

        // Send via all interfaces
        for (auto &iface : interfaces) {
            if (iface) {
                iface->send_bytes(data);
            }
        }
    }

    void Network::send_via(const std::string &interface_type, const std::vector<uint8_t> &data) {
        if (!initialized) {
            return;
        }

        // Find interface by type and send
        for (auto &iface : interfaces) {
            if (iface && iface->get_type() == interface_type) {
                iface->send_bytes(data);
                return;
            }
        }
    }

    void Network::send_to_peer(const std::string &peer_uuid, const std::vector<uint8_t> &data) {
        if (!initialized) {
            return;
        }

        // Find which interface has this peer connected and use that one
        for (auto &iface : interfaces) {
            if (iface) {
                auto peers = iface->get_connected_peers();
                if (std::find(peers.begin(), peers.end(), peer_uuid) != peers.end()) {
                    iface->send_bytes_to_peer(peer_uuid, data);
                    return;
                }
            }
        }
    }

    std::vector<std::vector<uint8_t>> Network::receive() {
        std::vector<std::vector<uint8_t>> all_data;

        if (!initialized) {
            return all_data;
        }

        // Aggregate received data from all interfaces
        for (auto &iface : interfaces) {
            if (iface) {
                auto iface_data = iface->receive_bytes();
                all_data.insert(all_data.end(), iface_data.begin(), iface_data.end());
            }
        }

        return all_data;
    }

} // namespace fs