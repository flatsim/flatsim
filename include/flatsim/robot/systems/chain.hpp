#pragma once

#include "flatsim/types.hpp"
#include "muli/world.h"
#include <vector>
#include <memory>

namespace fs {
    // Forward declarations
    class Robot;
    
    // Follower capabilities structure
    struct FollowerCapabilities {
        bool has_steering = false;
        bool has_throttle = false;
        bool has_tank = false;
        bool has_additional_hitches = false;
        std::vector<std::string> available_master_hitches;
    };

    // ============================================================================
    // ChainManager - Handles all connection/disconnection and chain management
    // ============================================================================
    class ChainManager {
    private:
        Robot* robot;
        std::vector<Robot*> connected_followers;
        std::vector<muli::RevoluteJoint*> connection_joints;
        Robot* master_robot = nullptr;
        FollowerCapabilities follower_capabilities;

    public:
        ChainManager(Robot* r) : robot(r) {}
        
        // Connection management
        bool try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>>& all_robots);
        bool try_connect_nearby();
        bool try_connect_from_chain_end();
        
        // Disconnection management
        void disconnect_trailer();
        void disconnect_all_followers();
        void disconnect_last_follower();
        void disconnect_at_position(int position);
        void disconnect_from_position(int position);
        
        // Chain queries
        bool is_connected() const;
        std::vector<Robot*> get_connected_followers() const { return connected_followers; }
        Robot* get_master_robot() const { return master_robot; }
        bool is_follower() const { return master_robot != nullptr; }
        Robot* get_root_master() const;
        std::vector<Robot*> get_full_chain() const;
        int get_chain_length() const;
        int get_position_in_chain() const;
        void print_chain_status() const;
        
        // Capability management
        void update_follower_capabilities();
        const FollowerCapabilities& get_follower_capabilities() const { return follower_capabilities; }
        bool has_steering_capability() const { return follower_capabilities.has_steering; }
        bool has_throttle_capability() const { return follower_capabilities.has_throttle; }
        bool has_available_master_hitches() const { return follower_capabilities.has_additional_hitches; }
        
        // Teleport handling - break connections
        void break_chain_for_teleport();
        
        // Allow Robot to set master relationship
        void set_master_robot(Robot* master) { master_robot = master; }
        void add_follower(Robot* follower, muli::RevoluteJoint* joint);
    };

} // namespace fs