#include "flatsim/robot/systems/chain.hpp"
#include "flatsim/robot.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <thread>
#include <chrono>
#include <spdlog/spdlog.h>

namespace fs {

// Helper function to calculate overlap between two rectangular hitches
static float calculate_hitch_overlap_percentage(const Hitch& hitch1, const Hitch& hitch2) {
    // Get the corners of both hitches in world coordinates
    auto corners1 = hitch1.get_corners();
    auto corners2 = hitch2.get_corners();
    
    // Calculate bounding boxes for both hitches
    float min_x1 = static_cast<float>(corners1[0].x), max_x1 = static_cast<float>(corners1[0].x);
    float min_y1 = static_cast<float>(corners1[0].y), max_y1 = static_cast<float>(corners1[0].y);
    float min_x2 = static_cast<float>(corners2[0].x), max_x2 = static_cast<float>(corners2[0].x);
    float min_y2 = static_cast<float>(corners2[0].y), max_y2 = static_cast<float>(corners2[0].y);
    
    for (const auto& corner : corners1) {
        min_x1 = std::min(min_x1, static_cast<float>(corner.x));
        max_x1 = std::max(max_x1, static_cast<float>(corner.x));
        min_y1 = std::min(min_y1, static_cast<float>(corner.y));
        max_y1 = std::max(max_y1, static_cast<float>(corner.y));
    }
    
    for (const auto& corner : corners2) {
        min_x2 = std::min(min_x2, static_cast<float>(corner.x));
        max_x2 = std::max(max_x2, static_cast<float>(corner.x));
        min_y2 = std::min(min_y2, static_cast<float>(corner.y));
        max_y2 = std::max(max_y2, static_cast<float>(corner.y));
    }
    
    // Calculate intersection area
    float intersection_x1 = std::max(min_x1, min_x2);
    float intersection_y1 = std::max(min_y1, min_y2);
    float intersection_x2 = std::min(max_x1, max_x2);
    float intersection_y2 = std::min(max_y1, max_y2);
    
    // Check if there's any intersection
    if (intersection_x1 >= intersection_x2 || intersection_y1 >= intersection_y2) {
        return 0.0f; // No overlap
    }
    
    float intersection_area = (intersection_x2 - intersection_x1) * (intersection_y2 - intersection_y1);
    
    // Calculate areas of both hitches
    float area1 = (max_x1 - min_x1) * (max_y1 - min_y1);
    float area2 = (max_x2 - min_x2) * (max_y2 - min_y2);
    
    // Calculate overlap percentage relative to the smaller hitch
    float smaller_area = std::min(area1, area2);
    if (smaller_area <= 0.0f) return 0.0f;
    
    return (intersection_area / smaller_area) * 100.0f;
}

void ChainManager::add_follower(Robot* follower, muli::RevoluteJoint* joint) {
    connected_followers.push_back(follower);
    connection_joints.push_back(joint);
    
    spdlog::info("Connected {} as follower to {}", follower->info.name, robot->info.name);
}

bool ChainManager::try_connect_nearby_slave(const std::vector<std::shared_ptr<Robot>>& all_robots) {
    // Only MASTER robots can initiate connections
    if (robot->role != RobotRole::MASTER || !robot->chassis) {
        return false;
    }
    
    // Already connected (check if we have any followers)
    if (!connected_followers.empty()) {
        return false;
    }
    
    const float minimum_overlap_percentage = 50.0f; // Require at least 50% overlap
    
    for (const auto &other_robot : all_robots) {
        // Skip self and non-slaves
        if (other_robot.get() == robot || other_robot->role != RobotRole::SLAVE) {
            continue;
        }
        
        // Skip if other robot has no chassis or hitches
        if (!other_robot->chassis || other_robot->chassis->hitches.empty()) {
            continue;
        }
        
        // Try to connect my master hitches to their slave hitches
        for (const auto &my_hitch : robot->chassis->hitches) {
            for (const auto &other_hitch : other_robot->chassis->hitches) {
                // Only connect master hitch to slave hitch
                if (!my_hitch.is_master || other_hitch.is_master) {
                    continue;
                }
                
                // Calculate overlap percentage between hitches
                float overlap_percentage = calculate_hitch_overlap_percentage(my_hitch, other_hitch);
                
                if (overlap_percentage >= minimum_overlap_percentage) {
                    // Get actual world positions of hitches (updated by tick)
                    concord::Point my_hitch_pos = my_hitch.pose.point;
                    concord::Point other_hitch_pos = other_hitch.pose.point;
                    
                    // Use the midpoint between the two hitch points as the joint position
                    muli::Vec2 hitch_world_pos((my_hitch_pos.x + other_hitch_pos.x) / 2.0f,
                                               (my_hitch_pos.y + other_hitch_pos.y) / 2.0f);
                    
                    auto new_joint = robot->world->CreateRevoluteJoint(robot->chassis->get_body(), 
                                                                       other_robot->chassis->get_body(), 
                                                                       hitch_world_pos,
                                                                       20.0f, // frequency
                                                                       0.8f,  // damping
                                                                       10.0f  // joint mass
                    );
                    
                    if (new_joint) {
                        // Add to followers using new system
                        add_follower(other_robot.get(), new_joint);
                        other_robot->chain_manager->set_master_robot(robot);
                        other_robot->role = RobotRole::FOLLOWER; // Change slave to follower
                        
                        // Follower adopts master's color
                        other_robot->update_color(robot->info.color);
                        
                        // Update capabilities for both robots
                        update_follower_capabilities();
                        other_robot->update_follower_capabilities();
                        
                        spdlog::info("Connected {} to {} (hitch overlap: {:.1f}%)", robot->info.name, 
                                     other_robot->info.name, overlap_percentage);
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}

bool ChainManager::try_connect_nearby() {
    // Only MASTER robots or FOLLOWERS with available master hitches can initiate connections
    if ((robot->role != RobotRole::MASTER && robot->role != RobotRole::FOLLOWER) || !robot->chassis) {
        return false;
    }
    
    // Check if this robot has any available master hitch
    bool has_available_master_hitch = false;
    for (const auto &hitch : robot->chassis->hitches) {
        if (hitch.is_master) {
            // Check if this hitch is already used
            bool hitch_used = false;
            for (size_t i = 0; i < connected_followers.size(); ++i) {
                // This is a simplified check - in reality we'd need to track which hitch was used
                // For now, assume only one master hitch per robot
                hitch_used = true;
                break;
            }
            if (!hitch_used) {
                has_available_master_hitch = true;
                break;
            }
        }
    }
    
    if (!has_available_master_hitch) {
        return false;
    }
    
    auto all_robots = robot->get_all_robots();
    const float minimum_overlap_percentage = 50.0f; // Require at least 50% overlap
    
    for (Robot *other_robot : all_robots) {
        // Skip self and non-slaves
        if (other_robot == robot || other_robot->role != RobotRole::SLAVE) {
            continue;
        }
        
        // Skip if other robot has no chassis or hitches
        if (!other_robot->chassis || other_robot->chassis->hitches.empty()) {
            continue;
        }
        
        // Try to connect my master hitches to their slave hitches
        for (const auto &my_hitch : robot->chassis->hitches) {
            for (const auto &other_hitch : other_robot->chassis->hitches) {
                // Only connect master hitch to slave hitch
                if (!my_hitch.is_master || other_hitch.is_master) {
                    continue;
                }
                
                // Calculate overlap percentage between hitches
                float overlap_percentage = calculate_hitch_overlap_percentage(my_hitch, other_hitch);
                
                if (overlap_percentage >= minimum_overlap_percentage) {
                    // Get actual world positions of hitches (updated by tick)
                    concord::Point my_hitch_pos = my_hitch.pose.point;
                    concord::Point other_hitch_pos = other_hitch.pose.point;
                    
                    // Use the midpoint between the two hitch points as the joint position
                    muli::Vec2 hitch_world_pos((my_hitch_pos.x + other_hitch_pos.x) / 2.0f,
                                               (my_hitch_pos.y + other_hitch_pos.y) / 2.0f);
                    
                    auto new_joint = robot->world->CreateRevoluteJoint(robot->chassis->get_body(),
                                                                       other_robot->chassis->get_body(), 
                                                                       hitch_world_pos,
                                                                       20.0f, // frequency
                                                                       0.8f,  // damping
                                                                       10.0f  // joint mass
                    );
                    
                    if (new_joint) {
                        // Add to followers list
                        add_follower(other_robot, new_joint);
                        
                        // Set backward reference
                        other_robot->chain_manager->set_master_robot(robot);
                        other_robot->role = RobotRole::FOLLOWER;
                        
                        // Follower adopts master's color
                        other_robot->update_color(robot->info.color);
                        
                        // Update capabilities for both robots
                        update_follower_capabilities();
                        other_robot->update_follower_capabilities();
                        
                        spdlog::info("Connected {} to {} (hitch overlap: {:.1f}%)", robot->info.name,
                                     other_robot->info.name, overlap_percentage);
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}

bool ChainManager::try_connect_from_chain_end() {
    // Find the end of the chain starting from this robot
    Robot* chain_end = robot;
    
    // Follow the chain to the end (robot with no followers)
    while (!chain_end->chain_manager->connected_followers.empty()) {
        chain_end = chain_end->chain_manager->connected_followers.back();  // Follow the last follower
    }
    
    // Try to connect from the chain end
    spdlog::info("Trying to connect from chain end: {}", chain_end->info.name);
    return chain_end->try_connect_nearby();
}

void ChainManager::disconnect_trailer() {
    disconnect_all_followers();
}

void ChainManager::disconnect_all_followers() {
    for (size_t i = 0; i < connected_followers.size(); ++i) {
        Robot* follower = connected_followers[i];
        muli::RevoluteJoint* joint = connection_joints[i];
        
        if (joint) {
            robot->world->Destroy(joint);
        }
        
        if (follower) {
            follower->role = RobotRole::SLAVE;  // Change back to slave
            follower->chain_manager->master_robot = nullptr;
            
            // Restore original color when disconnecting
            follower->update_color(follower->original_color);
            
            spdlog::info("Disconnected {} from {}", follower->info.name, robot->info.name);
            
            // Recursively disconnect any sub-followers
            follower->disconnect_all_followers();
            
            // Update capabilities after disconnection
            follower->update_follower_capabilities();
        }
    }
    
    connected_followers.clear();
    connection_joints.clear();
    
    // Update our own capabilities after disconnection
    update_follower_capabilities();
}

void ChainManager::disconnect_last_follower() {
    // Find the end of the chain and disconnect the last follower
    Robot* chain_end = robot;
    Robot* previous_robot = nullptr;
    
    // Traverse to the end of the chain
    while (!chain_end->chain_manager->connected_followers.empty()) {
        previous_robot = chain_end;
        chain_end = chain_end->chain_manager->connected_followers.back();  // Follow the chain
    }
    
    // If we found a chain end that isn't ourselves, disconnect it from its master
    if (chain_end != robot && previous_robot) {
        // Find the connection to disconnect
        auto it = std::find(previous_robot->chain_manager->connected_followers.begin(), 
                           previous_robot->chain_manager->connected_followers.end(), 
                           chain_end);
        
        if (it != previous_robot->chain_manager->connected_followers.end()) {
            size_t index = std::distance(previous_robot->chain_manager->connected_followers.begin(), it);
            
            // Destroy the physics joint
            if (index < previous_robot->chain_manager->connection_joints.size()) {
                muli::RevoluteJoint* joint = previous_robot->chain_manager->connection_joints[index];
                if (joint) {
                    previous_robot->world->Destroy(joint);
                }
                previous_robot->chain_manager->connection_joints.erase(previous_robot->chain_manager->connection_joints.begin() + index);
            }
            
            // Remove from followers list
            previous_robot->chain_manager->connected_followers.erase(it);
            
            // Reset disconnected robot's state
            chain_end->role = RobotRole::SLAVE;
            chain_end->chain_manager->master_robot = nullptr;
            
            // Restore original color when disconnecting
            chain_end->update_color(chain_end->original_color);
            
            chain_end->update_follower_capabilities();
            
            // Update master's capabilities
            previous_robot->update_follower_capabilities();
            
            spdlog::info("Disconnected last follower {} from {}", 
                       chain_end->info.name, previous_robot->info.name);
        }
    }
}

bool ChainManager::is_connected() const {
    return !connected_followers.empty() || master_robot != nullptr;
}

Robot* ChainManager::get_root_master() const {
    const Robot* current = robot;
    while (current->chain_manager->master_robot != nullptr) {
        current = current->chain_manager->master_robot;
    }
    return const_cast<Robot*>(current);
}

std::vector<Robot*> ChainManager::get_full_chain() const {
    std::vector<Robot*> chain;
    
    // Start from root master
    Robot* root = get_root_master();
    chain.push_back(root);
    
    // Follow the chain adding all followers
    std::function<void(Robot*)> add_followers = [&](Robot* current) {
        for (Robot* follower : current->chain_manager->connected_followers) {
            chain.push_back(follower);
            add_followers(follower);  // Recursively add sub-followers
        }
    };
    
    add_followers(root);
    return chain;
}

int ChainManager::get_chain_length() const {
    return static_cast<int>(get_full_chain().size());
}

int ChainManager::get_position_in_chain() const {
    auto chain = get_full_chain();
    for (int i = 0; i < static_cast<int>(chain.size()); ++i) {
        if (chain[i] == robot) {
            return i;
        }
    }
    return -1;  // Not found (shouldn't happen)
}

void ChainManager::print_chain_status() const {
    auto chain = get_full_chain();
    std::cout << "Chain from " << get_root_master()->info.name << " (length=" << chain.size() << "): ";
    for (int i = 0; i < static_cast<int>(chain.size()); ++i) {
        std::cout << i << ":" << chain[i]->info.name << "("
                  << (chain[i]->role == RobotRole::MASTER     ? "M"
                      : chain[i]->role == RobotRole::FOLLOWER ? "F"
                                                              : "S")
                  << ")";
        if (i < static_cast<int>(chain.size()) - 1)
            std::cout << " -> ";
    }
    std::cout << std::endl;
}

void ChainManager::update_follower_capabilities() {
    // Reset capabilities
    follower_capabilities.has_steering = false;
    follower_capabilities.has_throttle = false;
    follower_capabilities.has_tank = robot->has_tank();
    follower_capabilities.has_additional_hitches = false;
    follower_capabilities.available_master_hitches.clear();
    
    // Check for steering capability
    for (const auto &max_angle : robot->control_system->get_steerings_max()) {
        if (max_angle > 0) {
            follower_capabilities.has_steering = true;
            break;
        }
    }
    
    // Check for throttle capability
    for (const auto &max_throttle : robot->control_system->get_throttles_max()) {
        if (max_throttle > 0) {
            follower_capabilities.has_throttle = true;
            break;
        }
    }
    
    // Check for available master hitches (for chaining)
    if (robot->chassis) {
        for (const auto &hitch : robot->chassis->hitches) {
            if (hitch.is_master) {
                // Better hitch occupation detection
                bool hitch_occupied = false;
                
                // Check the specific hitch - if it's a rear master hitch on a trailer,
                // it should be available even if the trailer is a follower
                if (hitch.name == "rear_hitch" && robot->info.type == "trailer") {
                    // Rear hitch on trailer - available unless we already have a follower
                    hitch_occupied = (connected_followers.size() >= 1);
                } else {
                    // For other hitches (like tractor rear_hitch), use simpler logic
                    hitch_occupied = (connected_followers.size() >= 1);
                }
                
                if (!hitch_occupied) {
                    follower_capabilities.available_master_hitches.push_back(hitch.name);
                    follower_capabilities.has_additional_hitches = true;
                }
            }
        }
    }
    
    // Log capabilities for debugging
    spdlog::debug("{} capabilities: steering={}, throttle={}, tank={}, master_hitches={}", robot->info.name,
                  follower_capabilities.has_steering, follower_capabilities.has_throttle,
                  follower_capabilities.has_tank, follower_capabilities.available_master_hitches.size());
}

void ChainManager::disconnect_at_position(int position) {
    // Position 1 = first follower, 2 = second follower, etc.
    if (position < 1) {
        spdlog::warn("Invalid position {} for disconnection (must be >= 1)", position);
        return;
    }
    
    auto chain = get_full_chain();
    if (position >= static_cast<int>(chain.size())) {
        spdlog::warn("Position {} is beyond chain length {}", position, chain.size());
        return;
    }
    
    Robot* robot_to_disconnect = chain[position];
    Robot* its_master = robot_to_disconnect->chain_manager->master_robot;
    
    if (!its_master) {
        spdlog::warn("Robot at position {} has no master (might be root)", position);
        return;
    }
    
    // Find the connection in the master's followers list
    auto it = std::find(its_master->chain_manager->connected_followers.begin(), 
                       its_master->chain_manager->connected_followers.end(), 
                       robot_to_disconnect);
    
    if (it != its_master->chain_manager->connected_followers.end()) {
        size_t index = std::distance(its_master->chain_manager->connected_followers.begin(), it);
        
        // Destroy the physics joint
        if (index < its_master->chain_manager->connection_joints.size()) {
            muli::RevoluteJoint* joint = its_master->chain_manager->connection_joints[index];
            if (joint) {
                its_master->world->Destroy(joint);
            }
            its_master->chain_manager->connection_joints.erase(its_master->chain_manager->connection_joints.begin() + index);
        }
        
        // Remove from followers list
        its_master->chain_manager->connected_followers.erase(it);
        
        // Reset disconnected robot's state (this will also disconnect its followers)
        robot_to_disconnect->role = RobotRole::SLAVE;
        robot_to_disconnect->chain_manager->master_robot = nullptr;
        
        // Restore original color when disconnecting
        robot_to_disconnect->update_color(robot_to_disconnect->original_color);
        
        robot_to_disconnect->disconnect_all_followers();  // Disconnect everything after this point
        robot_to_disconnect->update_follower_capabilities();
        
        // Update master's capabilities
        its_master->update_follower_capabilities();
        
        spdlog::info("Disconnected robot at position {} ({}) from {}", 
                   position, robot_to_disconnect->info.name, its_master->info.name);
    }
}

void ChainManager::disconnect_from_position(int position) {
    // Disconnect everything from position onwards
    if (position < 1) {
        spdlog::warn("Invalid position {} for disconnection (must be >= 1)", position);
        return;
    }
    
    auto chain = get_full_chain();
    if (position >= static_cast<int>(chain.size())) {
        spdlog::warn("Position {} is beyond chain length {}", position, chain.size());
        return;
    }
    
    // Just disconnect at the position - the disconnect_all_followers() call in disconnect_at_position
    // will handle disconnecting everything after that point
    disconnect_at_position(position);
    
    spdlog::info("Disconnected chain from position {} onwards", position);
}

void ChainManager::break_chain_for_teleport() {
    spdlog::info("Breaking chain and teleporting all robots to spawn for robot {}", robot->info.name);
    
    // Get the root master and the full chain
    Robot* root_master = get_root_master();
    auto full_chain = root_master->chain_manager->get_full_chain();
    
    spdlog::info("Full chain has {} robots, starting from root {}", full_chain.size(), root_master->info.name);
    
    // Step 1: Disconnect all connections in the entire chain
    root_master->chain_manager->disconnect_all_followers();
    
    // Step 2: Teleport each robot individually to its spawn position
    for (Robot* chain_robot : full_chain) {
        if (chain_robot != robot) {  // Don't teleport the initiating robot here
            spdlog::info("Teleporting chain robot {} to its spawn position", chain_robot->info.name);
            chain_robot->teleport(chain_robot->get_spawn_position(), false);  // No propagation
            
            // Small delay to prevent physics conflicts
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

} // namespace fs