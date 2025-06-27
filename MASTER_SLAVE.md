# Master/Slave/Follower Control System Design

## Overview

This document outlines the implementation plan for a sophisticated control system where machines can connect in chains, with masters controlling followers, and followers potentially acting as masters for subsequent connections.

## Core Concepts

### 1. Role Transitions
- **SLAVE**: A machine that can be connected to (has slave hitches)
- **MASTER**: A machine that can connect to others (has master hitches)
- **FOLLOWER**: A slave that has been connected to a master (transitional state)

### 2. Connection Chain
```
Tractor (MASTER) -> Trailer1 (SLAVE->FOLLOWER) -> Trailer2 (SLAVE->FOLLOWER) -> ...
```

## Architecture Design

### 1. Connection Management

#### Current State
- `Robot::connected_slave` - single pointer to connected robot
- `Robot::connection_joint` - physics joint between robots
- Connection happens via overlapping hitch points

#### Proposed Changes
```cpp
class Robot {
    // Replace single connected_slave with a vector
    std::vector<Robot*> connected_followers;
    std::vector<muli::Joint*> connection_joints;
    
    // Backward reference to master (if this robot is a follower)
    Robot* master_robot = nullptr;
    
    // Exposed capabilities when acting as follower
    struct FollowerCapabilities {
        bool has_steering = false;
        bool has_throttle = false;
        bool has_tank = false;
        bool has_additional_hitches = false;
        std::vector<std::string> available_master_hitches;
    };
    FollowerCapabilities follower_capabilities;
};
```

### 2. Control Propagation System

#### Tick/Update Propagation
```cpp
void Robot::tick(float dt) {
    // ... existing tick logic ...
    
    // Propagate tick to all followers
    for (Robot* follower : connected_followers) {
        follower->tick_as_follower(dt, *this);
    }
}

void Robot::tick_as_follower(float dt, Robot& master) {
    // Update sensors with master-relative positioning
    // Update physics (already handled by joint)
    // Process any follower-specific logic
    
    // Continue chain - this follower might have its own followers
    for (Robot* sub_follower : connected_followers) {
        sub_follower->tick_as_follower(dt, *this);
    }
}
```

#### Control Input Propagation
```cpp
void Robot::set_angular(float angular) {
    // ... existing steering logic ...
    
    // Propagate to followers that have steering
    for (Robot* follower : connected_followers) {
        if (follower->follower_capabilities.has_steering) {
            follower->set_angular_as_follower(angular, *this);
        }
    }
}

void Robot::set_linear(float linear) {
    // ... existing throttle logic ...
    
    // Propagate to followers that have throttle
    for (Robot* follower : connected_followers) {
        if (follower->follower_capabilities.has_throttle) {
            follower->set_linear_as_follower(linear, *this);
        }
    }
}
```

### 3. Connection Chain Management

#### Connection Process
```cpp
bool Robot::try_connect_nearby() {
    // 1. Find compatible robot (existing logic)
    // 2. Create physics joint (existing logic)
    // 3. NEW: Establish bidirectional references
    connected_followers.push_back(other_robot);
    connection_joints.push_back(joint);
    other_robot->master_robot = this;
    
    // 4. NEW: Update follower capabilities
    other_robot->update_follower_capabilities();
    
    // 5. NEW: Check if follower can act as master for further connections
    if (other_robot->has_master_hitches_available()) {
        // Follower retains ability to connect to more robots
    }
}
```

#### Disconnection Process
```cpp
void Robot::disconnect_trailer() {
    // Disconnect ALL followers (or just the last one?)
    for (size_t i = 0; i < connected_followers.size(); ++i) {
        Robot* follower = connected_followers[i];
        muli::Joint* joint = connection_joints[i];
        
        // Destroy physics joint
        world->Destroy(joint);
        
        // Reset follower state
        follower->role = RobotRole::SLAVE;
        follower->master_robot = nullptr;
        
        // Recursively disconnect any sub-followers
        follower->disconnect_all_followers();
    }
    
    connected_followers.clear();
    connection_joints.clear();
}
```

### 4. Follower Capability System

#### Capability Detection
```cpp
void Robot::update_follower_capabilities() {
    follower_capabilities.has_steering = false;
    follower_capabilities.has_throttle = false;
    follower_capabilities.has_tank = tank.has_value();
    follower_capabilities.available_master_hitches.clear();
    
    // Check for steering capability
    for (const auto& max_angle : steerings_max) {
        if (max_angle > 0) {
            follower_capabilities.has_steering = true;
            break;
        }
    }
    
    // Check for throttle capability
    for (const auto& max_throttle : throttles_max) {
        if (max_throttle > 0) {
            follower_capabilities.has_throttle = true;
            break;
        }
    }
    
    // Check for available master hitches (for chaining)
    for (const auto& hitch : chassis->hitches) {
        if (hitch.is_master && !is_hitch_occupied(hitch)) {
            follower_capabilities.available_master_hitches.push_back(hitch.name);
            follower_capabilities.has_additional_hitches = true;
        }
    }
}
```

### 5. State Synchronization

#### Unified State Management
```cpp
struct RobotChainState {
    Robot* root_master;
    std::vector<Robot*> all_robots_in_chain;
    float total_length;
    float total_mass;
    bool has_steering_followers;
    bool has_powered_followers;
};

RobotChainState Robot::get_chain_state() {
    RobotChainState state;
    state.root_master = get_root_master();
    collect_all_chain_members(state.all_robots_in_chain);
    // ... calculate totals ...
    return state;
}
```

## Implementation Phases

### Phase 1: Basic Chain Connection
1. Replace single `connected_slave` with `connected_followers` vector
2. Add `master_robot` backward reference
3. Update connection/disconnection logic to handle multiple followers
4. Ensure physics joints work correctly in chains

### Phase 2: Control Propagation
1. Implement `tick_as_follower` method
2. Add steering/throttle propagation for followers with capabilities
3. Test with simple tractor->trailer->trailer chain

### Phase 3: Follower Capabilities
1. Implement capability detection system
2. Add capability-based control propagation
3. Allow followers to expose their features to masters

### Phase 4: Advanced Features
1. Selective disconnection (disconnect specific follower, not all)
2. State synchronization across chain
3. Visualization improvements for chains
4. Tank content management across chain

## Special Considerations

### 1. Physics Stability
- Multiple joints in a chain may cause instability
- May need to tune joint parameters (frequency, damping) based on chain length
- Consider maximum chain length limits

### 2. Control Latency
- Controls propagate sequentially through chain
- May need to implement parallel update system for large chains

### 3. Circular References
- Prevent robots from connecting in circles
- Implement cycle detection in connection logic

### 4. Save/Load State
- Need to serialize connection chains
- Restore master/follower relationships on load

## Example Use Cases

### 1. Basic Trailer Chain
```
Tractor -> Grain Trailer -> Grain Trailer
- All movement controlled by tractor
- No steering on trailers
- Tanks can be filled independently
```

### 2. Advanced Harvesting Train
```
Harvester -> Powered Trailer -> Steering Trailer
- Harvester controls movement
- Powered trailer assists with acceleration
- Steering trailer helps with turning radius
```

### 3. Modular Transport System
```
Truck -> Flatbed -> Container -> Container
- Each container can be loaded/unloaded independently
- Truck controls entire train
- Containers can be dynamically attached/detached
```

## Testing Strategy

1. **Unit Tests**: Test individual connection/disconnection logic
2. **Integration Tests**: Test full chains with physics simulation
3. **Performance Tests**: Measure impact of long chains on frame rate
4. **Stress Tests**: Maximum chain length, rapid connect/disconnect

## Future Extensions

1. **Hydraulic Systems**: Followers can provide hydraulic power to implements
2. **Electronic Systems**: CAN bus simulation for advanced communication
3. **Load Balancing**: Distribute engine power across powered followers
4. **Autonomous Followers**: Followers with limited autonomous capabilities
5. **Quick Couplers**: Faster connection/disconnection mechanisms