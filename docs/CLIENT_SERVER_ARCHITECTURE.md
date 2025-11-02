# Client-Server Architecture

## Overview

Flatsim uses a **process separation architecture** where robot control code runs in separate processes from the physics simulator. This is achieved using ZeroMQ (ZMQ) for inter-process communication (IPC) and TCP networking.

```
┌─────────────────┐                    ┌─────────────────┐
│  Robot Process  │                    │  Robot Process  │
│   (robo.cpp)    │                    │   (robo.cpp)    │
│                 │                    │                 │
│   Client        │                    │   Client        │
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         │  REQ/PUSH/SUB                        │  REQ/PUSH/SUB
         │  (IPC or TCP)                        │  (IPC or TCP)
         │                                      │
         └──────────┬───────────────────────────┘
                    │
                    ▼
         ┌──────────────────────┐
         │  Simulator Process   │
         │    (sim.cpp)         │
         │                      │
         │   Dispatcher         │
         │   REP/PULL/PUB       │
         │                      │
         │   Physics Engine     │
         │   World State        │
         └──────────────────────┘
```

## Components

### 1. **Dispatcher** (Server Side)
**Location:** `include/flatsim/dispatcher.hpp`, `src/dispatcher.cpp`

The Dispatcher runs in the simulator process and acts as the server. It manages:
- Robot spawn requests from robot processes
- Control commands from robot processes  
- Physics state broadcasts to robot processes

**ZMQ Sockets:**
- `spawn_socket` (REP): Listens for spawn requests on port 5555 (TCP) or `ipc:///tmp/flatsim_spawn` (IPC)
- `robot_command_sockets` (PULL per robot): Unique socket for each robot's commands
- `robot_state_sockets` (PUB per robot): Unique socket for each robot's state updates

### 2. **Client** (Robot Side)
**Location:** `include/flatsim/robot/systems/client.hpp`, `src/robot/systems/client.cpp`

The Client runs in robot processes and acts as the client. It manages:
- Spawning robot in simulator
- Sending control commands to simulator
- Receiving physics states from simulator

**ZMQ Sockets:**
- `spawn_socket` (REQ): Sends spawn requests and waits for replies
- `command_socket` (PUSH): Sends control commands to assigned endpoint
- `state_socket` (SUB): Subscribes to assigned state endpoint

## Communication Protocol

### 1. Robot Spawning

**Flow:**
1. Robot process creates `Client` and calls `client.init(use_tcp, host)`
2. Client connects to shared spawn endpoint and sends `SpawnRobotRequest` containing `RobotInfoMessage` via REQ socket
3. Dispatcher receives request on REP socket
4. Dispatcher creates robot in simulator and adds to registry
5. **Dispatcher allocates unique endpoints for this robot:**
   - Creates PULL socket for robot's commands (IPC: `ipc:///tmp/flatsim_cmd_{uuid}` or TCP: `tcp://*:{port}`)
   - Creates PUB socket for robot's state (IPC: `ipc:///tmp/flatsim_state_{uuid}` or TCP: `tcp://*:{port+1}`)
6. Dispatcher replies with `SpawnRobotReply` containing assigned endpoints
7. Client connects command and state sockets to assigned endpoints
8. Client is now ready to send/receive on dedicated channels

**Message Types:**
- `SpawnRobotRequest`: Contains robot configuration (RobotInfoMessage) and timestamp
- `SpawnRobotReply`: Contains robot UUID, success flag, error message, **command_endpoint**, and **state_endpoint**

### 2. Control Commands

**Flow:**
1. Robot process sends control commands via `client.send_control_command(cmd)`
2. Client pushes `ControlCommand` message via PUSH socket to its dedicated endpoint
3. Dispatcher pulls commands from all robot-specific PULL sockets in `receive_commands()`
4. Simulator applies commands to corresponding robot (matched by UUID)

**Message Type:**
- `ControlCommand`: Contains robot UUID, timestamp, steering, and throttle values

**Note:** Each robot has its own command socket, eliminating the need for shared channels.

### 3. Physics State Updates

**Flow:**
1. Simulator updates physics each tick
2. Dispatcher calls `send_states()` which sends `PhysicsState` to each robot's dedicated PUB socket
3. Client receives state via SUB socket (non-blocking) in `receive_physics_state()`
4. Robot process uses state for decision making

**Message Type:**
- `PhysicsState`: Contains robot UUID, timestamp, pose (position + orientation), and velocity

**Note:** Each robot receives ONLY its own state - no filtering needed on client side.

## Transport Modes

### IPC (Inter-Process Communication)
- **Default mode**
- Uses Unix domain sockets (`ipc:///tmp/flatsim_*`)
- Fast, low latency
- Only works on same machine
- Good for local development and testing

### TCP (Network Communication)
- **Optional mode**
- Uses TCP sockets on localhost or network (`tcp://host:port`)
- Supports remote connections
- Slightly higher latency than IPC
- Good for distributed systems, Docker containers, remote robots

**Usage:**
```cpp
// IPC mode (default)
client.init();

// TCP mode
client.init(true, "127.0.0.1");  // localhost
client.init(true, "192.168.1.100");  // remote host
```

## Port Allocation

| Service | IPC Path | TCP Port |
|---------|----------|----------|
| Spawn (REQ/REP) | `ipc:///tmp/flatsim_spawn` | 5555 (shared) |
| Robot Commands (PUSH/PULL) | `ipc:///tmp/flatsim_cmd_{uuid}` | 6000, 6010, 6020... (base+0) |
| Robot State (PUB/SUB) | `ipc:///tmp/flatsim_state_{uuid}` | 6001, 6011, 6021... (base+1) |
| *Reserved for sensors* | - | base+2 to base+9 |

**TCP Port Allocation:**
- Spawn endpoint: Fixed at 5555
- Robot endpoints: Each robot gets 10 ports (base, base+1, ..., base+9)
  - Port 0 (base+0): Commands
  - Port 1 (base+1): State  
  - Ports 2-9 (base+2 to base+9): Reserved for future use (sensors, cameras, etc.)

**Example for 3 robots:**
- Robot 1: Ports 6000-6009
- Robot 2: Ports 6010-6019
- Robot 3: Ports 6020-6029

## Example Usage

### Simulator (Server)
```cpp
// examples/sim.cpp
auto sim = std::make_shared<fs::Simulator>(rec);
sim->init(world_datum, world_size);
sim->enable_dispatcher();  // Start listening for connections
sim->ticktock([&](float dt) {
    return true;
}, 30);
```

### Robot (Client)
```cpp
// examples/robo.cpp
fs::Client client;
client.init(false);  // Use IPC
// OR
client.init(true, "127.0.0.1");  // Use TCP

// Spawn robot
if (client.spawn_robot(robot_info)) {
    // Control loop
    while (true) {
        auto state = client.receive_physics_state();
        
        fs::messages::ControlCommand cmd(
            client.get_uuid(), 0.0, steering, throttle
        );
        client.send_control_command(cmd);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
```

## Key Features

1. **Process Isolation**: Robot control logic runs independently from physics simulation
2. **Multiple Robots**: Many robot processes can connect to one simulator
3. **Dedicated Endpoints**: Each robot gets unique communication channels (no cross-talk)
4. **Non-blocking**: All receive operations are non-blocking to avoid hanging
5. **Dual Transport**: Supports both IPC (fast local) and TCP (networked)
6. **Dynamic Allocation**: Endpoints assigned at spawn time, no pre-configuration needed
7. **Serialization**: Uses JSON for message serialization (human-readable, debuggable)

## Message Serialization

All messages use JSON serialization via nlohmann/json:
```cpp
// Serialize
std::string json_str = message.serialize();

// Deserialize  
auto message = MessageType::deserialize(json_str);
```

See `include/flatsim/robot/systems/messages.hpp` for all message types and serialization implementations.
