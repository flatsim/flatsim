# FlатSim Architecture - Full Process Separation

## Overview

FlатSim is designed with **complete process separation** between agent (client) and simulator (server). They communicate exclusively via ZMQ network protocol.

## Design Principles

1. **Independent Processes**: Agent and simulator run as separate executables
2. **Network Communication Only**: ZMQ sockets with cista serialization
3. **Namespace Isolation**: 
   - `agent::` - Client-side code, NO physics dependencies
   - `simulator::` - Server-side code with physics (muli)
   - `types::` - Shared data structures for serialization
4. **Tick/Tock Pattern**: Both agent and simulator use tick/tock for update/visualization

## Architecture Diagram

```
┌─────────────────────────────────────┐
│         Agent Process               │
│  (agent:: namespace only)           │
│                                     │
│  ┌─────────────────────────────┐   │
│  │  agent::Agent               │   │
│  │  - tick() [BLOCKING]        │   │
│  │  - tock() [visualization]   │   │
│  └─────────────────────────────┘   │
│              │                      │
│  ┌───────────▼──────────────────┐  │
│  │  agent::Machine              │  │
│  │  - tick()                    │  │
│  │  - tock()                    │  │
│  └──────────────────────────────┘  │
│              │                      │
│  ┌───────────▼──────────────────┐  │
│  │  agent::ControlManager       │  │
│  │  - tick()                    │  │
│  │  - tock()                    │  │
│  └──────────────────────────────┘  │
│                                     │
│         ZMQ Client Sockets          │
│         (REQ, PUSH, SUB)            │
└──────────────┬──────────────────────┘
               │ ZMQ Protocol
               │ (IPC or TCP)
               │
┌──────────────▼──────────────────────┐
│         Simulator Process           │
│  (simulator:: namespace only)       │
│                                     │
│  ┌─────────────────────────────┐   │
│  │  simulator::Simulator       │   │
│  │  - tick()                   │   │
│  │  - tock()                   │   │
│  └─────────────────────────────┘   │
│              │                      │
│  ┌───────────▼──────────────────┐  │
│  │  simulator::Machine          │  │
│  │  - tick()                    │  │
│  │  - tock()                    │  │
│  └──────────────────────────────┘  │
│              │                      │
│  ┌───────────▼──────────────────┐  │
│  │  simulator::Chassis          │  │
│  │  simulator::Wheel            │  │
│  │  simulator::World (muli)     │  │
│  └──────────────────────────────┘  │
│                                     │
│         ZMQ Server Sockets          │
│         (REP, PULL, PUB)            │
└─────────────────────────────────────┘
```

## Tick/Tock Pattern

### Agent Side (BLOCKING)

```cpp
// tick() blocks until state message received from simulator
void agent::Agent::tick(float dt, int timeout_ms = 100) {
    // BLOCKING: Wait for state update via ZMQ SUB socket
    state_socket_->recv(...);  // Blocks here
    
    // Update machine state
    machine_.update_state(state);
    
    // Call machine tick to process update
    machine_.tick(dt);
}

void agent::Agent::tock() {
    // Visualization (currently placeholder)
    machine_.tock();
}
```

**Key Points:**
- `tick()` is **blocking** - waits for simulator state update
- Creates natural synchronization between agent and simulator
- Timeout prevents infinite blocking
- All sub-components (Machine, ControlManager) have tick/tock called hierarchically

### Simulator Side (NON-BLOCKING)

```cpp
void simulator::Simulator::tick(float dt) {
    // Process spawn/despawn requests (non-blocking)
    // Process control commands (non-blocking)
    
    // Tick all machines
    for (auto &machine : machines_) {
        machine.tick(dt);
    }
    
    // Step physics simulation
    world_->tick(dt);
    
    // Publish state to all agents
    for (auto &socket : state_sockets_) {
        socket->send(state);  // PUB socket, fire-and-forget
    }
}
```

## Communication Protocol

### Message Types (ZMQ)

1. **Spawn/Despawn** (REQ/REP)
   - Agent → Simulator: `types::ser::Request` (SPAWN/DESPAWN)
   - Simulator → Agent: `types::ser::Response` (success + world state)

2. **Control** (PUSH/PULL)
   - Agent → Simulator: `types::ser::WheelControl`
   - Fire-and-forget, no response

3. **State** (PUB/SUB)
   - Simulator → Agent: `types::ser::MachineState`
   - Published every tick, agent blocks on this

### Sockets

```
Agent Side:
- spawn_socket:   REQ  → /tmp/flatsim_spawn
- control_socket: PUSH → /tmp/flatsim_ctrl_{uuid}
- state_socket:   SUB  ← /tmp/flatsim_state_{uuid}

Simulator Side:
- spawn_socket:   REP  ← /tmp/flatsim_spawn
- control_socket: PULL ← /tmp/flatsim_ctrl_{uuid}
- state_socket:   PUB  → /tmp/flatsim_state_{uuid}
```

## Running Separate Processes

### 1. Start Simulator Server

```bash
./build/simulator_server
```

Output:
```
[Server] Starting simulator server...
[Server] Simulator ready. Waiting for agent connections...
[Server] Press Ctrl+C to stop
```

### 2. Start Agent Client(s)

```bash
# Terminal 2
./build/agent_client examples/machines/tractor.json

# Terminal 3
./build/agent_client examples/machines/husky.json
```

Output:
```
[Client] Starting agent client...
[Client] Loaded machine: Tractor
[Client] Spawning in simulator...
[Client] Spawned successfully!
[Client] Running control loop (10 seconds)...
[Client] Pose: (0.5, 0.2) yaw=0.1
...
```

## Code Organization

```
include/flatsim/
├── agent/              # Agent namespace (NO physics)
│   ├── control_manager.hpp
│   ├── loader.hpp
│   └── machine.hpp
├── simulator/          # Simulator namespace (WITH physics)
│   ├── machine/
│   │   ├── chassis.hpp
│   │   ├── wheel.hpp
│   │   └── ...
│   ├── world/
│   │   └── obstacle.hpp
│   ├── machine.hpp
│   └── world.hpp
├── agent.hpp           # Top-level agent API
├── simulator.hpp       # Top-level simulator API
├── types.hpp           # Shared types (serialization)
└── utils.hpp           # Shared utilities

src/
├── agent/              # Agent implementations
│   ├── control_manager.cpp
│   ├── loader.cpp
│   └── machine.cpp
├── simulator/          # Simulator implementations
│   ├── machine/
│   ├── world/
│   ├── machine.cpp
│   └── world.cpp
├── agent.cpp
└── simulator.cpp

examples/
├── simulator_server.cpp   # Standalone simulator
├── agent_client.cpp        # Standalone agent
├── simple.cpp              # Direct simulator API (testing)
└── README.md
```

## Separation Guarantees

✅ **Agent NEVER includes:**
- `simulator/*` headers
- `muli/*` headers (physics)
- Any simulator implementation details

✅ **Simulator NEVER includes:**
- `agent/*` headers (except types)
- Any agent implementation details

✅ **Communication:**
- Only via ZMQ network protocol
- Serialized using cista (zero-copy)
- Well-defined message types in `types::ser`

✅ **Build:**
- Can compile agent without simulator dependencies
- Can compile simulator without agent dependencies
- Shared types library for both

## Benefits

1. **Scalability**: Multiple agents can connect to one simulator
2. **Distribution**: Agent and simulator can run on different machines
3. **Testing**: Can test agent/simulator independently
4. **Development**: Teams can work on agent/simulator separately
5. **Deployment**: Deploy agent and simulator independently

## Future Extensions

- Add more tick/tock visualization in agent (rerun integration)
- Implement control smoothing in agent::ControlManager::tick()
- Add prediction/interpolation in agent::Machine::tick()
- Support TCP for remote connections
- Multi-simulator support (agent connects to multiple sims)
