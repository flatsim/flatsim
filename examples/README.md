# Examples

This directory contains examples demonstrating the flatsim architecture with **full process separation**.

## Architecture

The flatsim system is designed with complete separation between agent and simulator:

- **Simulator** (`simulator::` namespace) - Physics server that runs independently
- **Agent** (`agent::` namespace) - Client that controls machines via network
- **Communication** - ZMQ sockets with cista serialization (no shared memory)

## Process Separation Examples

### 1. Simulator Server (`simulator_server.cpp`)
Standalone simulator process that:
- Runs physics simulation at 60 Hz
- Listens for agent connections via IPC/TCP
- Handles spawn/despawn requests
- Processes control commands
- Publishes state updates
- **Uses ONLY** `simulator::` namespace

**Run:**
```bash
./build/simulator_server
```

### 2. Agent Client (`agent_client.cpp`)
Standalone agent process that:
- Connects to remote simulator
- Loads machine configuration
- Spawns machine in simulator
- Sends control commands
- Receives state updates
- **Uses ONLY** `agent::` namespace

**Run:**
```bash
# In another terminal (simulator must be running)
./build/agent_client [path/to/machine.json]
```

## Test/Debug Examples

### 3. Simple (`simple.cpp`)
Direct simulator API usage (no network, no agent).
Useful for testing simulator physics in isolation.
**Not recommended for production.**

### 4. Visualization (`visualization.cpp`)
Demonstrates rerun visualization integration.

### 5. Loader Demo (`loader_demo.cpp`)
Shows machine configuration loading from JSON.

## Running Separated Processes

1. **Start the simulator server:**
   ```bash
   ./build/simulator_server
   ```
   The server will listen on IPC socket `/tmp/flatsim_spawn`

2. **Run one or more agents:**
   ```bash
   # Terminal 2
   ./build/agent_client examples/machines/tractor.json

   # Terminal 3
   ./build/agent_client examples/machines/husky.json
   ```

3. Each agent runs independently and communicates only via ZMQ

## Key Principles

- ✅ Agent code **never includes** simulator headers
- ✅ Simulator code **never includes** agent headers
- ✅ Shared types (`types.hpp`) used by both for serialization
- ✅ Communication via network protocol only
- ✅ Each process can be compiled independently
- ✅ Multiple agents can connect to one simulator
- ✅ Agents and simulator can run on different machines
