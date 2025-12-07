
<img align="right" width="26%" src="./book/src/images/logo.png">

flatsim
===

simple robotics simulator using:
- [kokkos](https://github.com/kokkos/kokkos) gpu programming
- [muli](https://github.com/Sopiro/Muli) for physics
- [rerun](https://github.com/rerun-io/rerun) visualization
- [concord](https://github.com/onlyhead/concord) cordniate transformer
- [zoneout](https://github.com/onlyhead/zoneout) zones management


A short video:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/xjRNYtFulUs/0.jpg)](https://www.youtube.com/watch?v=xjRNYtFulUs)

## Architecture

Flatsim provides a clean, modular architecture that separates physics simulation from robot control and navigation. The system supports flexible deployment patterns from single-process to distributed multi-machine setups.

### Core API: fs::Simulator

The main simulation API based on the protocol layer for clean separation of concerns:

```cpp
// Create simulator
auto sim = std::make_shared<fs::Simulator>(rec);

// Initialize world
sim->init(world_datum, world_size);
sim->add_robot(robot_info);

// Main loop with protocol types
std::vector<protocol::RobotCommand> commands;
std::vector<protocol::RobotState> states;

sim->step(dt, commands, states);  // Physics step
```

## Deployment Modes

### Mode 1: Separated (Distributed Client/Server)

Run environment server and robot agents as separate processes. Ideal for multi-robot systems and multi-machine deployments.

**Usage:**
```bash
# Terminal 1: Start environment server
./build/server_env --config examples/machines/tractor.json

# Terminal 2: Start agent
./build/agent_nav --config examples/machines/tractor.json --pose 0,0,0 --target-x 10 --target-y 5
```

### Mode 2: Joined (Single Process)

Run environment and agents in the same process. Ideal for testing, debugging, and low-latency scenarios.

**Usage:**
```bash
./build/single_process_demo --config examples/machines/tractor.json --target-x 10 --target-y 5
```

## Key Components

### fs::Simulator
Physics-centric simulator managing world and robot instances:
- `init(datum, world_size)` - Initialize world
- `add_robot(robot_info)` - Add robot to simulation  
- `step(dt, commands, states)` - Step physics and produce robot states
- `world()`, `robots()` - Access world and robot instances

### protocol::
Transport-agnostic data types:
- `RobotState` - pose, velocity
- `RobotCommand` - steering, throttle
- `CollisionEvent` - collision data

### agent::
- `IRobotAgent` - Generic interface for robot control
- `NavAgent` - Navigation using navcon
- `LoggingAgent` - Optional Rerun logging

### ipc::
- `Adapters` - Convert protocol ↔ ZMQ messages
- `Server/Client` - ZMQ dispatcher

## Benefits

✅ **Clean Architecture** - Single modular API  
✅ **Scalable** - Physics separate from navigation  
✅ **Flexible** - Single-process or distributed  
✅ **Testable** - Agents independent from physics  
✅ **Multi-machine** - Agents on different hardware

## Examples

All examples use the `fs::Simulator` API:
- `server_env`, `agent_nav`, `single_process_demo` - New modular examples
- `sim`, `mvs`, `zmq_tractor_control` - Existing examples

See [PLAN.md](./PLAN.md) and [ARCHITECTURE_SUMMARY.md](./ARCHITECTURE_SUMMARY.md) for details.

