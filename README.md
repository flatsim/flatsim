<img align="right" width="26%" src="./book/src/images/logo.png">

flatsim
===

Lightweight 2D robotics simulator (C++) with a small, pragmatic core and clear Agent ↔ Simulator split.

Key libraries:
- **datapod** — geographic types & serialization (WGS/ENU, Pose, Geo, etc.)
- **flywheel** — 2D rigid-body physics and collisions
- **drivekit** — path following controllers (PID, MPPI, Stanley, Pure Pursuit, MPC)
- **agent47** — IPC/TCP communication protocol between Agent and Simulator
- **rerun** — optional visualization / recording stream
- **robomod** — URDF machine loader

## Quick Example (LOCAL mode)

```cpp
#include "flatsim/agent.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include <drivekit.hpp>

int main() {
    // Create simulator with datum (required for GPS conversions)
    datapod::Geo datum{51.989, 5.658, 53.8};
    simulator::Simulator sim(500.0f, 500.0f, datum);

    // Spawn a robot from URDF file
    datapod::Pose pose = utils::make_pose_2d(0.0, 0.0, 0.0);
    auto &robot = sim.spawn_agent("machines/urdf/tractor.urdf", pose, "my_robot");

    // Set up path following with Pure Pursuit controller
    robot.tracker()->set_controller_type(drivekit::TrackerType::PURE_PURSUIT);
    robot.set_tracker_enabled(true);

    // Define a path
    std::vector<datapod::Point> waypoints = {{10, 0}, {20, 5}, {30, 10}};
    drivekit::PathGoal path(waypoints, 2.0f, 3.0f, false);
    robot.tracker()->set_path(path);

    // Run simulation loop
    const float dt = 0.016f;  // ~60 Hz
    while (!robot.tracker()->is_path_completed()) {
        sim.tick(dt);
        sim.tock();
    }
}
```

## Deployment Modes

| Mode | Description |
|------|-------------|
| **LOCAL** | Simulator and Agents in same process (tests, debugging) |
| **IPC** | Agents connect via Unix sockets (same machine) |
| **TCP** | Agents connect over network (different machines) |

## Agent API

The Agent class provides a simplified interface for robot control:

```cpp
// Control (Twist commands - Simulator handles wheel conversion)
agent.set_velocity(linear, angular);   // Set both at once
agent.set_linear(1.0f);                // Forward/backward
agent.set_angular(0.5f);               // Rotation

// Speed scaling and braking
agent.set_speed(0.8f);                 // Scale factor (0.0 - 1.0)
agent.speed_up(0.1f);                  // Increase speed
agent.slow_down(0.1f);                 // Decrease speed
agent.brake();                         // Stop immediately

// Path following (via drivekit::Tracker)
agent.tracker()->set_controller_type(drivekit::TrackerType::PID);
agent.tracker()->set_path(path);
agent.set_tracker_enabled(true);       // Enable autonomous navigation

// State accessors
agent.get_position();                  // Current pose
agent.get_linear_velocity();           // Current linear velocity
agent.get_angular_velocity();          // Current angular velocity
agent.get_sensor_data();               // Sensor readings (lidar, etc.)
```

## Path Following Controllers

Available controller types in `drivekit::TrackerType`:
- `PID` — Classic PID control
- `MPPI` — Model Predictive Path Integral
- `STANLEY` — Stanley lateral controller
- `PURE_PURSUIT` — Carrot-following controller
- `MPC` — Model Predictive Control

## Client-Server Mode (IPC/TCP)

**Server (Simulator):**
```cpp
#include "flatsim/simulator.hpp"

int main() {
    datapod::Geo datum{51.989, 5.658, 53.8};

    // IPC mode (Unix sockets)
    simulator::Simulator sim(simulator::Conn::IPC, "", 500.0f, 500.0f, datum);

    // Or TCP mode (network)
    // simulator::Simulator sim(simulator::Conn::TCP, "0.0.0.0", 500.0f, 500.0f, datum);

    while (true) {
        sim.tick(0.016f);
        sim.tock();
    }
}
```

**Client (Agent via agent47):**
```cpp
#include "flatsim/agent.hpp"

int main() {
    types::Machine config = /* load from URDF */;

    // Connect via IPC (empty address) or TCP (server IP)
    agent::Agent client(config, "", rec);  // IPC
    // agent::Agent client(config, "192.168.1.10", rec);  // TCP

    client.spawn();

    while (running) {
        client.set_velocity(1.0f, 0.0f);
        client.tick(0.016f, 100);
        client.tock();
    }

    client.despawn();
}
```

## Project Structure

```
flatsim/
├── include/flatsim/
│   ├── agent.hpp          # Agent class
│   ├── simulator.hpp      # Simulator class
│   ├── types.hpp          # Core types (Machine, SensorData, etc.)
│   └── utils.hpp          # Utility functions
├── src/
│   ├── agent.cpp
│   ├── simulator.cpp
│   └── simulator/         # Physics, machine handling
├── machines/
│   └── urdf/              # Robot URDF definitions
│       ├── tractor.urdf
│       ├── harvester.urdf
│       └── ...
└── examples/
    ├── pipe_server.cpp        # IPC/TCP server example
    ├── pipe_agent.cpp         # IPC/TCP agent example
    ├── pipe_one_binary.cpp    # Single binary with both
    └── complex_examples/      # Path following demos
        ├── test_pid.cpp
        ├── test_mppi.cpp
        ├── test_stanley.cpp
        ├── test_pure_pursuit.cpp
        └── ...
```

## Building

```bash
# xmake (recommended)
xmake build

# Run examples
xmake run pipe_one_binary
xmake run test_pid
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Simulator                             │
│  ┌─────────┐  ┌─────────┐  ┌──────────────────────────────┐ │
│  │ World   │  │ Machine │  │ twist_to_wheel_control()     │ │
│  │(physics)│  │ (URDF)  │  │ (Ackermann/Differential)     │ │
│  └─────────┘  └─────────┘  └──────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
           ▲                           │
           │ Commands (Twist)          │ State (Pose, Sensors)
           │                           ▼
┌─────────────────────────────────────────────────────────────┐
│                          Agent                               │
│  ┌────────────────┐  ┌────────────────────────────────────┐ │
│  │ drivekit       │  │ set_velocity() / get_twist()       │ │
│  │ ::Tracker      │  │ (Twist commands)                   │ │
│  └────────────────┘  └────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

**Key design decisions:**
- Agent sends Twist commands (linear/angular velocity)
- Simulator converts Twist → WheelControl (Ackermann steering or differential drive)
- Path following is handled by drivekit::Tracker on the Agent side
- In networked mode, agent47 protocol handles communication (COMMAND, FEEDBACK, SENSOR, HEARTBEAT)

## Notes

- URDF paths are resolved relative to the working directory or project root
- Rerun visualization is optional (pass `nullptr` for `rec` parameter)
- The datum (GPS reference point) is required for coordinate conversions
