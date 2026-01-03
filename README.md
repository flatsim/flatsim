<img align="right" width="26%" src="./book/src/images/logo.png">

flatsim
===

Lightweight 2D robotics simulator (C++) with a small, pragmatic core and clear Agent ↔ Simulator split.

Key libraries used (current):
- datapod — geographic types & serialization (WGS/ENU, Pose, Geo, etc.)
- flywheel — 2D rigid-body physics and collisions
- rerun — optional visualization / recording stream
- pigment — simple RGB helpers used by examples & loaders
- ZeroMQ (libzmq / cppzmq) — IPC/TCP transport between Agent and Simulator
- boost::json — config / machine loader

Note: previous versions and older docs mention kokkos, muli, concord, zoneout and other dependencies. The codebase has moved — those are no longer primary dependencies.

Quick example (single-process / LOCAL mode):

```cpp
// Create simulator (datum required for GPS conversions)
datapod::Geo datum{51.989, 5.658, 53.8};
simulator::Simulator sim(500.0f, 500.0f, datum);

// Spawn a robot from JSON file
datapod::Pose pose{datapod::Point{5.0f, 0.0f, 0.0f}, datapod::Quaternion::from_euler({0,0,0})};
auto &robot = sim.spawn_agent("examples/machines/tractor.json", pose);

// Simple run loop
const float dt = 0.016f; // 60 Hz
for (int i = 0; i < 1000; ++i) { sim.tick(dt); sim.tock(); }
```

Deployment modes
- LOCAL: Simulator and Agents run in the same process (useful for tests and debugging)
- IPC / TCP: Run simulator as a server and connect Agents remotely (examples: simulator_server, agent_client)

Examples (build system dependent):
- simulator_server  # runs the physics server (use --ipc or --tcp)
- agent_client      # connects to simulator server and exercises Agent API
- simple            # single-process example that demonstrates Simulator + Agent

Client–server snippets (IPC / TCP)

IPC (same machine) — server:

```cpp
#include "flatsim/simulator.hpp"

int main() {
    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    // IPC mode: address string is unused; uses FLATSIM_IPC_DIR (defaults to /tmp)
    simulator::Simulator sim(simulator::Conn::IPC, "", 500.0f, 500.0f, datum);
    const float dt = 0.016f;
    bool running = true;
    while (running) {
        sim.tick(dt);
        sim.tock();
    }
}
```

IPC — client:

```cpp
#include "flatsim/agent.hpp"
#include "flatsim/agent/loader.hpp"

int main() {
    datapod::Pose pose = utils::make_pose_2d(10.0, 10.0, 0.0);
    types::Machine m = agent::Loader::load_from_json("examples/machines/tractor.json", pose);

    // Empty address -> IPC transport
    agent::Agent client("");
    client.set_machine(m);
    client.spawn();            // send SPAWN to server
    client.tick(0.016f, 100);  // tick blocks waiting for state (100 ms timeout)
    client.despawn();
}
```

TCP (different machines) — server snippet:

```cpp
// Bind to all interfaces (advertised to clients)
simulator::Simulator sim(simulator::Conn::TCP, "0.0.0.0", 500.0f, 500.0f, datum);
```

TCP — client snippet:

```cpp
// Connect to server running at 192.168.1.10
agent::Agent client("192.168.1.10");
```

Note: For TCP, ensure clients can reach the server (open firewall/ports). See examples/simulator_server.cpp and examples/agent_client.cpp for complete, working examples.

Key components
- simulator::Simulator — physics world, machine creation, tick/tock loop, lidar scans, teleport
- agent::Agent — local or remote Agent; handles controls, sensors, spawn/despawn
- types:: — machine/state/serialization types (datapod-based)
- flywheel/rerun integration — physics + optional visualization stream

Where to look
- examples/ — runnable demos (single-process and client/server)
- include/ & src/ — main implementation (simulator, agent, sensors, controls)
- CMakeLists.txt / xmake.lua / Makefile — build instructions

If something in the README or docs still references old libraries, that's likely stale: prefer the code and examples/ as ground truth.
