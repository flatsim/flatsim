# Refactor: Symmetric, Multiplexed ZMQ Messaging

This document captures the refactor work done to make Agent↔Simulator messaging more **predictable**, **pedantic**, and **symmetric**, while reducing the number of ZMQ endpoints/sockets used in IPC/TCP mode.

## Motivation / Aim

1. **State and sensors must be separate messages** (separate structs), but:
2. They should be **sent/received the same way** (same code shape, same error handling, same transport model).
3. Reduce “too many open things” by **multiplexing** multiple message types over a minimal number of sockets.
4. When an Agent adds/configures a sensor, it should **automatically send the sensor options/config to the simulator**.

## Summary of the new transport shape

ZMQ sockets are untyped “byte pipes”. We now multiplex message types over the same socket by tagging the payload with a **1-byte message kind**.

**Minimal, symmetric sockets per robot (IPC/TCP mode):**
- **Spawn**: `REQ/REP` (existing) for SPAWN/DESPAWN handshake and returning endpoints.
- **Downlink** (Simulator → Agent): `PUB/SUB` **one socket** that carries multiple kinds:
  - `STATE` (`types::ser::MachineState`)
  - `SENSORS` (`types::ser::SensorState`)
- **Uplink** (Agent → Simulator): `PUSH/PULL` **one socket** that carries multiple kinds:
  - `CONTROL` (`types::ser::WheelControl`)
  - `HEARTBEAT` (`types::ser::Request` with `MsgType::HEARTBEAT`)
  - `LIDAR_CFG` (`types::ser::LidarConfigMsg`)

This replaces the previous “separate control/state/heartbeat endpoints” model with just:
- `uplink_endpoint`
- `downlink_endpoint`

## Protocol: tagged cista payloads

All messages on uplink/downlink use a single-frame format:

```
[ 1 byte kind ][ cista-serialized payload bytes ... ]
```

This is implemented by:
- `include/flatsim/tagged_zmq.hpp`

Where:
- `flatsim::wire::Kind` is the 1-byte discriminator.
- `flatsim::wire::pack(kind, payload)` serializes payload and prefixes the tag.
- `flatsim::wire::unpack(bytes)` splits into `{kind, payload_bytes}`.
- `flatsim::wire::deserialize<T>(payload_bytes)` deserializes the payload into `T`.

### Why the payload buffer is copied/moved

`cista::deserialize<T>(std::vector<uint8_t>)` expects a mutable buffer in this codebase’s usage. The helper takes/moves a vector to satisfy that.

## Deterministic pairing: `tick_seq`

Because state and sensors are sent as different messages, we added a common monotonic counter so receivers can reason about pairing:

- `types::ser::MachineState` now includes `uint64_t tick_seq`
- `types::ser::SensorState` now includes `uint64_t tick_seq`

Simulator assigns the same `tick_seq` to both message types within a tick.

This doesn’t yet implement strict “wait for matching seq pairs” logic everywhere, but it creates the required invariant for pedantic pairing.

## Sensor config auto-send (LIDAR)

To support “Agent adds a sensor → automatically send options/config to server”:

1. `fs::SensorManager` now supports a callback hook:
   - `set_on_add(std::function<void(Sensor&)>)`
2. `agent::Agent` installs a callback that:
   - Detects newly-added `fs::LIDARSensor`
   - Converts its parameters (min/max range, fov, resolution) into a `types::LidarConfig`
   - Sends a tagged `LIDAR_CFG` uplink message to simulator

On the simulator side, uplink handling applies the config via `Simulator::set_lidar_config(uuid, cfg)`.

### Why a separate config message exists

`types::Machine` contains `lidar` config, but `types::ser::Machine` serialization does not include it yet. Therefore, sending LIDAR config during SPAWN would require changing serialization. The refactor keeps SPAWN minimal and sends LIDAR config post-spawn via uplink.

## Concrete code changes (by file)

### New files
- `include/flatsim/tagged_zmq.hpp`
  - Adds the tagged-wire utilities and `flatsim::wire::Kind` enum.

- `examples/zmq_tagged_types_demo.cpp`
  - Minimal proof that multiple `types::ser::*` structs can share one ZMQ socket using the 1-byte tag.

### Updated wire types
- `include/flatsim/types.hpp`
  - Added `tick_seq` to `types::ser::MachineState` and `types::ser::SensorState`.
  - Replaced `types::ser::ZmqInfo` endpoints:
    - removed: `control_endpoint`, `state_endpoint`, `heartbeat_endpoint`
    - added: `uplink_endpoint`, `downlink_endpoint`
  - Added `types::ser::LidarConfigMsg` for agent→sim LIDAR config updates.

### Agent changes
- `include/flatsim/agent.hpp`, `src/agent.cpp`
  - Replaced:
    - `control_socket_`, `state_socket_`, `heartbeat_socket_`
  - With:
    - `uplink_socket_` (PUSH), `downlink_socket_` (SUB)
  - Agent now:
    - Receives `STATE`/`SENSORS` tagged messages from the same SUB socket.
    - Sends `CONTROL`/`HEARTBEAT`/`LIDAR_CFG` tagged messages on the same PUSH socket.
  - Added `install_sensor_callbacks()` to install the SensorManager on-add hook.

### SensorManager changes
- `include/flatsim/agent/sensor_manager.hpp`, `src/agent/sensor_manager.cpp`
  - Added `set_on_add(...)` callback invoked after sensor is added.
  - Added `for_each(...)` helper for consistent iteration.

### Simulator changes
- `include/flatsim/simulator.hpp`, `src/simulator.cpp`
  - Removed the global `heartbeat_socket_`.
  - Replaced per-robot:
    - `control_sockets_` + `state_sockets_`
  - With:
    - `uplink_sockets_` (PULL) + `downlink_sockets_` (PUB)
  - Heartbeat is now a tagged message on the per-robot uplink channel (so fewer sockets).
  - Sensor state now also publishes over ZMQ (no longer LOCAL-only).

## Running the demo

Build everything with xmake:
```
xmake
```

Run the demo binary:
```
./build/linux/x86_64/release/zmq_tagged_types_demo
```

Expected output:
```
OK: sent/received types::ser::MachineState + types::ser::SensorState on one ZMQ socket using a 1-byte message kind tag
```

## Compatibility notes

- This changes the SPAWN response ZMQ endpoints (`types::ser::ZmqInfo`), so old agents/simulators will not interoperate with the new ones without a compatibility layer.
- In code, there is a best-effort fallback path in the Agent if the new endpoints are empty, but it’s intended only as a transition aid.

## Next suggested steps (to finish the “pedantic symmetry”)

1. Enforce strict pairing in the Agent tick:
   - Process messages until `STATE(tick_seq=N)` and `SENSORS(tick_seq=N)` are both present, then tick controls.
2. Convert more “one-off” paths to tagged messages:
   - Teleport requests, world queries, etc.
3. Continue loop deduplication:
   - Extract common code paths in `agent::Machine::tick(...)` overloads.
   - Use `SensorManager::for_each` across update functions to avoid repeated loop scaffolding.

