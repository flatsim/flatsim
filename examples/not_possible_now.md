# TODO: Remaining Gaps for “Pedantic + Symmetric” Behavior

This file lists the remaining missing pieces to fully realize:
- symmetric state/sensors transport, and
- obstacle-aware (SOC/MCA) tracking, and
- chain (tractor+trailer / follower) operations in LOCAL mode.

## 1) WorldConstraints wiring (SOC/MCA obstacle awareness)

### What’s missing
- `agent::Tracker::update(...)` calls `drivekit::Tracker::tick(state, dt)` without providing `drivekit::WorldConstraints`.
  - File: `src/agent/control/tracker.cpp`
- The simulator has obstacles (`simulator::World` and `types::{StaticObstacle, DynamicObstacle}`), but we do not convert
  them into the `drivekit::WorldConstraints` format expected by SOC/MCA:
  - `drivekit::WorldConstraints::obstacles` uses Gaussian predictions (`Obstacle::modes[...].mean_x/mean_y/std_x/std_y`).
  - Files: `include/flatsim/simulator/world.hpp`, `src/simulator/world.cpp`, `src/simulator/world/obstacle.cpp`,
    `build/_deps/drivekit-src/include/drivekit/types.hpp`
- Dynamic obstacles are not currently updated from inside `simulator::World::tick(...)`.
  - Only `World::update_obstacles(dt, ref_x, ref_y)` updates `DynamicObstacle` positions.

### What to implement
1. Add a per-tick `drivekit::WorldConstraints` builder:
   - Inputs: simulator obstacles (static + dynamic), robot pose (for activation distance), time horizon info.
   - Output: `drivekit::WorldConstraints` with `Obstacle` predictions for each obstacle id.
2. Pass it to drivekit:
   - Change `agent::Tracker::update(...)` to call `tracker_->tick(state, dt, &constraints)` when available.
3. Decide where the constraints live:
   - Option A: build constraints in the Agent (LOCAL mode only) by reading `simulator::World`.
   - Option B: build constraints in the Simulator and include them in a new message type (IPC/TCP too).
4. Make dynamic obstacle updates deterministic:
   - Call `world_.update_obstacles(dt, ref_x, ref_y)` inside `Simulator::tick(...)` (once per tick).

### Current status
- `examples/test_soc_local.cpp` and `examples/test_mca_local.cpp` intentionally run without `WorldConstraints` (they place
  obstacles in the physics world, but SOC/MCA obstacle cost is not engaged).

## 2) Chain operations (attach/detach trailers, chain status)

### What’s missing
- The current simulator chain helper exists, but it only toggles hitch flags and does not create an articulated physics
  joint:
  - File: `include/flatsim/simulator/chain.hpp` (note says “physics joint creation is TODO”)
- LOCAL-mode Agent API does not expose chain operations (connect/disconnect/status) for examples to use.
  - This blocks porting `examples_old/mvs.cpp` chain button actions 1:1.

### What to implement
1. Add actual physics coupling:
   - Implement a muli joint/constraint between tractor and trailer bodies at the hitch attachment points.
2. Expose chain operations in LOCAL mode:
   - Provide a `simulator::Simulator` API to connect/disconnect by uuid and hitch names, or “connect nearest compatible”.
   - Provide a safe wrapper so examples can do:
     - connect nearest
     - disconnect last follower
     - print chain status
3. (Optional) If chain should work in IPC/TCP too:
   - Add tagged uplink request messages (agent→sim) for connect/disconnect and sim→agent responses.

### Current status
- `examples/mvs_local.cpp` is a simplified multi-vehicle control demo without chain button operations.

