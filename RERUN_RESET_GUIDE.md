# Rerun Reset Guide for Multiverse Simulator

## Problem
Every time you run the multiverse simulation, old data remains in the Rerun viewer, requiring you to manually close and reopen the viewer.

## Solutions

### 1. **Clear All Entities (Recommended)**
Use the new `clear_all_entities()` method to clear all visualization data:

```cpp
// In your simulation loop or at startup
simulator.clear_all_entities();
```

**What it does:**
- Clears all entities recursively from the root
- Resets the timeline
- Keeps the same recording session

### 2. **Reset Recording Session**
Create a completely new recording session with `reset_recording()`:

```cpp
// At the start of your simulation
simulator.reset_recording();
```

**What it does:**
- Creates a new recording stream with a unique session ID
- Spawns a new connection to Rerun viewer
- Resets all time state
- Increments session counter automatically

### 3. **Manual Approach**
If you want more control, you can manually clear specific entities:

```cpp
// Clear specific robot data
rec->log("robot_name", rerun::Clear::RECURSIVE);

// Clear all data from root
rec->log("", rerun::Clear::RECURSIVE);

// Reset timeline
rec->reset_time();
```

## Usage Examples

### Basic Usage in Main Loop
```cpp
#include "multiverse/simulator.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("multiverse");
    rec->spawn().exit_on_failure();
    
    mvs::Simulator simulator(rec);
    
    // Clear any existing data at startup
    simulator.clear_all_entities();
    
    // Initialize your simulation
    simulator.init(datum, world_size);
    
    // Run simulation loop
    while (running) {
        simulator.tick(dt);
    }
    
    return 0;
}
```

### Reset Between Simulation Runs
```cpp
void restart_simulation() {
    // Option 1: Clear data but keep same session
    simulator.clear_all_entities();
    
    // Option 2: Start completely new session
    // simulator.reset_recording();
    
    // Reset simulation state
    simulator.init(datum, world_size);
}
```

### Clear Specific Robot Data
```cpp
void remove_robot(const std::string& robot_name) {
    // Clear the robot's visualization data
    rec->log(robot_name, rerun::Clear::RECURSIVE);
    
    // Remove from simulator
    // ... your robot removal logic
}
```

## API Reference

### `Simulator::clear_all_entities()`
- **Purpose**: Clear all visualization data from current session
- **Effect**: Removes all entities, resets timeline
- **Use case**: Clean slate while keeping same recording session

### `Simulator::reset_recording()`
- **Purpose**: Start completely new recording session
- **Effect**: New session ID, fresh connection to viewer
- **Use case**: Complete reset, especially useful for long-running applications

### Manual Clear with `rerun::Clear`
- **`rerun::Clear::FLAT`**: Clear specific entity only
- **`rerun::Clear::RECURSIVE`**: Clear entity and all children
- **Usage**: `rec->log("entity_path", rerun::Clear::RECURSIVE)`

## Best Practices

1. **Call `clear_all_entities()` at startup** for clean visualization
2. **Use `reset_recording()` sparingly** - only when you need a completely fresh session
3. **Clear specific entities** when removing objects from simulation
4. **Reset timeline** when restarting time-based simulations

## Limitations

- **Viewer memory**: The Rerun viewer may still hold some data in memory
- **No retroactive clearing**: Only affects latest-at queries, not historical data
- **Buffering**: Some data may be buffered until flushed

## Troubleshooting

**Q: Data still appears after clearing**
A: Try `reset_recording()` instead of `clear_all_entities()`

**Q: Performance issues with frequent clearing**
A: Clear specific entities instead of all entities

**Q: Viewer not updating**
A: Ensure `rec->spawn()` was called and connection is active