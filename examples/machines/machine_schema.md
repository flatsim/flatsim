# Machine Definition JSON Schema

This document describes the JSON format for defining machines in the Multiverse simulator.

## Coordinate System

**All positions are relative to the chassis center (0,0)**:
- **Origin**: Chassis geometric center
- **X-axis**: Left (-) to Right (+)  
- **Y-axis**: Back (-) to Front (+)
- **Z-axis**: Down (-) to Up (+) (usually 0 for 2D simulation)

```
        Front (+Y)
           ↑
    +------+------+
    |      |      |
←---+------O------+---→  
Left |      |      | Right
(-X) |      |      | (+X)
    +------+------+
           |
           ↓
        Back (-Y)
```

## JSON Structure

```json
{
  "info": {
    "type": "string",              // Machine type identifier
    "name": "string",              // Display name
    "uuid": "string",              // Unique identifier (optional)
    "rci": 3,                      // Robot Control Interface version
    "works_on": ["string"],        // Array of materials this machine can work with
    "role": "MASTER|SLAVE|FOLLOWER" // Robot role (default: MASTER)
  },
  
  "dimensions": {
    "width": 0.0,                  // Chassis width in meters
    "height": 0.0,                 // Chassis length in meters
    "description": "string"        // Optional description
  },
  
  "color": {
    "r": 0,                        // Red (0-255)
    "g": 0,                        // Green (0-255)
    "b": 0                         // Blue (0-255)
  },
  
  "wheels": [
    {
      "name": "string",            // Wheel identifier
      "position": {                // Position relative to chassis center
        "x": 0.0,                  
        "y": 0.0,
        "z": 0.0
      },
      "size": {                    // Wheel dimensions
        "width": 0.0,              
        "height": 0.0,
        "depth": 0.0
      },
      "side": "left|right"         // Which side of the vehicle
    }
  ],
  
  "controls": {
    "steering": {
      "max_angles": [0.0],         // Max steering angle per wheel (degrees)
      "differential": [0.0]        // Steering differential per wheel (degrees)
    },
    "throttle": {
      "max_values": [0.0]          // Max throttle per wheel (0.0-1.0)
    }
  },
  
  "karosseries": [                 // Optional body attachments
    {
      "name": "string",
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "size": { "width": 0.0, "height": 0.0, "depth": 0.0 },
      "sections": 0,               // Number of controllable sections (0 = non-working)
      "has_physics": true,         // Whether this part has collision
      "color": {                   // Optional override color
        "r": 0, "g": 0, "b": 0
      },
      "description": "string"      // Optional description
    }
  ],
  
  "hitches": {                     // Optional connection points
    "hitch_name": {
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "size": { "width": 0.05, "height": 0.05, "depth": 0.0 }
    }
  },
  
  "tank": {                        // Optional storage tank
    "name": "string",
    "capacity": 0.0,               // Storage capacity in units
    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "size": { "width": 0.0, "height": 0.0, "depth": 0.0 },
    "description": "string"
  },
  
  "power": {                       // Optional power source
    "name": "string",
    "type": "FUEL|BATTERY",
    "capacity": 0.0,               // Fuel liters or battery kWh
    "consumption_rate": 0.0,       // Units per second at full work
    "charge_rate": 0.0             // For batteries only (units/second)
  }
}
```

## Field Requirements

### Required Fields
- `info` (all subfields required except `uuid`)
- `dimensions` (width and height required)
- `color`
- `wheels` (at least one wheel)
- `controls`

### Optional Fields
- `karosseries`
- `hitches`
- `tank`
- `power`

## Examples

See the following example files:
- `tractor.json` - Simple 4-wheel tractor with rear hitch
- `oxbo_harvester.json` - Complex 6-wheel harvester with sections
- `trailer.json` - 2-wheel trailer with front hitch
- `truck.json` - 8-wheel cargo truck

## Usage

```cpp
#include "flatsim/loader.hpp"

// Load a single machine
auto robot = Loader::load_from_json(
    "machines/tractor.json",
    concord::Pose(10, 10, 0),  // Spawn position
    "MyTractor",                // Optional name override
    pigment::RGB(255, 0, 0)     // Optional color override
);

// Find all machines in a directory
auto files = Loader::find_machine_files("machines/");

// Validate a machine file
bool valid = Loader::validate_json("machines/tractor.json");
```

## Best Practices

1. **Coordinate Calculations**: Always calculate positions relative to chassis center (0,0)
2. **Wheel Order**: List wheels from front to back, right then left
3. **Control Arrays**: Must match the number and order of wheels
4. **Sections**: Only karosseries with `sections > 0` can perform work
5. **Physics**: Set `has_physics: false` for decorative parts
6. **Hitches**: Use standard names (`front_hitch`, `rear_hitch`) for compatibility