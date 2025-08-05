# Machine and Implement System Architecture

This document provides a comprehensive analysis of AgOpenGPS machine/implement architecture and how to enhance Flatsim's existing slave vehicle system for agricultural implements.

## 🎯 **Key Finding: Flatsim Already Has Excellent Slave Vehicle Architecture!**

Flatsim's **master-slave-follower** system with hitch-based connections and section control is perfect for implements. We just need to add AgOpenGPS's advanced section control logic.

## Table of Contents
1. [Current Flatsim Slave Vehicle System](#current-flatsim-slave-vehicle-system)
2. [AgOpenGPS Machine/Implement Overview](#agopengps-machineimplement-overview)
3. [Gap Analysis](#gap-analysis)
4. [Enhancement Roadmap](#enhancement-roadmap)
5. [Code Examples](#code-examples)

## Current Flatsim Slave Vehicle System ✅

### **Architecture Overview**
Flatsim has a sophisticated slave vehicle system perfect for implements:

```cpp
// Three-role system
enum class RobotRole { 
    MASTER,     // Tractor (drives, navigates)
    SLAVE,      // Implement (awaiting connection)  
    FOLLOWER    // Connected implement (follows master)
};

// Chain management with physics
class ChainManager {
    std::vector<Robot*> connected_followers;        // Connected implements
    std::vector<muli::RevoluteJoint*> connection_joints; // Physics joints
    Robot* master_robot = nullptr;                  // Master reference
    FollowerCapabilities follower_capabilities;     // What this robot can do
};
```

### **Connection System** (`chain.cpp:69-144`)
1. **Master scans** for nearby `SLAVE` vehicles
2. **Hitch overlap detection** (50% minimum overlap required)  
3. **Physics joint creation** using `muli::RevoluteJoint`
4. **Role transition**: `SLAVE` → `FOLLOWER` + adopts master's color
5. **Capability updates** for both vehicles

### **Section Control System** (`section.hpp/cpp`)
```cpp
class Section {
    std::string name;           // Section identifier
    bool working = false;       // Current on/off state
    int section_id;            // Unique ID
    concord::Bound bound;      // Physical dimensions
    concord::Pose pose;        // World position
    
    void toggle_work();        // Individual section control
    void tick(dt, trans_pose); // Follow parent chassis
};
```

### **Current Usage** (`mvs.cpp:213-214`)
```cpp
// Toggle all sections except middle one  
robot.toggle_all_except_section_work("front", 2);
```

### **What Works Perfectly:**
- ✅ **Tractor-Implement Connection**: Physics-based hitching
- ✅ **Multi-implement Chains**: Can connect multiple implements
- ✅ **Individual Section Control**: Each section toggles independently  
- ✅ **Visual Feedback**: Solid=working, wireframe=off
- ✅ **Disconnection System**: Can disconnect at any position
- ✅ **Implement-as-Vehicle**: Perfect architecture already in place

### **Implement = Slave Vehicle with Sections**

Your trailer example shows the perfect implement architecture:

```json
// trailer.json - Transport implement
{
  "info": {
    "type": "trailer",        // Vehicle type
    "role": "SLAVE"          // Slave vehicle (gets pulled)
  },
  "wheels": [...],           // Has wheels (vehicle)
  "controls": {              // Vehicle controls (0 for implements)
    "steering": {"max_angles": [0, 0]},
    "throttle": {"max_values": [0.0, 0.0]}
  },
  "hitches": {
    "front_hitch": {"is_master": false},  // Connects to tractor
    "rear_hitch": {"is_master": true}     // Can pull another implement
  }
}
```

**Agricultural implements** follow the same pattern but with **working sections**:

```json
// seeder.json - Working implement
{
  "info": {
    "type": "seeder",         // Vehicle type (implement)
    "role": "SLAVE"          // Slave vehicle
  },
  "wheels": [...],           // Has wheels (implement vehicle)
  "karosseries": [
    {
      "name": "toolbar",
      "sections": 8          // Key difference: working sections
    }
  ],
  "hitches": {
    "front_hitch": {"is_master": false}  // Connects to tractor
  }
}
```

## AgOpenGPS Machine/Implement Overview

### Vehicle Configuration
AgOpenGPS models agricultural vehicles with sophisticated configuration:

```csharp
// Vehicle Types
enum VehicleType {
    Tractor = 0,     // Standard tractor
    Harvester = 1,   // Self-propelled harvester
    Articulated = 2  // Articulated tractor
}

// Key Vehicle Parameters
- AntennaHeight      // GPS antenna height above ground
- AntennaPivot       // Forward/back from pivot axle
- AntennaOffset      // Left/right from centerline
- Wheelbase         // Distance between axles
- TrackWidth        // Distance between wheels
- hitchLength       // Distance to rear hitch point
```

### Tool/Implement System

AgOpenGPS supports multiple implement configurations:

1. **Tool Types**:
   - **Rear Fixed**: Rigidly attached to rear (3-point hitch)
   - **Trailing**: Pivots at hitch point (pulled implements)
   - **TBT (Tow Between Tow)**: Implement between tractor and another implement
   - **Front Fixed**: Front-mounted implements

2. **Key Tool Parameters**:
   ```csharp
   - toolWidth                    // Total working width
   - toolOffset                   // Left/right offset from center
   - toolTrailingHitchLength      // Distance from hitch to tool pivot
   - trailingToolToPivotLength    // Distance from tool pivot to sections
   - toolLookAheadOn/Off          // Look-ahead distances for section control
   - toolOffDelay                 // Delay before turning sections off
   ```

### Section Control System

Sections are the individual controllable units across the implement width:

1. **Section Configuration**:
   - Up to 16 individual sections
   - Each section has left/right positions
   - Sections can be grouped into zones
   - Variable width sections supported

2. **Section States**:
   ```csharp
   enum btnStates { Off, Auto, On }
   
   // Each section tracks:
   - isSectionOn           // Current on/off state
   - sectionBtnState       // Manual/Auto/On control state
   - positionLeft/Right    // Physical position on implement
   - isInBoundary          // Inside field boundary
   - isInHeadlandArea      // In headland zone
   - speedPixels           // Ground speed at section
   ```

3. **Section Control Logic**:
   - Look-ahead for turning on (entering worked area)
   - Look-ahead for turning off (leaving field)
   - Speed-based control (minimum speed cutoff)
   - Boundary awareness
   - Headland management

### Hydraulic Lift Control

AgOpenGPS simulates hydraulic implement control:

```csharp
- hydLiftLookAheadTime          // Time-based look-ahead
- isHydLiftOn                   // Hydraulic control active
- hydLiftLookAheadDistanceLeft  // Distance-based look-ahead
- hydLiftLookAheadDistanceRight
```

### Tram Line System

Permanent wheel tracks for repeated passes:

```csharp
- tramWidth            // Width between tram lines
- halfWheelTrack       // Half of vehicle track width
- passes               // Number of passes between trams
- isOuter              // Inner or outer tram mode
- displayMode          // Off/All/Lines/Outer
```

## Current Flatsim Machine System

### Existing Features

1. **Machine Definition** (JSON-based):
   ```json
   {
     "info": { "type", "name", "works_on", "role" },
     "dimensions": { "width", "height" },
     "wheels": [...],
     "karosseries": [{ "sections": N }],  // Work sections
     "hitches": { "rear_hitch", "front_hitch" },
     "tank": { "capacity" },
     "power": { "type", "capacity" }
   }
   ```

2. **Section Implementation**:
   ```cpp
   class Section {
       bool working = false;
       concord::Bound bound;
       void toggle_work();
   }
   ```

3. **Hitching System**:
   - Basic connection points
   - Chain management for connected vehicles
   - Master/Slave/Follower roles

### **Types of Agricultural Implements (All as Slave Vehicles):**

1. **Transport Implements**:
   - **Trailer**: Grain/material transport (your current example)
   - **Tank**: Liquid transport, fuel supply

2. **Field Work Implements** (with sections):
   - **Seeder/Planter**: Plants seeds in rows (8-24 sections)
   - **Sprayer**: Applies pesticides/fertilizers (6-36 sections)
   - **Spreader**: Distributes granular materials (2-8 sections)
   - **Cultivator**: Soil preparation (8-16 sections)
   - **Mower**: Cutting crops/grass (4-12 sections)

3. **Processing Implements**:
   - **Harvester**: Self-propelled crop harvesting
   - **Baler**: Hay/straw baling

All follow the same **vehicle pattern**: wheels, controls (usually 0), hitches, and sections for working implements.

## Gap Analysis

### Major Missing Features

1. **Dynamic Section Control**:
   - No automatic on/off based on field position
   - No look-ahead logic
   - No speed-based control
   - No boundary/headland awareness

2. **Implement Physics**:
   - No pivoting for trailing implements
   - No ground following
   - No lift/lower simulation
   - No weight transfer

3. **Advanced Configuration**:
   - No antenna position modeling
   - No tool offset support
   - No TBT configuration
   - Limited implement types

4. **Control Systems**:
   - No hydraulic simulation
   - No section timing/delays
   - No manual/auto switching
   - No zone grouping

5. **Working Area Tracking**:
   - No coverage mapping
   - No overlap detection
   - No efficiency metrics
   - No persistence

## Proposed Architecture

### Enhanced Machine Model

```cpp
namespace fs {

// Implement types matching AgOpenGPS
enum class ImplementType {
    REAR_FIXED,    // 3-point hitch
    TRAILING,      // Pulled implement
    TBT,           // Tow between tow
    FRONT_FIXED    // Front mounted
};

// Section control states
enum class SectionState {
    OFF,
    AUTO,
    ON
};

// Enhanced section class
class ImplementSection {
public:
    // Position and geometry
    double position_left;
    double position_right;
    concord::Bound bound;
    
    // Control state
    SectionState control_state = SectionState::OFF;
    bool is_on = false;
    
    // Timing
    double on_timer = 0.0;
    double off_timer = 0.0;
    
    // Speed and position
    double speed_mps = 0.0;
    bool in_boundary = true;
    bool in_headland = false;
    
    // Look-ahead points
    concord::Point look_ahead_on;
    concord::Point look_ahead_off;
};

// Implement configuration
class Implement {
public:
    // Basic properties
    ImplementType type;
    std::string name;
    
    // Dimensions
    double width;
    double offset;  // Left/right offset
    
    // Attachment
    double hitch_length;         // Distance to hitch
    double trailing_hitch_length; // For trailing implements
    double tool_to_pivot_length;  // Pivot to sections
    
    // Sections
    std::vector<ImplementSection> sections;
    int num_sections;
    
    // Control parameters
    double look_ahead_on;   // Distance to look ahead for on
    double look_ahead_off;  // Distance to look ahead for off
    double off_delay;       // Seconds delay before off
    double min_speed;       // Minimum speed for operation
    
    // Hydraulic control
    bool hydraulic_lift_enabled;
    double hydraulic_look_ahead_time;
    bool is_lifted = false;
    
    // Physics
    double pivot_angle = 0.0;  // For trailing implements
    double ground_height = 0.0; // Height above ground
    
    // Methods
    void update_sections(const RobotState& state, double dt);
    void calculate_section_positions();
    bool should_section_be_on(int section_idx, const FieldContext& field);
    void apply_hydraulic_control(const FieldContext& field);
};

// Machine configuration matching AgOpenGPS
class MachineConfig {
public:
    // Vehicle properties
    VehicleType vehicle_type;
    
    // Antenna configuration
    double antenna_height;
    double antenna_pivot;   // Forward from pivot
    double antenna_offset;  // Left/right offset
    
    // Attached implements
    std::vector<std::shared_ptr<Implement>> implements;
    
    // Tram lines
    bool tram_lines_enabled;
    double tram_width;
    int tram_passes;
};

}
```

### Section Control Algorithm

```cpp
void Implement::update_sections(const RobotState& state, double dt) {
    for (size_t i = 0; i < sections.size(); ++i) {
        auto& section = sections[i];
        
        // Calculate look-ahead points
        double heading = state.pose.angle.yaw;
        section.look_ahead_on = calculate_look_ahead_point(
            section.bound.get_center(), heading, look_ahead_on);
        section.look_ahead_off = calculate_look_ahead_point(
            section.bound.get_center(), heading, look_ahead_off);
        
        // Check field context at look-ahead points
        bool should_be_on = should_section_be_on(i, field_context);
        
        // Apply control logic based on state
        switch (section.control_state) {
            case SectionState::AUTO:
                // Automatic control with timing
                if (should_be_on && !section.is_on) {
                    section.on_timer += dt;
                    if (section.on_timer > 0.3) { // 300ms delay
                        section.is_on = true;
                        section.off_timer = 0;
                    }
                } else if (!should_be_on && section.is_on) {
                    section.off_timer += dt;
                    if (section.off_timer > off_delay) {
                        section.is_on = false;
                        section.on_timer = 0;
                    }
                }
                break;
                
            case SectionState::ON:
                section.is_on = true;
                break;
                
            case SectionState::OFF:
                section.is_on = false;
                break;
        }
        
        // Speed cutoff
        if (state.velocity < min_speed) {
            section.is_on = false;
        }
    }
}
```

### Integration with Existing Systems

1. **With farmtrax field management**:
   ```cpp
   // Use farmtrax boundaries and headlands
   bool in_boundary = farmtrax::is_point_in_field(look_ahead_point);
   bool in_headland = farmtrax::is_point_in_headland(look_ahead_point);
   ```

2. **With navigation controller**:
   ```cpp
   // Adjust path following based on implement
   if (machine.has_trailing_implement()) {
       controller.set_pivot_offset(implement.tool_to_pivot_length);
   }
   ```

3. **With existing sections**:
   ```cpp
   // Enhance existing Section class
   class Section : public ImplementSection {
       // Keep existing functionality
       // Add new implement features
   };
   ```

## Implementation Roadmap

### Phase 1: Core Implement System (Week 1-2)
1. Create `Implement` base class
2. Port section control logic from AgOpenGPS
3. Implement look-ahead calculations
4. Add timing and state management

### Phase 2: Physics Integration (Week 3)
1. Add trailing implement physics
2. Implement ground following
3. Create hydraulic lift simulation
4. Add weight transfer calculations

### Phase 3: Advanced Features (Week 4)
1. Tram line generation
2. Coverage mapping
3. Multi-implement support (TBT)
4. Efficiency metrics

### Phase 4: Integration (Week 5)
1. Update JSON schema
2. Create UI for configuration
3. Add persistence
4. Write tests and examples

## Code Examples

### Example 1: Creating a Sprayer Implement
```cpp
auto sprayer = std::make_shared<Implement>();
sprayer->type = ImplementType::TRAILING;
sprayer->name = "24m Sprayer";
sprayer->width = 24.0;
sprayer->num_sections = 12;
sprayer->hitch_length = 5.0;
sprayer->trailing_hitch_length = 3.0;
sprayer->look_ahead_on = 2.0;
sprayer->look_ahead_off = 1.0;
sprayer->off_delay = 0.5;
sprayer->calculate_section_positions();

machine.attach_implement(sprayer);
```

### Example 2: Section Control in Action
```cpp
// In simulation loop
void update_machine(Machine& machine, double dt) {
    // Update implement physics
    for (auto& implement : machine.implements) {
        if (implement->type == ImplementType::TRAILING) {
            // Calculate pivot angle based on turning
            implement->pivot_angle = calculate_trailer_angle(
                machine.state, implement->hitch_length);
        }
        
        // Update section control
        implement->update_sections(machine.state, dt);
        
        // Apply hydraulic control if enabled
        if (implement->hydraulic_lift_enabled) {
            implement->apply_hydraulic_control(field_context);
        }
    }
    
    // Update coverage mapping
    coverage_tracker.record_sections(machine.implements);
}
```

### Example 3: JSON Configuration
```json
{
  "implements": [{
    "type": "trailing",
    "name": "24m Sprayer",
    "dimensions": {
      "width": 24.0,
      "hitch_length": 5.0,
      "trailing_hitch_length": 3.0
    },
    "sections": {
      "count": 12,
      "symmetric": true
    },
    "control": {
      "look_ahead_on": 2.0,
      "look_ahead_off": 1.0,
      "off_delay": 0.5,
      "min_speed": 0.5
    },
    "hydraulic": {
      "enabled": true,
      "look_ahead_time": 1.5
    }
  }]
}
```

## Testing Strategy

1. **Unit Tests**:
   - Section position calculations
   - Look-ahead point computation
   - State machine transitions
   - Physics calculations

2. **Integration Tests**:
   - Field boundary interaction
   - Multi-implement configurations
   - Coverage accuracy
   - Performance benchmarks

3. **Validation**:
   - Compare with AgOpenGPS behavior
   - Field testing with GPS data
   - Efficiency metrics validation

## Performance Considerations

1. **Spatial Indexing**: Use R-tree for section-to-field queries
2. **Caching**: Cache boundary checks for look-ahead points
3. **LOD**: Reduce physics detail for distant implements
4. **Threading**: Separate physics and control updates

## Conclusion

Implementing the AgOpenGPS machine/implement system in Flatsim will provide:
- Professional-grade section control
- Realistic implement physics
- Efficient field coverage
- Industry-standard compatibility

The modular design allows incremental implementation while maintaining compatibility with existing Flatsim features.
