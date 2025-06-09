# Sensor System

This directory contains the sensor system for the multiverse robotics simulation framework. The sensor system provides a modular, extensible architecture for simulating various types of sensors that robots can carry.

## Architecture

### Base Sensor Class
All sensors inherit from the abstract `Sensor` base class, which defines the core interface:

```cpp
class Sensor {
public:
    virtual void update(double dt) = 0;
    virtual void set_robot_pose(const concord::Pose& pose) = 0;
    virtual void* get_data() = 0;
    virtual std::string get_type() const = 0;
    virtual bool is_data_valid() const = 0;
    virtual double get_frequency() const = 0;
};
```

### Available Sensors

#### GPS Sensor (`GPSSensor`)
A high-precision GPS sensor with RTK (Real-Time Kinematic) capabilities:

- **Double precision** for RTK-grade accuracy (centimeter-level)
- Configurable update frequency (default: 10 Hz)
- Multiple fix types: NO_FIX, SINGLE, DGPS, RTK_FLOAT, RTK_FIXED
- Realistic noise simulation based on satellite count and RTK status
- Accuracy ranging from 3m (standard GPS) to 2cm (RTK fixed)

**Features:**
- Satellite count simulation affecting accuracy
- RTK base station availability simulation
- Configurable noise parameters
- Velocity estimation
- Timestamp information

## Usage

### Creating Sensors

```cpp
#include "multiverse/robot/sensors.hpp"

// Create a GPS sensor with RTK capabilities
auto gps = mvs::sensors::create_sensor<mvs::GPSSensor>(
    10.0,    // 10 Hz update rate
    true,    // Enable RTK
    3.0,     // 3m base accuracy
    0.02     // 2cm RTK accuracy
);
```

### Adding Sensors to Robots

```cpp
// Add sensor to robot
robot.add_sensor(std::move(gps));

// Or create and add in one step
robot.add_sensor<mvs::GPSSensor>(10.0, true, 3.0, 0.02);
```

### Updating and Reading Sensors

```cpp
// Sensors are automatically updated during robot.tick(dt)
robot.tick(0.1);  // 100ms update

// Get sensor data
auto* gps_sensor = robot.get_sensor<mvs::GPSSensor>("GPS");
if (gps_sensor && gps_sensor->is_data_valid()) {
    const mvs::GPSData& data = gps_sensor->get_gps_data();
    std::cout << "Position: " << data.latitude << ", " << data.longitude << std::endl;
}
```

## File Structure

```
sensors/
├── sensor.hpp              # Base sensor class
├── sensor.cpp              # Base sensor implementation
├── sensors.hpp             # Convenience header including all sensors
├── gps_sensor.hpp          # GPS sensor header
├── gps_sensor.cpp          # GPS sensor implementation
└── README.md               # This file
```

## Adding New Sensors

To add a new sensor type:

1. Create header file `sensors/your_sensor.hpp`
2. Create implementation file `sensors/your_sensor.cpp`
3. Inherit from `mvs::Sensor` base class
4. Implement all pure virtual methods
5. Add to `sensors.hpp` for convenience
6. Add source file to CMakeLists.txt

### Example Sensor Template

```cpp
// your_sensor.hpp
#pragma once
#include "multiverse/robot/sensor.hpp"

namespace mvs {
    struct YourSensorData {
        // Define your sensor's data structure
        double value = 0.0;
        // ... other fields
    };

    class YourSensor : public Sensor {
    private:
        YourSensorData current_data;
        double update_frequency;
        
    public:
        YourSensor(double frequency = 1.0);
        
        void update(double dt) override;
        void set_robot_pose(const concord::Pose& pose) override;
        void* get_data() override;
        std::string get_type() const override;
        bool is_data_valid() const override;
        double get_frequency() const override;
        
        const YourSensorData& get_sensor_data() const;
    };
}
```

## GPS Sensor Details

### RTK Status Levels

1. **NO_FIX** - No GPS fix, very poor accuracy (999m+)
2. **SINGLE** - Standard GPS fix (~3m accuracy)
3. **DGPS** - Differential GPS with corrections (~1m accuracy)
4. **RTK_FLOAT** - RTK with floating point ambiguities (~10cm accuracy)
5. **RTK_FIXED** - RTK with fixed ambiguities (~2cm accuracy)

### Coordinate System

The GPS sensor converts from the robot's local ENU (East-North-Up) coordinate system to WGS84 latitude/longitude coordinates. This allows realistic GPS simulation within the physics simulation environment.

### Noise Simulation

The GPS sensor adds realistic noise based on:
- RTK status (higher accuracy modes have less noise)
- Satellite count (more satellites = better accuracy)
- Configurable base noise parameters

## Future Enhancements

Potential sensors to add:
- IMU (Inertial Measurement Unit)
- LiDAR
- Camera sensors
- Wheel encoders
- Magnetometer/Compass
- Barometric pressure sensor
- Temperature sensors
- Proximity sensors

Each sensor should follow the same pattern and integrate seamlessly with the robot simulation system.
