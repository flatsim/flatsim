# Multiverse Sensor System

## Overview

The Multiverse robotics simulation framework includes a comprehensive sensor system designed for high-fidelity robotics simulation. The system provides a unified interface for various sensor types commonly used in robotics applications.

## Architecture

### Base Sensor Interface

All sensors inherit from the virtual base class `Sensor` which provides a standardized interface:

```cpp
class Sensor {
public:
    virtual ~Sensor() = default;
    virtual void update(double dt) = 0;
    virtual void set_robot_pose(const concord::Pose& pose) = 0;
    virtual bool is_data_valid() const = 0;
    virtual std::string get_type() const = 0;
    virtual double get_frequency() const = 0;
    virtual nlohmann::json get_data() const = 0;
};
```

### Key Features

- **Unified Interface**: All sensors implement the same base methods
- **High-Frequency Updates**: Support for different update rates per sensor type
- **Physics Integration**: Sensors can interact with the Muli physics engine
- **Data Validation**: Built-in data validity checking
- **JSON Serialization**: Standard data format for integration

## Implemented Sensors

### 1. GPS Sensor with RTK Capabilities

**File**: `include/multiverse/robot/sensors/gps_sensor.hpp`

#### Features
- **Double Precision Coordinates**: High-accuracy latitude/longitude representation
- **RTK Support**: Real-Time Kinematic positioning with multiple fix types
- **Satellite Tracking**: Configurable satellite count and signal quality
- **Noise Simulation**: Realistic GPS error modeling
- **Multiple Fix Types**: NO_FIX, SINGLE, DGPS, RTK_FLOAT, RTK_FIXED

#### Usage Example
```cpp
auto gps = sensors::create_sensor<GPSSensor>(
    10.0,    // 10 Hz update rate
    true,    // Enable RTK
    3.0,     // 3m base accuracy
    0.02     // 2cm RTK accuracy
);

// Configure the sensor
auto* gps_sensor = static_cast<GPSSensor*>(gps.get());
gps_sensor->set_satellite_count(12);
gps_sensor->configure_noise(0.05, 0.01);

// Update and get data
gps_sensor->set_robot_pose(robot_pose);
gps_sensor->update(0.1);

if (gps_sensor->is_data_valid()) {
    const GPSData& data = gps_sensor->get_gps_data();
    std::cout << "Position: " << data.latitude << ", " << data.longitude << std::endl;
}
```

#### Data Structure
```cpp
struct GPSData {
    double latitude;           // Degrees
    double longitude;          // Degrees
    double altitude;           // Meters above sea level
    double horizontal_accuracy; // Meters
    double vertical_accuracy;   // Meters
    int num_satellites;        // Number of satellites in view
    RTKStatus rtk_status;      // RTK fix quality
    double timestamp;          // Seconds since epoch
};
```

### 2. IMU Sensor (9-DOF)

**File**: `include/multiverse/robot/sensors/imu_sensor.hpp`

#### Features
- **9-Axis Measurements**: Accelerometer, Gyroscope, Magnetometer
- **Orientation Estimation**: Quaternion-based orientation output
- **Calibration Status**: Tracks sensor calibration state
- **Temperature Compensation**: Built-in temperature monitoring
- **Bias Modeling**: Configurable sensor bias and noise

#### Usage Example
```cpp
auto imu = sensors::create_sensor<IMUSensor>(
    100.0,    // 100 Hz update rate
    0.01,     // Low noise
    0.05      // Small bias
);

// Update and get data
imu->set_robot_pose(robot_pose);
imu->update(0.01);

if (imu->is_data_valid()) {
    const IMUData& data = static_cast<IMUSensor*>(imu.get())->get_imu_data();
    std::cout << "Orientation: " << data.orientation.w << ", " 
              << data.orientation.x << ", " << data.orientation.y << ", " 
              << data.orientation.z << std::endl;
}
```

#### Data Structure
```cpp
struct IMUData {
    Vector3 acceleration;      // m/s² in body frame
    Vector3 angular_velocity;  // rad/s in body frame
    Vector3 magnetic_field;    // μT in body frame
    Quaternion orientation;    // Estimated orientation
    double temperature;        // °C
    CalibrationStatus calibration_status;
    double accuracy;           // Orientation accuracy estimate
    double timestamp;          // Seconds since epoch
};
```

### 3. LIDAR Sensor with Physics Integration

**File**: `include/multiverse/robot/sensors/lidar_sensor.hpp`

#### Features
- **Multiple Scan Patterns**: 2D Horizontal, 2D Vertical, Full 3D
- **Physics-Based Raycasting**: Integration with Muli physics engine
- **Environmental Effects**: Weather attenuation, dust simulation
- **Configurable Parameters**: Range, FOV, resolution
- **Point Cloud Output**: 3D points with intensity data

#### Scan Patterns
- **HORIZONTAL_2D**: Traditional 2D LIDAR scanning horizontally
- **VERTICAL_2D**: Vertical plane scanning for terrain mapping
- **FULL_3D**: Complete 3D point cloud generation

#### Usage Example
```cpp
auto lidar = sensors::create_sensor<LIDARSensor>(
    10.0,     // 10 Hz update rate
    LIDARSensor::ScanPattern::HORIZONTAL_2D,
    100.0,    // 100m max range
    360.0,    // 360 degree FOV
    0.25      // 0.25 degree resolution
);

// Configure environmental effects
auto* lidar_sensor = static_cast<LIDARSensor*>(lidar.get());
lidar_sensor->set_weather_attenuation(0.95);
lidar_sensor->set_dust_level(0.1);

// Update and get data
lidar_sensor->set_robot_pose(robot_pose);
lidar_sensor->update(0.1);

if (lidar_sensor->is_data_valid()) {
    const LIDARData& data = lidar_sensor->get_lidar_data();
    std::cout << "Points detected: " << data.points.size() << std::endl;
}
```

#### Data Structure
```cpp
struct LIDARData {
    std::vector<Point3D> points;      // 3D point cloud
    std::vector<float> intensities;   // Return intensity values
    ScanPattern scan_pattern;         // Scan type used
    double max_range;                 // Maximum detection range
    double field_of_view;             // Field of view in degrees
    double angular_resolution;        // Angular resolution in degrees
    double timestamp;                 // Seconds since epoch
};
```

## Sensor Factory System

The sensor system includes a factory pattern for easy sensor creation:

```cpp
namespace sensors {
    template<typename SensorType, typename... Args>
    std::unique_ptr<Sensor> create_sensor(Args&&... args) {
        return std::make_unique<SensorType>(std::forward<Args>(args)...);
    }
}
```

## Integration with Robot Class

Sensors can be easily integrated with the Robot class:

```cpp
class Robot {
private:
    std::vector<std::unique_ptr<Sensor>> sensors_;
    
public:
    template<typename SensorType, typename... Args>
    void add_sensor(Args&&... args) {
        sensors_.push_back(sensors::create_sensor<SensorType>(std::forward<Args>(args)...));
    }
    
    void update_sensors(double dt) {
        for (auto& sensor : sensors_) {
            sensor->set_robot_pose(pose_);
            sensor->update(dt);
        }
    }
};
```

## Multi-Sensor Management

The system supports managing multiple sensors with different update frequencies:

```cpp
// Create sensors with different frequencies
auto gps = sensors::create_sensor<GPSSensor>(1.0);     // 1 Hz
auto imu = sensors::create_sensor<IMUSensor>(100.0);   // 100 Hz
auto lidar = sensors::create_sensor<LIDARSensor>(10.0); // 10 Hz

// Update based on their individual frequencies
for (auto& sensor : sensors) {
    if (should_update(sensor.get(), current_time)) {
        sensor->update(dt);
    }
}
```

## Performance Considerations

### Update Frequencies
- **GPS**: Typically 1-10 Hz for real-world applications
- **IMU**: 100-1000 Hz for high-rate inertial navigation
- **LIDAR**: 5-40 Hz depending on scan density and range

### Memory Usage
- LIDAR sensors can generate large point clouds (10K-1M+ points)
- Consider point cloud filtering and downsampling for performance
- Use appropriate data structures for spatial queries

### Threading
The sensor system is designed to be thread-safe for concurrent updates:
- Each sensor maintains its own internal state
- No shared mutable state between sensor instances
- Safe for multi-threaded robot simulations

## Example Applications

### 1. Autonomous Navigation
```cpp
// GPS for global positioning
auto gps = sensors::create_sensor<GPSSensor>(5.0, true);

// IMU for orientation and motion sensing
auto imu = sensors::create_sensor<IMUSensor>(100.0);

// LIDAR for obstacle detection
auto lidar = sensors::create_sensor<LIDARSensor>(10.0, 
    LIDARSensor::ScanPattern::HORIZONTAL_2D, 50.0, 270.0, 0.5);
```

### 2. Mapping and SLAM
```cpp
// High-resolution 3D LIDAR for detailed mapping
auto lidar_3d = sensors::create_sensor<LIDARSensor>(5.0,
    LIDARSensor::ScanPattern::FULL_3D, 100.0, 360.0, 0.1);

// IMU for motion compensation
auto imu = sensors::create_sensor<IMUSensor>(200.0, 0.005, 0.01);

// GPS for global reference
auto gps = sensors::create_sensor<GPSSensor>(1.0, true, 1.0, 0.01);
```

### 3. Precision Agriculture
```cpp
// RTK GPS for centimeter-level accuracy
auto rtk_gps = sensors::create_sensor<GPSSensor>(10.0, true, 2.0, 0.02);

// Vertical LIDAR for crop height measurement
auto crop_lidar = sensors::create_sensor<LIDARSensor>(20.0,
    LIDARSensor::ScanPattern::VERTICAL_2D, 10.0, 60.0, 0.1);
```

## Building and Testing

### Build System
The sensor system is integrated into the main CMake build:

```bash
cd /doc/code/multiverse
./run.sh b  # Build the project
```

### Running Examples
```bash
# The sensor_example.cpp demonstrates all sensor types
# (Note: Currently disabled with #if 0, enable for testing)
```

### Unit Testing
Consider adding unit tests for:
- Sensor data validation
- Coordinate transformations
- Physics integration
- Multi-sensor synchronization

## Future Enhancements

### Planned Features
1. **Camera Sensors**: RGB and depth cameras
2. **Radar Sensors**: For all-weather detection
3. **Ultrasonic Sensors**: For close-range obstacle detection
4. **Force/Torque Sensors**: For manipulation tasks

### Performance Optimizations
1. **SIMD Acceleration**: Vector operations for point cloud processing
2. **GPU Acceleration**: CUDA support for LIDAR ray tracing
3. **Memory Pooling**: Reduce allocation overhead for high-frequency sensors
4. **Sensor Fusion**: Built-in Kalman filtering and sensor fusion algorithms

### Integration Improvements
1. **ROS2 Bridge**: Direct integration with ROS2 sensor messages
2. **Gazebo Plugin**: Plugin interface for Gazebo simulation
3. **Real Hardware**: Bridge to real sensor hardware for HIL testing

## Troubleshooting

### Common Issues

1. **Compilation Errors**
   - Ensure all dependencies are installed (Muli physics engine, Concord)
   - Check that the CMakeLists.txt includes all sensor source files

2. **Runtime Errors**
   - Verify robot pose is set before calling update()
   - Check that sensor frequencies are reasonable (> 0 Hz)
   - Ensure physics world is properly initialized for LIDAR

3. **Performance Issues**
   - Reduce LIDAR resolution or range for better performance
   - Use appropriate update frequencies for each sensor type
   - Consider spatial partitioning for complex environments

### Debug Output
Enable debug output by setting log levels:
```cpp
spdlog::set_level(spdlog::level::debug);
```

## Contributing

When adding new sensor types:

1. Inherit from the base `Sensor` class
2. Implement all virtual methods
3. Add appropriate data structures
4. Include factory function support
5. Add comprehensive examples
6. Update this documentation

For questions or contributions, please refer to the main project documentation.
