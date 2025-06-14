#include "multiverse/robot/sensors/lidar_sensor.hpp"
#include <random>
#include <cmath>
#include <algorithm>

namespace mvs {
    
    LIDARSensor::LIDARSensor(std::shared_ptr<muli::World> world,
                           ScanPattern pattern,
                           double frequency,
                           double min_r,
                           double max_r,
                           double h_fov,
                           double h_res)
        : physics_world(world)
        , scan_pattern(pattern)
        , update_frequency(frequency)
        , next_update_time(0.0)
        , min_range(min_r)
        , max_range(max_r)
        , range_accuracy(0.01)
        , horizontal_fov(h_fov * M_PI / 180.0)  // Convert to radians
        , vertical_fov(0.0)
        , horizontal_resolution(h_res * M_PI / 180.0)  // Convert to radians
        , vertical_resolution(1.0 * M_PI / 180.0)
        , current_scan_angle(0.0)
        , current_beam_index(0)
        , scan_in_progress(false)
        , scan_start_time(0.0)
        , range_noise_std(0.01)
        , beam_divergence(0.1 * M_PI / 180.0)  // 0.1 degrees
        , dropout_probability(0.01)
        , false_positive_rate(0.001)
        , simulate_weather(false)
        , rain_intensity(0.0)
        , fog_density(0.0)
        , dust_level(0.0)
    {
        // Initialize collision filter to detect all objects
        filter.bit = 0xFFFFFFFF;
        filter.mask = 0xFFFFFFFF;
        
        // Initialize sensor offset to identity
        sensor_offset = concord::Pose();
        
        initialize_scan_parameters();
    }
    
    void LIDARSensor::update(double dt) {
        last_update_time += dt;
        
        // Check if it's time for an update
        if (last_update_time >= next_update_time) {
            // Update scan based on pattern
            switch (scan_pattern) {
                case ScanPattern::CIRCULAR_2D:
                    update_circular_2d_scan(dt);
                    break;
                case ScanPattern::SECTOR_2D:
                    update_sector_2d_scan(dt);
                    break;
                case ScanPattern::MULTI_LAYER_3D:
                    update_multi_layer_3d_scan(dt);
                    break;
                case ScanPattern::SPINNING_3D:
                    // Similar to multi-layer but with continuous rotation
                    update_multi_layer_3d_scan(dt);
                    break;
                case ScanPattern::SOLID_STATE_3D:
                    // Instantaneous 3D scan
                    update_multi_layer_3d_scan(dt);
                    break;
            }
            
            // Update timestamp
            current_data.timestamp = std::chrono::system_clock::now();
            
            // Mark data as valid
            data_valid = true;
            
            // Schedule next update
            next_update_time = last_update_time + (1.0 / update_frequency);
        }
    }
    
    void LIDARSensor::set_robot_pose(const concord::Pose& pose) {
        robot_pose = pose;
    }
    
    void* LIDARSensor::get_data() {
        return &current_data;
    }
    
    std::string LIDARSensor::get_type() const {
        return "LIDAR";
    }
    
    bool LIDARSensor::is_data_valid() const {
        return data_valid;
    }
    
    double LIDARSensor::get_frequency() const {
        return update_frequency;
    }
    
    const LIDARData& LIDARSensor::get_lidar_data() const {
        return current_data;
    }
    
    void LIDARSensor::set_sensor_offset(const concord::Pose& offset) {
        sensor_offset = offset;
    }
    
    void LIDARSensor::configure_range(double min_r, double max_r, double accuracy) {
        min_range = min_r;
        max_range = max_r;
        range_accuracy = accuracy;
        
        current_data.min_range = min_range;
        current_data.max_range = max_range;
    }
    
    void LIDARSensor::configure_angular(double h_fov, double v_fov, double h_res, double v_res) {
        horizontal_fov = h_fov * M_PI / 180.0;
        vertical_fov = v_fov * M_PI / 180.0;
        horizontal_resolution = h_res * M_PI / 180.0;
        vertical_resolution = v_res * M_PI / 180.0;
        
        initialize_scan_parameters();
    }
    
    void LIDARSensor::configure_noise(double range_noise, double beam_div, double dropout) {
        range_noise_std = range_noise;
        beam_divergence = beam_div * M_PI / 180.0;
        dropout_probability = dropout;
    }
    
    void LIDARSensor::set_environmental_conditions(double rain, double fog, double dust) {
        rain_intensity = std::clamp(rain, 0.0, 1.0);
        fog_density = std::clamp(fog, 0.0, 1.0);
        dust_level = std::clamp(dust, 0.0, 1.0);
    }
    
    void LIDARSensor::enable_weather_simulation(bool enable) {
        simulate_weather = enable;
    }
    
    void LIDARSensor::set_collision_filter(const muli::CollisionFilter& new_filter) {
        filter = new_filter;
    }
    
    void LIDARSensor::start_scan() {
        scan_in_progress = true;
        current_scan_angle = 0.0;
        current_beam_index = 0;
        scan_start_time = last_update_time;
        current_data.scan_status = LIDARData::ScanStatus::SCANNING;
        
        // Clear previous data
        current_data.clear();
        current_data.reserve(num_horizontal_beams * num_vertical_layers);
    }
    
    bool LIDARSensor::is_scanning() const {
        return scan_in_progress;
    }
    
    double LIDARSensor::get_scan_progress() const {
        if (!scan_in_progress) return 1.0;
        
        int total_beams = num_horizontal_beams * num_vertical_layers;
        return static_cast<double>(current_beam_index) / total_beams;
    }
    
    void LIDARSensor::get_point_cloud(std::vector<muli::Vec2>& points, bool transform_to_world) const {
        points.clear();
        points.reserve(current_data.ranges.size());
        
        for (size_t i = 0; i < current_data.ranges.size(); ++i) {
            if (current_data.valid[i]) {
                if (transform_to_world) {
                    muli::Vec2 world_point = calculate_world_position(
                        current_data.angles[i], 
                        current_data.ranges[i]
                    );
                    points.push_back(world_point);
                } else {
                    // Return in sensor frame
                    double x = current_data.ranges[i] * std::cos(current_data.angles[i]);
                    double y = current_data.ranges[i] * std::sin(current_data.angles[i]);
                    points.push_back(muli::Vec2(x, y));
                }
            }
        }
    }
    
    double LIDARSensor::raycast_beam(double angle, double layer_height) {
        if (!physics_world) {
            return max_range;
        }
        
        // Calculate sensor position and orientation in world frame
        muli::Vec2 sensor_pos = get_sensor_world_position();
        double sensor_orientation = get_sensor_world_orientation();
        
        // Calculate beam direction in world frame
        double world_angle = sensor_orientation + angle;
        muli::Vec2 beam_direction(std::cos(world_angle), std::sin(world_angle));
        
        // Calculate end point of ray
        muli::Vec2 ray_end = sensor_pos + beam_direction * max_range;
        
        // Perform raycast
        double closest_distance = max_range;
        bool hit_found = false;
        
        hit_found = physics_world->RayCastClosest(
            sensor_pos, ray_end, 0.0f,
            [&](muli::Collider* collider, const muli::Vec2& point, const muli::Vec2& normal, float fraction) -> void {
                // Calculate distance to hit point
                muli::Vec2 hit_vector = point - sensor_pos;
                double distance = hit_vector.Length();
                
                // Check if hit is within valid range
                if (distance >= min_range && distance <= max_range) {
                    closest_distance = distance;
                }
            }
        );
        
        // Apply environmental effects if enabled
        if (simulate_weather && hit_found) {
            closest_distance = apply_environmental_effects(closest_distance, angle);
        }
        
        // Add measurement noise
        if (hit_found) {
            closest_distance = add_measurement_noise(closest_distance);
        }
        
        return closest_distance;
    }
    
    void LIDARSensor::update_circular_2d_scan(double dt) {
        if (!scan_in_progress) {
            start_scan();
        }
        
        // Calculate how many beams to process this update
        double beams_per_second = num_horizontal_beams * update_frequency;
        double beams_this_update = beams_per_second * dt;
        int beams_to_process = std::max(1, static_cast<int>(beams_this_update));
        
        for (int i = 0; i < beams_to_process && current_beam_index < num_horizontal_beams; ++i) {
            // Calculate beam angle
            double angle = current_scan_angle;
            
            // Perform raycast
            double range = raycast_beam(angle);
            
            // Check if measurement should be dropped
            bool valid = !should_drop_measurement() && range < max_range;
            
            // Store measurement
            current_data.ranges.push_back(range);
            current_data.angles.push_back(angle);
            current_data.valid.push_back(valid);
            
            // Calculate and store point coordinates
            if (valid) {
                double x = range * std::cos(angle);
                double y = range * std::sin(angle);
                current_data.points_x.push_back(x);
                current_data.points_y.push_back(y);
                current_data.points_z.push_back(0.0);  // 2D scan
                
                // Simulate intensity based on range and surface properties
                double intensity = std::max(0.0, 1.0 - (range / max_range));
                current_data.intensities.push_back(intensity);
                current_data.num_returns.push_back(1);
            } else {
                current_data.points_x.push_back(0.0);
                current_data.points_y.push_back(0.0);
                current_data.points_z.push_back(0.0);
                current_data.intensities.push_back(0.0);
                current_data.num_returns.push_back(0);
            }
            
            // Advance to next beam
            current_beam_index++;
            current_scan_angle += horizontal_resolution;
        }
        
        // Check if scan is complete
        if (current_beam_index >= num_horizontal_beams) {
            scan_in_progress = false;
            current_data.scan_status = LIDARData::ScanStatus::COMPLETE;
            current_data.scan_duration = last_update_time - scan_start_time;
        }
    }
    
    void LIDARSensor::update_sector_2d_scan(double dt) {
        // Similar to circular scan but with limited FOV
        if (!scan_in_progress) {
            start_scan();
            current_scan_angle = -horizontal_fov / 2.0;  // Start from left edge
        }
        
        // Calculate how many beams to process this update
        double beams_per_second = num_horizontal_beams * update_frequency;
        double beams_this_update = beams_per_second * dt;
        int beams_to_process = std::max(1, static_cast<int>(beams_this_update));
        
        for (int i = 0; i < beams_to_process && current_beam_index < num_horizontal_beams; ++i) {
            // Calculate beam angle (relative to sensor)
            double angle = current_scan_angle;
            
            // Perform raycast
            double range = raycast_beam(angle);
            
            // Check if measurement should be dropped
            bool valid = !should_drop_measurement() && range < max_range;
            
            // Store measurement
            current_data.ranges.push_back(range);
            current_data.angles.push_back(angle);
            current_data.valid.push_back(valid);
            
            // Calculate and store point coordinates
            if (valid) {
                double x = range * std::cos(angle);
                double y = range * std::sin(angle);
                current_data.points_x.push_back(x);
                current_data.points_y.push_back(y);
                current_data.points_z.push_back(0.0);
                
                double intensity = std::max(0.0, 1.0 - (range / max_range));
                current_data.intensities.push_back(intensity);
                current_data.num_returns.push_back(1);
            } else {
                current_data.points_x.push_back(0.0);
                current_data.points_y.push_back(0.0);
                current_data.points_z.push_back(0.0);
                current_data.intensities.push_back(0.0);
                current_data.num_returns.push_back(0);
            }
            
            // Advance to next beam
            current_beam_index++;
            current_scan_angle += horizontal_resolution;
        }
        
        // Check if scan is complete
        if (current_beam_index >= num_horizontal_beams) {
            scan_in_progress = false;
            current_data.scan_status = LIDARData::ScanStatus::COMPLETE;
            current_data.scan_duration = last_update_time - scan_start_time;
        }
    }
    
    void LIDARSensor::update_multi_layer_3d_scan(double dt) {
        // 3D scanning with multiple vertical layers
        if (!scan_in_progress) {
            start_scan();
        }
        
        // For 3D scans, we typically scan all angles for one layer, then move to next layer
        int total_beams = num_horizontal_beams * num_vertical_layers;
        double beams_per_second = total_beams * update_frequency;
        double beams_this_update = beams_per_second * dt;
        int beams_to_process = std::max(1, static_cast<int>(beams_this_update));
        
        for (int i = 0; i < beams_to_process && current_beam_index < total_beams; ++i) {
            // Calculate current layer and horizontal angle
            int layer = current_beam_index / num_horizontal_beams;
            int horizontal_index = current_beam_index % num_horizontal_beams;
            
            // Calculate angles
            double horizontal_angle = horizontal_index * horizontal_resolution;
            double vertical_angle = (layer - num_vertical_layers / 2.0) * vertical_resolution;
            double layer_height = max_range * std::tan(vertical_angle);  // Simplified
            
            // Perform raycast for this beam
            double range = raycast_beam(horizontal_angle, layer_height);
            
            // Check if measurement should be dropped
            bool valid = !should_drop_measurement() && range < max_range;
            
            // Store measurement
            current_data.ranges.push_back(range);
            current_data.angles.push_back(horizontal_angle);
            current_data.valid.push_back(valid);
            
            // Calculate 3D point coordinates
            if (valid) {
                double x = range * std::cos(horizontal_angle) * std::cos(vertical_angle);
                double y = range * std::sin(horizontal_angle) * std::cos(vertical_angle);
                double z = range * std::sin(vertical_angle);
                
                current_data.points_x.push_back(x);
                current_data.points_y.push_back(y);
                current_data.points_z.push_back(z);
                
                double intensity = std::max(0.0, 1.0 - (range / max_range));
                current_data.intensities.push_back(intensity);
                current_data.num_returns.push_back(1);
            } else {
                current_data.points_x.push_back(0.0);
                current_data.points_y.push_back(0.0);
                current_data.points_z.push_back(0.0);
                current_data.intensities.push_back(0.0);
                current_data.num_returns.push_back(0);
            }
            
            current_beam_index++;
        }
        
        // Check if scan is complete
        if (current_beam_index >= total_beams) {
            scan_in_progress = false;
            current_data.scan_status = LIDARData::ScanStatus::COMPLETE;
            current_data.scan_duration = last_update_time - scan_start_time;
        }
    }
    
    double LIDARSensor::apply_environmental_effects(double range, double angle) {
        double modified_range = range;
        
        // Rain effects - reduces range and adds noise
        if (rain_intensity > 0.0) {
            double rain_attenuation = 1.0 - (rain_intensity * 0.1);  // 10% max reduction
            modified_range *= rain_attenuation;
        }
        
        // Fog effects - significantly reduces range
        if (fog_density > 0.0) {
            double fog_attenuation = 1.0 - (fog_density * 0.5);  // 50% max reduction
            modified_range *= fog_attenuation;
        }
        
        // Dust effects - slight range reduction and noise
        if (dust_level > 0.0) {
            double dust_attenuation = 1.0 - (dust_level * 0.05);  // 5% max reduction
            modified_range *= dust_attenuation;
        }
        
        return std::max(min_range, modified_range);
    }
    
    double LIDARSensor::add_measurement_noise(double range) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        // Add Gaussian noise proportional to range
        std::normal_distribution<double> noise(0.0, range_noise_std);
        double noisy_range = range + noise(gen);
        
        // Add range-dependent noise (longer ranges are noisier)
        double range_factor = range / max_range;
        std::normal_distribution<double> range_noise(0.0, range_noise_std * range_factor);
        noisy_range += range_noise(gen);
        
        return std::max(min_range, std::min(max_range, noisy_range));
    }
    
    bool LIDARSensor::should_drop_measurement() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<double> dist(0.0, 1.0);
        
        return dist(gen) < dropout_probability;
    }
    
    muli::Vec2 LIDARSensor::calculate_world_position(double angle, double range, double height) const {
        // Get sensor position in world frame
        muli::Vec2 sensor_pos = get_sensor_world_position();
        double sensor_orientation = get_sensor_world_orientation();
        
        // Calculate point in world frame
        double world_angle = sensor_orientation + angle;
        muli::Vec2 point_offset(range * std::cos(world_angle), range * std::sin(world_angle));
        
        return sensor_pos + point_offset;
    }
    
    muli::Vec2 LIDARSensor::get_sensor_world_position() const {
        // Transform sensor offset to world frame
        double cos_yaw = std::cos(robot_pose.angle.yaw);
        double sin_yaw = std::sin(robot_pose.angle.yaw);
        
        // Rotate sensor offset by robot orientation
        double world_x = robot_pose.point.enu.x + 
                        (sensor_offset.point.enu.x * cos_yaw - sensor_offset.point.enu.y * sin_yaw);
        double world_y = robot_pose.point.enu.y + 
                        (sensor_offset.point.enu.x * sin_yaw + sensor_offset.point.enu.y * cos_yaw);
        
        return muli::Vec2(world_x, world_y);
    }
    
    double LIDARSensor::get_sensor_world_orientation() const {
        return robot_pose.angle.yaw + sensor_offset.angle.yaw;
    }
    
    void LIDARSensor::initialize_scan_parameters() {
        // Calculate number of beams based on FOV and resolution
        num_horizontal_beams = static_cast<int>(std::ceil(horizontal_fov / horizontal_resolution));
        
        if (vertical_fov > 0.0) {
            num_vertical_layers = static_cast<int>(std::ceil(vertical_fov / vertical_resolution));
        } else {
            num_vertical_layers = 1;  // 2D scan
        }
        
        // Update data structure
        current_data.num_beams = num_horizontal_beams * num_vertical_layers;
        current_data.angular_resolution = horizontal_resolution;
        current_data.min_range = min_range;
        current_data.max_range = max_range;
        current_data.scan_duration = 1.0 / update_frequency;
        current_data.beam_interval = current_data.scan_duration / current_data.num_beams;
        
        // Ensure minimum values
        if (num_horizontal_beams < 1) num_horizontal_beams = 1;
        if (num_vertical_layers < 1) num_vertical_layers = 1;
    }
    
} // namespace mvs
