# Kokkos Integration Plan for Multiverse

This document outlines a comprehensive plan for integrating Kokkos throughout the Multiverse robotics simulation framework, with Layer drawing as the first priority target.

## Overview

Kokkos is already added to the project dependencies (CMakeLists.txt line 30-32). This plan details how to leverage Kokkos for massive parallelization across CPU and GPU architectures.

## Priority 1: Layer Drawing/Painting System

Layer drawing is the ideal starting point because:
- Embarrassingly parallel operations (each grid cell is independent)
- Currently has O(rows × cols) complexity that can benefit from parallelization
- No complex synchronization requirements
- Clear performance bottleneck in the main simulation loop

### Implementation Plan for Layer

#### 1. Add Kokkos Infrastructure to Layer Class

```cpp
// layer.hpp modifications
#pragma once

#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"
#include <Kokkos_Core.hpp>
#include <any>
#include <random>
#include <type_traits>
#include <vector>

namespace fs {

struct GridData {
    pigment::RGB color;
    float data;
};

class Layer {
public:
    LayerInfo info;

private:
    // Keep existing grid for compatibility
    concord::Grid<GridData> grid;
    
    // Add Kokkos views for parallel operations
    using GridDataView = Kokkos::View<GridData**, Kokkos::LayoutRight>;
    using ColorView = Kokkos::View<uint8_t***, Kokkos::LayoutRight>; // rows x cols x rgba
    using FloatView = Kokkos::View<float**, Kokkos::LayoutRight>;
    using BoolView = Kokkos::View<bool**, Kokkos::LayoutRight>;
    
    // Kokkos data structures
    GridDataView kokkos_grid_data;
    ColorView kokkos_colors;     // For direct RGBA access
    FloatView kokkos_data;       // For noise data
    FloatView kokkos_points_x;   // X coordinates of grid points
    FloatView kokkos_points_y;   // Y coordinates of grid points
    
    // Other members remain the same
    std::shared_ptr<rerun::RecordingStream> rec;
    entropy::NoiseGen noise;
    concord::Datum datum;
    std::vector<uint8_t> image;
    std::vector<uint8_t> data_img;
    size_t rows, cols;
    uint freq = 0;
    bool has_noise = false;
    bool use_kokkos = true;  // Flag to enable/disable Kokkos
    
    std::mt19937 rnd;
    std::vector<std::array<float, 3>> enu_corners_;
    std::vector<std::array<float, 3>> polygon_corners_;
    std::vector<rerun::LatLon> wgs_corners_;
    std::vector<rerun::LatLon> polygon_corners_wgs_;
    
    // Add sync methods
    void sync_to_kokkos();
    void sync_from_kokkos();
    void init_kokkos_views();

public:
    Layer() = default;
    Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum);
    
    // Existing methods
    void init(LayerInfo info);
    void tick(float dt);
    void add_noise(bool in_polygon_only = false);
    void paint(pigment::RGB color, concord::Polygon brush);
    void color_field();
    void to_image(std::vector<uint8_t> &image);
    void visualize();
    concord::Grid<uint8_t> get_grid_data() const;
    
    // Add Kokkos-specific versions
    void add_noise_kokkos(bool in_polygon_only = false);
    void paint_kokkos(pigment::RGB color, concord::Polygon brush);
    void color_field_kokkos();
    void to_image_kokkos(std::vector<uint8_t> &image);
    
    // Accessors remain the same
    concord::Point at(uint x, uint y) const { return grid.at(x, y).first; }
    GridData data_at(uint x, uint y) const { return grid.at(x, y).second; }
    concord::Grid<GridData> &getGrid() { return grid; }
    const concord::Grid<GridData> &getGrid() const { return grid; }
};

} // namespace fs
```

#### 2. Parallelize Core Methods

**a) Parallel `add_noise()` Implementation**
```cpp
// layer.cpp - Kokkos implementation
void Layer::init_kokkos_views() {
    // Allocate Kokkos views
    kokkos_grid_data = GridDataView("grid_data", rows, cols);
    kokkos_colors = ColorView("colors", rows, cols, 4);  // RGBA
    kokkos_data = FloatView("noise_data", rows, cols);
    kokkos_points_x = FloatView("points_x", rows, cols);
    kokkos_points_y = FloatView("points_y", rows, cols);
    
    // Copy grid points to Kokkos views
    auto h_points_x = Kokkos::create_mirror_view(kokkos_points_x);
    auto h_points_y = Kokkos::create_mirror_view(kokkos_points_y);
    
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            auto& [pt, gd] = grid(r, c);
            h_points_x(r, c) = pt.x;
            h_points_y(r, c) = pt.y;
        }
    }
    
    Kokkos::deep_copy(kokkos_points_x, h_points_x);
    Kokkos::deep_copy(kokkos_points_y, h_points_y);
}

void Layer::add_noise_kokkos(bool in_polygon_only) {
    // Setup noise parameters
    noise.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
    auto sz = std::max(info.bound.size.x, info.bound.size.y);
    float frequency = sz / 300000.0f;
    int seed = int(rnd());
    
    // Get polygon bounds for quick rejection
    float min_x = enu_corners_[0][0];
    float max_x = enu_corners_[enu_corners_.size() - 1][0];
    float min_y = enu_corners_[0][1];
    float max_y = enu_corners_[enu_corners_.size() - 1][1];
    
    // Copy field polygon to device (simplified - using bounding box for now)
    Kokkos::parallel_for("Layer::add_noise", 
        Kokkos::MDRangePolicy<Kokkos::Rank<2>>({0, 0}, {rows, cols}),
        KOKKOS_LAMBDA(const size_t r, const size_t c) {
            // Each thread needs its own noise generator
            // Note: This assumes entropy::NoiseGen can be used in device code
            // In practice, you might need a simpler noise function
            float x = kokkos_points_x(r, c);
            float y = kokkos_points_y(r, c);
            
            if (in_polygon_only) {
                if (x > min_x && x < max_x && y > min_y && y < max_y) {
                    // Simple noise calculation - replace with actual noise function
                    float noise_val = sin(frequency * float(r) + seed) * 
                                     cos(frequency * float(c) + seed);
                    kokkos_data(r, c) = noise_val / 2.0f;
                }
            } else {
                // Simple noise calculation for all cells
                float noise_val = sin(frequency * float(r) + seed) * 
                                 cos(frequency * float(c) + seed);
                kokkos_data(r, c) = noise_val / 2.0f;
            }
        });
    
    Kokkos::fence();
    has_noise = true;
}
```

**b) Parallel `paint()` with Optimized Index Computation**
```cpp
void Layer::paint_kokkos(pigment::RGB color, concord::Polygon brush) {
    // Convert brush polygon to device-friendly format
    // For now, we'll use the existing indices_within method and parallelize the painting
    auto indices = grid.indices_within(brush);
    
    // Create device copy of indices
    Kokkos::View<size_t*> d_indices("indices", indices.size());
    auto h_indices = Kokkos::create_mirror_view(d_indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        h_indices(i) = indices[i];
    }
    Kokkos::deep_copy(d_indices, h_indices);
    
    // Parallel color assignment
    uint8_t r = static_cast<uint8_t>(color.r);
    uint8_t g = static_cast<uint8_t>(color.g);
    uint8_t b = static_cast<uint8_t>(color.b);
    uint8_t a = static_cast<uint8_t>(color.a);
    
    Kokkos::parallel_for("Layer::paint_color", indices.size(),
        KOKKOS_LAMBDA(const size_t i) {
            size_t idx = d_indices(i);
            size_t row = idx / cols;
            size_t col = idx % cols;
            
            kokkos_colors(row, col, 0) = r;
            kokkos_colors(row, col, 1) = g;
            kokkos_colors(row, col, 2) = b;
            kokkos_colors(row, col, 3) = a;
        });
    
    Kokkos::fence();
}

// Alternative: Fully parallel polygon check
void Layer::paint_kokkos_v2(pigment::RGB color, concord::Polygon brush) {
    // This version checks polygon containment in parallel
    uint8_t r = static_cast<uint8_t>(color.r);
    uint8_t g = static_cast<uint8_t>(color.g);
    uint8_t b = static_cast<uint8_t>(color.b);
    uint8_t a = static_cast<uint8_t>(color.a);
    
    // Get brush bounds for early rejection
    auto brush_bounds = brush.bounding_box();
    float brush_min_x = brush_bounds.min.x;
    float brush_max_x = brush_bounds.max.x;
    float brush_min_y = brush_bounds.min.y;
    float brush_max_y = brush_bounds.max.y;
    
    Kokkos::parallel_for("Layer::paint_with_check", 
        Kokkos::MDRangePolicy<Kokkos::Rank<2>>({0, 0}, {rows, cols}),
        KOKKOS_LAMBDA(const size_t row, const size_t col) {
            float x = kokkos_points_x(row, col);
            float y = kokkos_points_y(row, col);
            
            // Quick bounds check
            if (x >= brush_min_x && x <= brush_max_x &&
                y >= brush_min_y && y <= brush_max_y) {
                // TODO: Implement device-side point-in-polygon test
                // For now, just paint within bounds
                kokkos_colors(row, col, 0) = r;
                kokkos_colors(row, col, 1) = g;
                kokkos_colors(row, col, 2) = b;
                kokkos_colors(row, col, 3) = a;
            }
        });
    
    Kokkos::fence();
}
```

**c) Parallel `to_image()` with Direct Memory Write**
```cpp
void Layer::to_image_kokkos(std::vector<uint8_t>& image) {
    const size_t total_size = rows * cols * 4;
    image.resize(total_size);
    
    // Create unmanaged view of image data for direct write
    Kokkos::View<uint8_t*, Kokkos::MemoryTraits<Kokkos::Unmanaged>> 
        image_view(image.data(), total_size);
    
    // Sync colors from grid to kokkos if needed
    sync_to_kokkos();
    
    Kokkos::parallel_for("Layer::to_image",
        Kokkos::MDRangePolicy<Kokkos::Rank<2>>({0, 0}, {rows, cols}),
        KOKKOS_LAMBDA(const size_t r, const size_t c) {
            // Apply 180-degree rotation as in original
            size_t rotated_r = rows - 1 - r;
            size_t rotated_c = cols - 1 - c;
            size_t base_idx = (rotated_r * cols + rotated_c) * 4;
            
            // Direct copy from kokkos color view
            image_view[base_idx + 0] = kokkos_colors(r, c, 0);  // R
            image_view[base_idx + 1] = kokkos_colors(r, c, 1);  // G
            image_view[base_idx + 2] = kokkos_colors(r, c, 2);  // B
            image_view[base_idx + 3] = kokkos_colors(r, c, 3);  // A
        });
    
    Kokkos::fence();
}

// Sync methods for transitioning between serial and parallel code
void Layer::sync_to_kokkos() {
    auto h_colors = Kokkos::create_mirror_view(kokkos_colors);
    auto h_data = Kokkos::create_mirror_view(kokkos_data);
    
    // Copy from grid to host views
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            auto& [pt, gd] = grid(r, c);
            h_colors(r, c, 0) = static_cast<uint8_t>(gd.color.r);
            h_colors(r, c, 1) = static_cast<uint8_t>(gd.color.g);
            h_colors(r, c, 2) = static_cast<uint8_t>(gd.color.b);
            h_colors(r, c, 3) = static_cast<uint8_t>(gd.color.a);
            h_data(r, c) = gd.data;
        }
    }
    
    // Copy to device
    Kokkos::deep_copy(kokkos_colors, h_colors);
    Kokkos::deep_copy(kokkos_data, h_data);
}

void Layer::sync_from_kokkos() {
    auto h_colors = Kokkos::create_mirror_view(kokkos_colors);
    auto h_data = Kokkos::create_mirror_view(kokkos_data);
    
    // Copy from device
    Kokkos::deep_copy(h_colors, kokkos_colors);
    Kokkos::deep_copy(h_data, kokkos_data);
    
    // Copy to grid
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            auto& [pt, gd] = grid(r, c);
            gd.color.r = h_colors(r, c, 0);
            gd.color.g = h_colors(r, c, 1);
            gd.color.b = h_colors(r, c, 2);
            gd.color.a = h_colors(r, c, 3);
            gd.data = h_data(r, c);
        }
    }
}
```

**d) Parallel `color_field()` Implementation**
```cpp
void Layer::color_field_kokkos() {
    // Get indices within the field polygon
    auto indices = grid.indices_within(info.field);
    
    // Convert color values
    uint8_t base_r = static_cast<uint8_t>(info.color.r);
    uint8_t base_g = static_cast<uint8_t>(info.color.g);
    uint8_t base_b = static_cast<uint8_t>(info.color.b);
    uint8_t base_a = static_cast<uint8_t>(info.color.a);
    
    // Create device view of indices
    Kokkos::View<size_t*> d_indices("field_indices", indices.size());
    auto h_indices = Kokkos::create_mirror_view(d_indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        h_indices(i) = indices[i];
    }
    Kokkos::deep_copy(d_indices, h_indices);
    
    // Parallel color assignment with noise modulation
    Kokkos::parallel_for("Layer::color_field", indices.size(),
        KOKKOS_LAMBDA(const size_t i) {
            size_t idx = d_indices(i);
            size_t r = idx / cols;
            size_t c = idx % cols;
            
            kokkos_colors(r, c, 0) = base_r;
            kokkos_colors(r, c, 1) = base_g;
            kokkos_colors(r, c, 2) = base_b;
            kokkos_colors(r, c, 3) = base_a;
            
            if (has_noise) {
                // Apply noise-based color modulation
                float noise_val = kokkos_data(r, c);
                uint8_t val = static_cast<uint8_t>((noise_val + 0.5f) * 255.0f);
                // Map value from [0,255] to [100,155] as in original
                kokkos_colors(r, c, 0) = 100 + (val * 55) / 255;
            }
        });
    
    Kokkos::fence();
}
```

#### 3. Optimize Grid Operations

**Parallel `indices_within()` with Reduction**
```cpp
std::vector<size_t> Grid::indices_within_parallel(const concord::Polygon& polygon) {
    size_t total_cells = rows * cols;
    
    // First: count cells in polygon
    size_t count = 0;
    Kokkos::parallel_reduce("Grid::count_within",
        Kokkos::RangePolicy<>(0, total_cells),
        KOKKOS_LAMBDA(const size_t idx, size_t& local_count) {
            size_t r = idx / cols;
            size_t c = idx % cols;
            if (polygon.contains_point(points_view(r, c))) {
                local_count++;
            }
        }, count);
    
    // Second: collect indices
    Kokkos::View<size_t*> indices_view("indices", count);
    Kokkos::View<size_t> counter("counter");
    
    Kokkos::parallel_for("Grid::collect_indices",
        Kokkos::RangePolicy<>(0, total_cells),
        KOKKOS_LAMBDA(const size_t idx) {
            size_t r = idx / cols;
            size_t c = idx % cols;
            if (polygon.contains_point(points_view(r, c))) {
                auto pos = Kokkos::atomic_fetch_add(&counter(), 1);
                indices_view(pos) = idx;
            }
        });
    
    // Copy back to vector
    std::vector<size_t> result(count);
    Kokkos::deep_copy(Kokkos::View<size_t*, Kokkos::HostSpace, 
                                   Kokkos::MemoryTraits<Kokkos::Unmanaged>>(
                                   result.data(), count), indices_view);
    return result;
}
```

## Priority 2: Multi-Robot Update System

### Parallel Robot Updates
```cpp
void Simulator::tick_parallel(float dt) {
    size_t num_robots = robots.size();
    
    // Create views for robot data
    Kokkos::View<Robot**> robot_view("robots", num_robots);
    
    // Parallel robot physics update
    Kokkos::parallel_for("Simulator::update_robots",
        Kokkos::TeamPolicy<>(num_robots, Kokkos::AUTO),
        KOKKOS_LAMBDA(const Kokkos::TeamPolicy<>::member_type& team) {
            auto robot_idx = team.league_rank();
            auto& robot = robot_view(robot_idx);
            
            // Update sensors in parallel within team
            team.team_barrier();
            Kokkos::parallel_for(Kokkos::TeamThreadRange(team, robot->sensors.size()),
                [&](const size_t sensor_idx) {
                    robot->sensors[sensor_idx]->update(dt);
                });
            
            // Single thread updates chassis
            Kokkos::single(Kokkos::PerTeam(team), [&]() {
                robot->chassis->update(dt);
            });
        });
}
```

## Priority 3: LIDAR Sensor Parallelization

### Parallel Raycast Processing
```cpp
void LidarSensor::update_3d_parallel(float dt) {
    size_t total_rays = num_horizontal_beams * num_vertical_layers;
    
    Kokkos::View<RayResult*> ray_results("ray_results", total_rays);
    
    Kokkos::parallel_for("Lidar::cast_rays",
        Kokkos::RangePolicy<>(0, total_rays),
        KOKKOS_LAMBDA(const size_t idx) {
            size_t h_idx = idx % num_horizontal_beams;
            size_t v_idx = idx / num_horizontal_beams;
            
            float h_angle = h_idx * h_angle_step;
            float v_angle = v_idx * v_angle_step + vertical_angle_start;
            
            // Perform raycast
            Vec2 direction = rotate_2d(base_direction, h_angle);
            auto result = cast_ray_3d(origin, direction, v_angle, max_range);
            ray_results(idx) = result;
        });
    
    // Process results
    process_ray_results(ray_results);
}
```

## Priority 4: Spatial Query Optimization

### Parallel Spatial Indexing
```cpp
class KokkosQuadTree {
    struct Node {
        AABB bounds;
        Kokkos::View<size_t*> object_indices;
        Kokkos::View<Node*> children;
    };
    
    void query_parallel(const AABB& query_bounds, 
                       Kokkos::View<size_t*>& results) {
        Kokkos::parallel_for("QuadTree::query",
            Kokkos::TeamPolicy<>(num_nodes, Kokkos::AUTO),
            KOKKOS_LAMBDA(const team_member& team) {
                // Parallel tree traversal with team-based work sharing
            });
    }
};
```

## Priority 5: Physics Engine Integration

### Parallel Collision Detection
```cpp
void World::detect_collisions_parallel() {
    size_t num_bodies = rigid_bodies.size();
    
    // Broad phase: parallel AABB overlap detection
    Kokkos::View<CollisionPair*> potential_pairs;
    
    Kokkos::parallel_for("World::broad_phase",
        Kokkos::TeamPolicy<>(num_bodies, Kokkos::AUTO),
        KOKKOS_LAMBDA(const team_member& team) {
            auto i = team.league_rank();
            Kokkos::parallel_for(Kokkos::TeamThreadRange(team, i+1, num_bodies),
                [&](const size_t j) {
                    if (aabb_overlap(bodies[i], bodies[j])) {
                        // Add to potential pairs with atomic
                    }
                });
        });
    
    // Narrow phase: parallel contact generation
    Kokkos::parallel_for("World::narrow_phase",
        potential_pairs.size(),
        KOKKOS_LAMBDA(const size_t idx) {
            generate_contacts(potential_pairs[idx]);
        });
}
```

## Implementation Timeline

### Phase 1: Layer System (Week 1)
- [ ] Add Kokkos views to Layer class
- [ ] Implement parallel `add_noise()`
- [ ] Implement parallel `paint()` and `color_field()`
- [ ] Implement parallel `to_image()`
- [ ] Benchmark and optimize

### Phase 2: Robot Updates (Week 2)
- [ ] Parallelize robot tick updates
- [ ] Parallelize sensor updates within robots
- [ ] Implement parallel chassis updates
- [ ] Add synchronization for chain management

### Phase 3: Sensors (Week 3)
- [ ] Parallelize LIDAR raycast operations
- [ ] Optimize GPS sensor batch updates
- [ ] Parallelize IMU sensor calculations
- [ ] Implement sensor data aggregation

### Phase 4: Spatial Queries (Week 4)
- [ ] Implement Kokkos-based spatial index
- [ ] Parallelize proximity queries
- [ ] Optimize layer-robot intersection tests
- [ ] Add parallel collision culling

### Phase 5: Physics Integration (Week 5-6)
- [ ] Work with Muli physics to add Kokkos support
- [ ] Parallelize collision detection
- [ ] Optimize constraint solver
- [ ] Implement parallel island building

## Build Configuration

```cmake
# Add to CMakeLists.txt
set(Kokkos_ENABLE_OPENMP ON)
set(Kokkos_ENABLE_CUDA OFF)  # Enable for GPU support
set(Kokkos_ENABLE_SERIAL ON)
set(Kokkos_ARCH_NATIVE ON)   # Optimize for build architecture
```

## Performance Targets

### Layer Drawing
- **Current**: O(rows × cols) serial
- **Target**: O(rows × cols / P) where P = number of threads
- **Expected speedup**: 8-16x on modern CPUs, 100x+ on GPUs

### Robot Updates
- **Current**: O(robots × sensors) serial
- **Target**: O(robots × sensors / P)
- **Expected speedup**: Linear with robot count

### LIDAR Processing
- **Current**: O(beams × layers) serial raycasts
- **Target**: Parallel raycasts with spatial acceleration
- **Expected speedup**: 10-20x for typical configurations

## Memory Management

### Kokkos Memory Spaces
```cpp
// CPU-only configuration
using MemSpace = Kokkos::HostSpace;
using ExecSpace = Kokkos::OpenMP;

// GPU configuration
using MemSpace = Kokkos::CudaSpace;
using ExecSpace = Kokkos::Cuda;

// Unified configuration
using MemSpace = Kokkos::SharedSpace;
using ExecSpace = Kokkos::DefaultExecutionSpace;
```

### View Management Best Practices
1. Minimize deep copies between host and device
2. Use unmanaged views for existing data
3. Prefer mirror views for CPU/GPU portability
4. Use atomic operations sparingly

## Testing Strategy

### Unit Tests
```cpp
TEST_CASE("Kokkos Layer paint performance") {
    Kokkos::initialize();
    
    Layer layer(1000, 1000); // 1M cells
    
    BENCHMARK("Serial paint") {
        layer.paint_serial(color, polygon);
    };
    
    BENCHMARK("Kokkos paint") {
        layer.paint(color, polygon);
    };
    
    Kokkos::finalize();
}
```

### Integration Tests
- Verify identical results between serial and parallel
- Test with various grid sizes and polygon complexities
- Validate thread safety with concurrent operations
- Benchmark on different hardware configurations

## Debugging Tools

```bash
# Enable Kokkos profiling
export KOKKOS_PROFILE_LIBRARY=libkokkos-tools-simple-kernel-timer.so

# Enable debug mode
cmake -DKokkos_ENABLE_DEBUG=ON ..

# Use Kokkos tools for performance analysis
nsys profile ./mvs  # NVIDIA systems
vtune ./mvs         # Intel systems
```

## Integration Example

### Main Application Setup
```cpp
// mvs.cpp modifications
#include <Kokkos_Core.hpp>

int main(int argc, char* argv[]) {
    // Initialize Kokkos
    Kokkos::initialize(argc, argv);
    
    {  // Kokkos scope
        // Parse arguments and setup...
        
        // Initialize Kokkos execution space info
        std::cout << "Kokkos execution space: " 
                  << Kokkos::DefaultExecutionSpace::name() << std::endl;
        std::cout << "Kokkos host execution space: " 
                  << Kokkos::DefaultHostExecutionSpace::name() << std::endl;
        std::cout << "Kokkos concurrency: " 
                  << Kokkos::DefaultExecutionSpace::concurrency() << std::endl;
        
        // Create simulator with Kokkos support
        auto sim = std::make_unique<fs::Simulator>(rec);
        sim->enable_kokkos(true);  // Enable Kokkos optimizations
        
        // Main loop remains the same
        while (running) {
            // ... input handling ...
            sim->tick(dt.count());  // Will use Kokkos internally
        }
    }
    
    // Finalize Kokkos
    Kokkos::finalize();
    return 0;
}
```

### Modified Layer Constructor
```cpp
// layer.cpp
Layer::Layer(std::shared_ptr<rerun::RecordingStream> rec, concord::Datum datum)
    : rec(rec), datum(datum), rnd(std::random_device()()) {
    // Check if Kokkos is initialized
    use_kokkos = Kokkos::is_initialized();
}

void Layer::init(LayerInfo info) {
    // ... existing initialization ...
    
    if (use_kokkos) {
        init_kokkos_views();
        
        // Initialize colors in Kokkos views
        Kokkos::parallel_for("Layer::init_colors",
            Kokkos::MDRangePolicy<Kokkos::Rank<2>>({0, 0}, {rows, cols}),
            KOKKOS_LAMBDA(const size_t r, const size_t c) {
                kokkos_colors(r, c, 0) = 50;  // R
                kokkos_colors(r, c, 1) = 50;  // G
                kokkos_colors(r, c, 2) = 50;  // B
                kokkos_colors(r, c, 3) = 255; // A
                kokkos_data(r, c) = 0.0f;
            });
        Kokkos::fence();
    }
}

// Wrapper methods that choose implementation
void Layer::paint(pigment::RGB color, concord::Polygon brush) {
    if (use_kokkos) {
        paint_kokkos(color, brush);
    } else {
        // Original implementation
        auto indices = grid.indices_within(brush);
        for (auto idx : indices) {
            std::size_t r = idx / grid.cols();
            std::size_t c = idx % grid.cols();
            grid(r, c).second.color = color;
        }
    }
}
```

## Performance Benchmarking

### Simple Benchmark Code
```cpp
// benchmark_layer.cpp
#include <chrono>
#include <iostream>
#include "flatsim/world/layer.hpp"

void benchmark_layer_operations() {
    Kokkos::initialize();
    
    // Create test layers of different sizes
    std::vector<size_t> grid_sizes = {100, 500, 1000, 2000};
    
    for (auto size : grid_sizes) {
        std::cout << "\nBenchmarking " << size << "x" << size << " grid:\n";
        
        // Setup
        auto rec = std::make_shared<rerun::RecordingStream>("benchmark");
        Layer layer_serial(rec, datum);
        Layer layer_kokkos(rec, datum);
        
        LayerInfo info;
        info.resolution = 1.0f;
        info.bound.size = {float(size), float(size), 0};
        
        layer_serial.use_kokkos = false;
        layer_kokkos.use_kokkos = true;
        
        layer_serial.init(info);
        layer_kokkos.init(info);
        
        // Create test polygon (covers 25% of area)
        concord::Polygon brush;
        brush.add_point({size * 0.25f, size * 0.25f});
        brush.add_point({size * 0.75f, size * 0.25f});
        brush.add_point({size * 0.75f, size * 0.75f});
        brush.add_point({size * 0.25f, size * 0.75f});
        
        // Benchmark serial
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 100; ++i) {
            layer_serial.paint({255, 0, 0}, brush);
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto serial_time = std::chrono::duration<double, std::milli>(end - start).count();
        
        // Benchmark Kokkos
        start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 100; ++i) {
            layer_kokkos.paint({255, 0, 0}, brush);
        }
        end = std::chrono::high_resolution_clock::now();
        auto kokkos_time = std::chrono::duration<double, std::milli>(end - start).count();
        
        std::cout << "Serial: " << serial_time << " ms\n";
        std::cout << "Kokkos: " << kokkos_time << " ms\n";
        std::cout << "Speedup: " << serial_time / kokkos_time << "x\n";
    }
    
    Kokkos::finalize();
}
```

## Troubleshooting

### Common Issues

1. **Kokkos not found during compilation**
   ```bash
   # Ensure Kokkos is properly fetched
   cd build
   cmake ..
   make clean
   make build
   ```

2. **Runtime errors with uninitialized Kokkos**
   - Always check `Kokkos::is_initialized()` before using Kokkos features
   - Ensure `Kokkos::initialize()` is called before any Kokkos operations

3. **Memory access violations**
   - Verify view dimensions match grid dimensions
   - Use `Kokkos::fence()` after parallel operations
   - Check array bounds in lambda functions

4. **Poor performance**
   - Enable OpenMP: `export OMP_NUM_THREADS=8`
   - Check execution space: might be running serial
   - Profile with Kokkos tools to identify bottlenecks

### Debug Build
```cmake
# Add to CMakeLists.txt for debug
set(Kokkos_ENABLE_DEBUG ON)
set(Kokkos_ENABLE_DEBUG_BOUNDS_CHECK ON)
set(Kokkos_ENABLE_PROFILING ON)
```

## Next Steps

1. **Immediate Actions**:
   - Implement Layer Kokkos integration
   - Create unit tests for parallel correctness
   - Benchmark on target hardware

2. **Short Term** (1-2 weeks):
   - Extend to robot sensor updates
   - Parallelize LIDAR processing
   - Add GPU support configuration

3. **Long Term** (1 month):
   - Full physics engine integration
   - Distributed simulation support
   - Custom kernels for specific operations

## Conclusion

Kokkos integration will transform Multiverse into a high-performance parallel simulation framework. Starting with Layer drawing provides immediate benefits with minimal risk, establishing patterns for broader parallelization throughout the codebase. The modular approach allows gradual adoption while maintaining backward compatibility.