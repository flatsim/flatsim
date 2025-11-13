# Rust Migration Plan for Flatsim

## Overview

This document outlines the strategy for gradually migrating the Flatsim C++ codebase to Rust while maintaining compatibility with existing C++ dependencies and ensuring a smooth transition.

## Why Rust?

- **Memory Safety**: Eliminate segfaults, use-after-free, and data races at compile time
- **Fearless Concurrency**: Safe parallel programming without data races
- **Modern Tooling**: Superior package manager (Cargo), formatter (rustfmt), linter (Clippy)
- **Performance**: Matches or exceeds C++ performance with zero-cost abstractions
- **Better Error Handling**: Result<T, E> and Option<T> vs exceptions and null pointers
- **Maintainability**: Stronger type system catches bugs at compile time

## Interoperability Approaches

### 1. CXX (Recommended for Bidirectional Interop)

**Use for**: Gradual component replacement with type-safe bidirectional communication

**Pros**:
- Type-safe FFI boundary
- Zero or near-zero overhead
- Excellent error messages
- Prevents common FFI mistakes

**Example**:
```rust
#[cxx::bridge]
mod ffi {
    unsafe extern "C++" {
        include!("flatsim/robot.hpp");
        type Robot;
        fn update(self: &Robot, dt: f64);
    }
    
    extern "Rust" {
        type SimulatorState;
        fn step_simulation(state: &mut SimulatorState);
    }
}
```

**Resources**: https://cxx.rs

### 2. Bindgen (C++ to Rust Bindings)

**Use for**: Automatically generating Rust bindings to existing C++ libraries

**Pros**:
- Automatic binding generation from headers
- Handles complex C++ constructs
- Well-maintained by Rust project

**Example**:
```rust
// build.rs
bindgen::Builder::default()
    .header("include/flatsim/types.hpp")
    .generate()
    .expect("Unable to generate bindings");
```

**Resources**: https://rust-lang.github.io/rust-bindgen/

### 3. cbindgen (Rust to C++ Bindings)

**Use for**: Exposing new Rust modules to existing C++ code

**Pros**:
- Generates C/C++ headers from Rust code
- Good for maintaining C++ API surface
- Integrates well with CMake

**Resources**: https://github.com/eqrion/cbindgen

## Migration Strategy

### Project Structure

```
flatsim/
├── Cargo.toml              # Rust workspace root
├── CMakeLists.txt          # Existing CMake (modified)
├── src/                    # C++ source (gradually deprecated)
├── include/                # C++ headers (gradually deprecated)
├── rust/
│   ├── flatsim-core/       # Core Rust library
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── types.rs
│   │       ├── robot.rs
│   │       └── ...
│   ├── flatsim-ffi/        # FFI layer for C++ interop
│   │   ├── Cargo.toml
│   │   └── src/
│   │       └── lib.rs
│   └── flatsim-bindings/   # Bindings to C++ dependencies
│       ├── Cargo.toml
│       ├── build.rs
│       └── src/
│           └── lib.rs
├── examples/               # Examples (mixed C++/Rust)
└── tests/                  # Tests (mixed C++/Rust)
```

### Phase 1: Setup and Infrastructure (Weeks 1-2)

**Goals**:
- Set up Rust workspace
- Integrate Rust build with CMake
- Create basic FFI infrastructure
- Set up CI/CD for Rust

**Tasks**:
1. Create `Cargo.toml` workspace file
2. Install Corrosion or cargo-cmake for CMake integration
3. Create initial `flatsim-core` and `flatsim-ffi` crates
4. Add Rust toolchain to CI/CD pipeline
5. Document build process

**CMake Integration Example**:
```cmake
include(FetchContent)
FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.4
)
FetchContent_MakeAvailable(Corrosion)

corrosion_import_crate(MANIFEST_PATH rust/Cargo.toml)
target_link_libraries(flatsim_internal PUBLIC flatsim_rust)
```

**Deliverables**:
- [ ] Rust workspace builds successfully
- [ ] CMake can link Rust libraries
- [ ] "Hello World" FFI call works (C++ → Rust → C++)
- [ ] Documentation updated

### Phase 2: Core Types Migration (Weeks 3-6)

**Goals**:
- Migrate basic data structures
- Establish patterns for FFI
- Create comprehensive tests

**Components to Migrate**:
1. `include/flatsim/types.hpp`
   - Vec2, Vec3, Quaternion
   - Basic geometric types
   - Color, RGB types
2. `include/flatsim/constants.hpp`
   - Physical constants
   - Configuration constants
3. `include/flatsim/exceptions.hpp`
   - Error types → Result<T, E>

**Example Migration**:

**Before (C++)**:
```cpp
// include/flatsim/types.hpp
struct Vec2 {
    double x, y;
    Vec2 operator+(const Vec2& other) const;
    double length() const;
};
```

**After (Rust)**:
```rust
// rust/flatsim-core/src/types.rs
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

impl std::ops::Add for Vec2 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Vec2 { x: self.x + other.x, y: self.y + other.y }
    }
}

// FFI bridge
#[cxx::bridge]
mod ffi {
    extern "Rust" {
        type Vec2;
        fn vec2_new(x: f64, y: f64) -> Vec2;
        fn vec2_length(v: &Vec2) -> f64;
    }
}
```

**Testing Strategy**:
- Property-based tests with proptest
- Comparison tests against C++ implementation
- Benchmark both implementations

**Deliverables**:
- [ ] Core types in Rust with full test coverage
- [ ] FFI layer working with existing C++ code
- [ ] Performance benchmarks showing no regression
- [ ] Documentation and migration guide

### Phase 3: Utilities and Pure Functions (Weeks 7-10)

**Goals**:
- Migrate stateless utility functions
- Build confidence in Rust implementations
- Establish performance baselines

**Components to Migrate**:
1. `include/flatsim/utils.hpp`
   - Math utilities
   - Geometry calculations
   - Coordinate transformations
2. Pure algorithms without external dependencies

**Strategy**:
- Keep C++ API unchanged
- Replace implementation with Rust
- Extensive testing and benchmarking

**Example**:
```rust
// rust/flatsim-core/src/utils.rs
pub fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

// Expose to C++ via CXX
#[cxx::bridge]
mod ffi {
    extern "Rust" {
        fn clamp(value: f64, min: f64, max: f64) -> f64;
        fn lerp(a: f64, b: f64, t: f64) -> f64;
    }
}
```

**Deliverables**:
- [ ] All utility functions migrated
- [ ] Unit tests with 100% coverage
- [ ] Performance benchmarks
- [ ] Documentation

### Phase 4: Message and Network Layer (Weeks 11-16)

**Goals**:
- Migrate message types
- Replace network interfaces
- Improve serialization/deserialization

**Components to Migrate**:
1. `include/flatsim/robot/systems/messages.hpp`
2. `include/flatsim/network/interface.hpp`
3. Network interface implementations

**Rust Advantages**:
- Use `serde` for serialization (better than Boost.JSON)
- Use `tokio-zmq` for ZeroMQ (async support)
- Strong typing for message protocols

**Example**:
```rust
// rust/flatsim-core/src/messages.rs
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Message {
    Position { x: f64, y: f64, timestamp: f64 },
    Velocity { vx: f64, vy: f64 },
    Command { action: String, params: Vec<f64> },
}

// rust/flatsim-core/src/network.rs
use tokio::net::TcpStream;

pub struct NetworkInterface {
    stream: TcpStream,
}

impl NetworkInterface {
    pub async fn send_message(&mut self, msg: &Message) -> Result<(), Error> {
        let bytes = serde_json::to_vec(msg)?;
        // Send over network
        Ok(())
    }
}
```

**Deliverables**:
- [ ] Message types in Rust with serde
- [ ] Network layer working with existing C++ clients
- [ ] Async runtime integrated (optional)
- [ ] Integration tests

### Phase 5: Sensor and Robot Systems (Weeks 17-24)

**Goals**:
- Migrate sensor implementations
- Port robot subsystems
- Maintain physics engine integration

**Components to Migrate**:
1. `include/flatsim/robot/sensor.hpp`
2. `include/flatsim/robot/sensors/` (GPS, IMU, LiDAR)
3. `include/flatsim/robot/systems/control.hpp`
4. `include/flatsim/robot/systems/chain.hpp`

**Strategy**:
- Keep Muli physics engine in C++ (use via FFI)
- Migrate sensor logic to Rust
- Use trait objects for extensibility

**Example**:
```rust
// rust/flatsim-core/src/sensor.rs
pub trait Sensor: Send + Sync {
    fn update(&mut self, dt: f64);
    fn get_data(&self) -> SensorData;
}

#[derive(Debug, Clone)]
pub struct GpsSensor {
    position: Vec2,
    noise: f64,
}

impl Sensor for GpsSensor {
    fn update(&mut self, dt: f64) {
        // Update sensor state
    }
    
    fn get_data(&self) -> SensorData {
        SensorData::Gps {
            lat: self.position.x,
            lon: self.position.y,
        }
    }
}
```

**Physics Integration**:
```rust
// rust/flatsim-bindings/src/muli.rs
// Use bindgen to create bindings to Muli

#[cxx::bridge]
mod muli_ffi {
    unsafe extern "C++" {
        include!("muli/muli.h");
        type World;
        type Body;
        
        fn create_world() -> UniquePtr<World>;
        fn step(self: Pin<&mut World>, dt: f32);
    }
}
```

**Deliverables**:
- [ ] Sensor trait and implementations
- [ ] Robot systems in Rust
- [ ] Physics engine FFI bindings
- [ ] Integration tests with C++ physics

### Phase 6: World and Simulation Core (Weeks 25-32)

**Goals**:
- Migrate simulation loop
- Port world management
- Integrate all components

**Components to Migrate**:
1. `include/flatsim/world.hpp`
2. `include/flatsim/simulator.hpp`
3. `include/flatsim/dispatcher.hpp`
4. Obstacle management

**Major Refactoring Opportunities**:
- Use Rust's ownership system for resource management
- Replace manual threading with Rayon for parallelism
- Use async/await for I/O operations

**Example**:
```rust
// rust/flatsim-core/src/simulator.rs
use rayon::prelude::*;

pub struct Simulator {
    world: World,
    robots: Vec<Box<dyn Robot>>,
    time: f64,
}

impl Simulator {
    pub fn step(&mut self, dt: f64) {
        // Update physics (C++ Muli)
        self.world.step(dt);
        
        // Update robots in parallel (Rust Rayon)
        self.robots.par_iter_mut()
            .for_each(|robot| robot.update(dt));
        
        self.time += dt;
    }
}
```

**Deliverables**:
- [ ] Core simulator in Rust
- [ ] All components integrated
- [ ] Performance meets or exceeds C++ version
- [ ] Full system tests

### Phase 7: Chassis and Machine Models (Weeks 33-40)

**Goals**:
- Migrate vehicle dynamics
- Port machine-specific logic
- Maintain JSON configuration compatibility

**Components to Migrate**:
1. `include/flatsim/robot/chassis/` (wheel, chassis, hitch, etc.)
2. `include/flatsim/machines.hpp`
3. Machine configuration loading

**Configuration in Rust**:
```rust
// rust/flatsim-core/src/machines.rs
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct MachineConfig {
    pub name: String,
    pub chassis: ChassisConfig,
    pub sensors: Vec<SensorConfig>,
    pub mass: f64,
}

impl MachineConfig {
    pub fn from_file(path: &str) -> Result<Self, Error> {
        let json = std::fs::read_to_string(path)?;
        let config: MachineConfig = serde_json::from_str(&json)?;
        Ok(config)
    }
}
```

**Deliverables**:
- [ ] Chassis models in Rust
- [ ] JSON configuration parser
- [ ] Example machines working
- [ ] Performance validation

### Phase 8: Finalization and C++ Deprecation (Weeks 41-48)

**Goals**:
- Remove deprecated C++ code
- Optimize pure Rust codebase
- Comprehensive documentation

**Tasks**:
1. Remove unused C++ files
2. Optimize Rust code (profiling, benchmarking)
3. Update all documentation
4. Create migration guide for users
5. Publish Rust crates (optional)

**Final Optimizations**:
- Profile with `cargo flamegraph`
- Use `#[inline]` judiciously
- Consider unsafe optimizations where proven safe
- Benchmark against original C++ version

**Deliverables**:
- [ ] Pure Rust codebase (except external C++ deps)
- [ ] Comprehensive documentation
- [ ] Migration guide published
- [ ] Performance report
- [ ] Celebration! 🎉

## Dependency Migration

### Current C++ Dependencies → Rust Alternatives

| C++ Library | Purpose | Rust Alternative | Strategy |
|-------------|---------|------------------|----------|
| Muli | Physics | Keep C++ via FFI | Use bindgen/cxx |
| Rerun | Visualization | **Native Rust SDK!** | Direct migration |
| TBB | Parallelism | Rayon, tokio | Replace |
| ZeroMQ | Messaging | tokio-zmq, zeromq crate | Replace |
| spdlog | Logging | tracing, log, env_logger | Replace |
| fmt | Formatting | Built-in Rust formatting | No migration needed |
| Boost.JSON | JSON | serde, serde_json | Replace |
| CLI11 | CLI parsing | clap, structopt | Replace |
| concord | Coordinates | Port or FFI | Evaluate |
| pigment | (Unknown) | Evaluate | Evaluate |
| entropy | (Unknown) | Evaluate | Evaluate |
| zoneout | Zone management | Port or FFI | Evaluate |
| farmtrax | (Unknown) | Evaluate | Evaluate |
| navcon | Navigation | Port or FFI | Evaluate |

### Recommended Rust Crates

**Core**:
- `serde` / `serde_json` - Serialization
- `tokio` - Async runtime
- `rayon` - Data parallelism
- `crossbeam` - Concurrency primitives

**Numerics**:
- `nalgebra` - Linear algebra (Vec2, Vec3, matrices)
- `ndarray` - N-dimensional arrays
- `rand` - Random number generation

**Networking**:
- `tokio` - Async I/O
- `zeromq` - ZeroMQ bindings
- `tokio-zmq` - Async ZeroMQ

**Logging & Diagnostics**:
- `tracing` - Structured logging
- `tracing-subscriber` - Log output
- `log` - Simple logging facade

**CLI & Config**:
- `clap` - CLI argument parsing
- `config` - Configuration management
- `toml` / `ron` - Config file formats

**Testing**:
- `proptest` - Property-based testing
- `criterion` - Benchmarking
- `insta` - Snapshot testing

## Build System Integration

### CMake + Cargo Integration

Use Corrosion for seamless integration:

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project(flatsim VERSION 0.2.0 LANGUAGES CXX)

# Fetch Corrosion
include(FetchContent)
FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.4
)
FetchContent_MakeAvailable(Corrosion)

# Import Rust crates
corrosion_import_crate(
    MANIFEST_PATH rust/Cargo.toml
    CRATES flatsim-core flatsim-ffi
)

# Link Rust libraries
target_link_libraries(flatsim_internal PUBLIC flatsim_ffi)
```

### Cargo Workspace

```toml
# Cargo.toml (workspace root)
[workspace]
members = [
    "rust/flatsim-core",
    "rust/flatsim-ffi",
    "rust/flatsim-bindings",
]

[workspace.package]
version = "0.2.0"
edition = "2021"
rust-version = "1.70"

[workspace.dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
tokio = { version = "1.0", features = ["full"] }
rayon = "1.7"
tracing = "0.1"
nalgebra = "0.32"
cxx = "1.0"
```

## Testing Strategy

### Unit Tests
- Test each migrated component against C++ version
- Property-based testing with `proptest`
- Benchmark performance

### Integration Tests
- FFI boundary tests
- End-to-end simulation tests
- Cross-language tests

### Performance Testing
- Benchmark key functions
- Profile with `cargo flamegraph`
- Compare against C++ baseline
- Track memory usage

### Continuous Integration
```yaml
# .github/workflows/rust.yml
name: Rust CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions-rs/toolchain@v1
        with:
          toolchain: stable
      - run: cargo test --all
      - run: cargo clippy -- -D warnings
      - run: cargo fmt -- --check
```

## Risk Mitigation

### Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| FFI overhead | Performance | Benchmark early, optimize hot paths |
| Learning curve | Timeline | Training, pair programming, code reviews |
| C++ interop bugs | Quality | Extensive testing, gradual rollout |
| Build complexity | Developer experience | Good documentation, automated scripts |
| Dependency conflicts | Build failures | Careful version management |

### Project Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Timeline overrun | Delayed features | Phased approach, can pause at any phase |
| Team resistance | Adoption | Training, show benefits, optional initially |
| Compatibility breaks | User impact | Maintain C++ API surface during transition |
| Performance regression | User experience | Continuous benchmarking, rollback plan |

## Success Criteria

### Technical Metrics
- [ ] All existing tests pass in Rust version
- [ ] Performance within 5% of C++ version (or better)
- [ ] Memory usage equivalent or lower
- [ ] No increase in bug reports
- [ ] Build times acceptable (<10min for full build)

### Quality Metrics
- [ ] Test coverage >80%
- [ ] Zero unsafe code (except FFI boundaries)
- [ ] No Clippy warnings on default lints
- [ ] Documentation for all public APIs
- [ ] Benchmarks for critical paths

### Team Metrics
- [ ] All team members comfortable with Rust basics
- [ ] Contribution rate maintained or improved
- [ ] Code review turnaround time maintained
- [ ] Developer satisfaction improved

## Resources and Learning

### Official Documentation
- [The Rust Book](https://doc.rust-lang.org/book/)
- [Rust by Example](https://doc.rust-lang.org/rust-by-example/)
- [The Rustonomicon](https://doc.rust-lang.org/nomicon/) - Advanced topics
- [Rust FFI Omnibus](http://jakegoulding.com/rust-ffi-omnibus/) - FFI patterns

### Interop Resources
- [CXX Documentation](https://cxx.rs)
- [rust-bindgen User Guide](https://rust-lang.github.io/rust-bindgen/)
- [cbindgen User Guide](https://github.com/eqrion/cbindgen/blob/master/docs.md)

### Community
- [Rust Users Forum](https://users.rust-lang.org/)
- [r/rust subreddit](https://www.reddit.com/r/rust/)
- [Rust Discord](https://discord.gg/rust-lang)

### Books
- "Programming Rust" by Blandy & Orendorff
- "Rust for Rustaceans" by Jon Gjengset
- "Zero To Production In Rust" by Luca Palmieri

## Timeline Summary

| Phase | Duration | Cumulative | Key Deliverable |
|-------|----------|------------|-----------------|
| 1. Setup | 2 weeks | 2 weeks | Rust builds with CMake |
| 2. Types | 4 weeks | 6 weeks | Core types in Rust |
| 3. Utils | 4 weeks | 10 weeks | Pure functions migrated |
| 4. Network | 6 weeks | 16 weeks | Network layer in Rust |
| 5. Sensors | 8 weeks | 24 weeks | Robot systems working |
| 6. Simulator | 8 weeks | 32 weeks | Core simulation in Rust |
| 7. Chassis | 8 weeks | 40 weeks | Full machine models |
| 8. Finalization | 8 weeks | 48 weeks | Pure Rust codebase |

**Total Estimated Timeline**: 10-12 months for complete migration

**Note**: This is a phased approach - you can stop at any phase and maintain a hybrid C++/Rust codebase indefinitely if needed.

## Next Steps

1. **Review this plan** with the team
2. **Prototype Phase 1** (2 weeks)
   - Set up Rust workspace
   - Create "Hello World" FFI example
   - Document build process
3. **Evaluate prototype** and adjust plan
4. **Get team buy-in** for Phase 2
5. **Begin gradual migration**

## Questions to Consider

Before starting:
- [ ] Do we have Rust expertise on the team?
- [ ] What's our target completion date?
- [ ] Can we afford a mixed codebase during transition?
- [ ] Which components are most painful in C++ (prioritize these)?
- [ ] What are our performance requirements?
- [ ] Do we need to maintain C++ API for external users?

---

**Document Version**: 1.0  
**Last Updated**: 2025-11-13  
**Author**: Migration Planning Team  
**Status**: Draft - Ready for Review
