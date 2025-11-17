//! Flatsim Core Library
//!
//! This is the core Rust library for the Flatsim agricultural robotics simulator.
//! It contains the main simulation logic, data structures, and algorithms.

pub mod types;
pub mod constants;
pub mod utils;

// Re-export commonly used types
pub use types::{Vec2, Vec3};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanity_check() {
        let v = Vec2::new(1.0, 2.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
    }
}
