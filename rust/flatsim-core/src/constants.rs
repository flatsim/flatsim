//! Physical and simulation constants
//!
//! This module contains various constants used throughout the simulator,
//! including physical constants, default values, and configuration parameters.

/// Gravitational acceleration (m/s²)
pub const GRAVITY: f64 = 9.81;

/// Default simulation timestep (seconds)
pub const DEFAULT_DT: f64 = 0.01;

/// Pi constant
pub const PI: f64 = std::f64::consts::PI;

/// Epsilon for floating point comparisons
pub const EPSILON: f64 = 1e-10;

/// Conversion factor: degrees to radians
pub const DEG_TO_RAD: f64 = PI / 180.0;

/// Conversion factor: radians to degrees
pub const RAD_TO_DEG: f64 = 180.0 / PI;
