//! Flatsim FFI Layer
//!
//! This crate provides C++ interoperability for the Flatsim core library using CXX.

use flatsim_core::Vec2;

/// Simple hello world function to test FFI connection
pub fn rust_hello() -> String {
    "Hello from Rust! Flatsim FFI is working.".to_string()
}

/// Creates a new Vec2 and returns its length
pub fn vec2_example(x: f64, y: f64) -> f64 {
    let v = Vec2::new(x, y);
    v.length()
}

#[cxx::bridge]
mod ffi {
    extern "Rust" {
        fn rust_hello() -> String;
        fn vec2_example(x: f64, y: f64) -> f64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rust_hello() {
        let msg = rust_hello();
        assert!(msg.contains("Rust"));
    }

    #[test]
    fn test_vec2_example() {
        assert_eq!(vec2_example(3.0, 4.0), 5.0);
    }
}
