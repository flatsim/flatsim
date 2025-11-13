//! Utility functions
//!
//! This module contains various utility functions for mathematics,
//! geometry, and other common operations.

/// Clamps a value between a minimum and maximum
#[inline]
pub fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

/// Linear interpolation between two values
#[inline]
pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Normalizes an angle to the range [-PI, PI]
#[inline]
pub fn normalize_angle(angle: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let mut normalized = angle % (2.0 * pi);
    if normalized > pi {
        normalized -= 2.0 * pi;
    } else if normalized < -pi {
        normalized += 2.0 * pi;
    }
    normalized
}

/// Computes the shortest angular difference between two angles
#[inline]
pub fn angle_diff(from: f64, to: f64) -> f64 {
    normalize_angle(to - from)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clamp() {
        assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clamp(-5.0, 0.0, 10.0), 0.0);
        assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
    }

    #[test]
    fn test_lerp() {
        assert_eq!(lerp(0.0, 10.0, 0.0), 0.0);
        assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
        assert_eq!(lerp(0.0, 10.0, 1.0), 10.0);
    }

    #[test]
    fn test_normalize_angle() {
        let pi = std::f64::consts::PI;
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(2.0 * pi) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(-2.0 * pi) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(pi) - pi).abs() < 1e-10);
    }
}
