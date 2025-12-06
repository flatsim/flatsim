//! Flatsim Rust Main
//! 
//! Calling C++ navcon library from Rust using CXX

#[cxx::bridge(namespace = "flatsim")]
mod ffi {
    unsafe extern "C++" {
        include!("rust_shim.hpp");
        
        // Use concord::Point as an opaque type
        #[namespace = "concord"]
        type Point;
        
        // Accessor functions for Point (in flatsim namespace)
        fn point_x(p: &Point) -> f64;
        fn point_y(p: &Point) -> f64;
        
        // Factory function to create a point (in flatsim namespace)
        fn create_point(x: f64, y: f64) -> UniquePtr<Point>;
        
        // Calculate distance between two points (in flatsim namespace)
        fn calculate_distance(p1: &Point, p2: &Point) -> f64;
    }
}

fn main() {
    println!("=== Flatsim: Calling C++ from Rust ===\n");
    
    let p1 = ffi::create_point(10.0, 20.0);
    let p2 = ffi::create_point(0.0, 0.0);
    
    println!("Point 1: ({}, {})", ffi::point_x(&p1), ffi::point_y(&p1));
    println!("Point 2: ({}, {})", ffi::point_x(&p2), ffi::point_y(&p2));
    
    let distance = ffi::calculate_distance(&p1, &p2);
    println!("Distance: {:.2}", distance);
    
    println!("\n=== Done ===");
}
