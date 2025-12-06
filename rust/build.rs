// Build script for CXX integration
use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    
    // We need to build the C++ project first with CMake
    // Then we can find the headers in the build directory
    let build_dir = manifest_dir.join("build");
    
    // Path to navcon and other dependencies (from CMake FetchContent)
    let navcon_include = manifest_dir.join("xtra/navcon/include");
    let concord_include = build_dir.join("_deps/concord-src/include");
    let pigment_include = build_dir.join("_deps/pigment-src/include");
    let entropy_include = build_dir.join("_deps/entropy-src/include");
    let flatsim_include = manifest_dir.join("include");
    let src_dir = manifest_dir.join("src");
    
    // Build the CXX bridge
    let mut build = cxx_build::bridge("src/main.rs");
    build
        .flag_if_supported("-std=c++20")
        .include(&flatsim_include)
        .include(&src_dir);
        
    // Add includes if they exist (after CMake build)
    if concord_include.exists() {
        build.include(&concord_include);
    }
    if navcon_include.exists() {
        build.include(&navcon_include);
    }
    if pigment_include.exists() {
        build.include(&pigment_include);
    }
    if entropy_include.exists() {
        build.include(&entropy_include);
    }
    
    // Add the shim C++ file
    build.file("src/rust_shim.cpp");
    
    build.compile("flatsim");

    println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=src/rust_shim.cpp");
    
    // Link to the C++ libraries built by CMake
    if build_dir.join("navcon").exists() {
        println!("cargo:rustc-link-search=native={}", build_dir.join("navcon").display());
        println!("cargo:rustc-link-lib=static=navcon");
    }
}
