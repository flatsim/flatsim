// Build script for CXX bridge
fn main() {
    cxx_build::bridge("src/lib.rs")
        .flag_if_supported("-std=c++20")
        .flag_if_supported("-Wno-cpp") // Suppress _FORTIFY_SOURCE warning in debug builds
        .flag_if_supported("-U_FORTIFY_SOURCE") // Undefine _FORTIFY_SOURCE for debug builds
        .compile("flatsim-ffi");

    println!("cargo:rerun-if-changed=src/lib.rs");
}
