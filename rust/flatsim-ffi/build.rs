// Build script for CXX bridge
fn main() {
    cxx_build::bridge("src/lib.rs")
        .flag_if_supported("-std=c++20")
        .compile("flatsim-ffi");

    println!("cargo:rerun-if-changed=src/lib.rs");
}
