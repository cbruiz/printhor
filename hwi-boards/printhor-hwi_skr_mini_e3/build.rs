//! Exports the path of the board memory layout (`skr_mini_e3_v3/memory.x`) to the firmware build script,
//! which links it: `DEP_PRINTHOR_HWI_SKR_MINI_E3_MEMORY_X` (see `links` in Cargo.toml). This way it is found wherever this
//! crate is (the repository or the crates.io registry)

fn main() {
    let memory_x = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap())
        .join("skr_mini_e3_v3")
        .join("memory.x");
    println!("cargo::metadata=memory_x={}", memory_x.display());
    println!("cargo::rerun-if-changed=build.rs");
}
