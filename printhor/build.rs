#[allow(unused)]
use std::io::Write;

/// The memory layout (`memory.x`) of a board crate: its build script exports the path (`links`
/// metadata), so it is found wherever the board crate is (this repository or the crates.io registry)
#[allow(unused)]
fn board_memory_x(dep_env: &str) -> std::path::PathBuf {
    std::path::PathBuf::from(
        std::env::var_os(dep_env)
            .unwrap_or_else(|| panic!("{} is not set: the board crate does not export its memory.x", dep_env)),
    )
}

fn main() {
    // The class diagram of the documentation (design/architecture.md), where the rustdoc output
    // expects it (`target/doc/img`). In the repository checkout only: built from the crates.io
    // registry, `..` is the registry itself (and docs.rs does not document binary crates)
    let manifest_dir = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let workspace_dir = manifest_dir.join("..");
    if workspace_dir.join("printhor-hwa-common").join("Cargo.toml").is_file() {
        let doc_in = manifest_dir.join("img").join("contract-class-diagram.png");
        let out = workspace_dir.join("target").join("doc").join("img");
        std::fs::create_dir_all(&out).unwrap();
        std::fs::copy(doc_in, out.join("contract-class-diagram.png")).unwrap();
    }

    cfg_if::cfg_if! {
        if #[cfg(feature="native")] {

        }
        else if #[cfg(feature="nucleo64-f410rb")] {
            println!("cargo:rustc-link-arg-bins=--nmagic");
            println!("cargo:rustc-link-arg-bins=-Tlink.x");
        }
        else if #[cfg(feature="nucleo64-l476rg")] {
            println!("cargo:rustc-link-arg-bins=--nmagic");
            println!("cargo:rustc-link-arg-bins=-Tlink.x");
        }
        else if #[cfg(feature="rp-2040")] {
            let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
            let memory_x_path = board_memory_x("DEP_PRINTHOR_HWI_RP_2040_MEMORY_X");
            let memory_x_path_str = memory_x_path.as_path().to_str().unwrap();
            std::fs::File::create(out.join("memory.x"))
                .unwrap()
                .write_all(std::fs::read(memory_x_path_str).unwrap().as_slice())
                .unwrap();
            println!("cargo:rustc-link-search={}", out.display());
            println!("cargo:rerun-if-changed={}", memory_x_path_str);

            println!("cargo:rustc-link-arg-bins=--nmagic");
            println!("cargo:rustc-link-arg-bins=-Tlink.x");
            println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
        }
        else if #[cfg(feature="mks_robin_nano")] {
            cfg_if::cfg_if! {
                if #[cfg(feature="without-bootloader")] {
                }
                else {
                    let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
                    let memory_x_path = board_memory_x("DEP_PRINTHOR_HWI_MKS_ROBIN_NANO_MEMORY_X");
                    let memory_x_path_str = memory_x_path.as_path().to_str().unwrap();
                    std::fs::File::create(out.join("memory.x"))
                        .unwrap()
                        .write_all(std::fs::read(memory_x_path_str).unwrap().as_slice())
                        .unwrap();
                    println!("cargo:rustc-link-search={}", out.display());
                    println!("cargo:rerun-if-changed={}", memory_x_path_str);
                }
            }

            println!("cargo:rustc-link-arg-bins=--nmagic");
            println!("cargo:rustc-link-arg-bins=-Tlink.x");
        }
        else if #[cfg(any(feature="skr_mini_e3_v3", feature="skr_mini_e3_v3_fdm"))] {
            cfg_if::cfg_if! {
                if #[cfg(feature="without-bootloader")] {
                }
                else {
                    let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
                    let memory_x_path = board_memory_x("DEP_PRINTHOR_HWI_SKR_MINI_E3_MEMORY_X");
                    let memory_x_path_str = memory_x_path.as_path().to_str().unwrap();
                    std::fs::File::create(out.join("memory.x"))
                        .unwrap()
                        .write_all(std::fs::read(memory_x_path_str).unwrap().as_slice())
                        .unwrap();
                    println!("cargo:rustc-link-search={}", out.display());
                    println!("cargo:rerun-if-changed={}", memory_x_path_str);
                }
            }
            println!("cargo:rustc-link-arg-bins=--nmagic");
            println!("cargo:rustc-link-arg-bins=-Tlink.x");
        }
        else {
            //compile_error!("No board set");
        }
    }

    #[cfg(feature = "with-defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Identical code folding (rust-lld) on the boards: the functions whose machine code is the
    // same (e.g. `__aeabi_memcpy4`/`8`, empty functions, generic instances compiled to the same
    // code) share a single copy. ~1 KB of flash on the SKR Mini E3, the executed code is unchanged.
    // `all` and not `safe`: rustc emits no address-significance tables, so `safe` folds nothing.
    // Rust does not guarantee distinct addresses for distinct functions anyway (it merges them
    // itself). Not in unoptimized builds: a debugger would show a folded function by another name.
    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let opt_level = std::env::var("OPT_LEVEL").unwrap_or_default();
    if target_arch == "arm" && target_os == "none" && opt_level != "0" {
        println!("cargo:rustc-link-arg-bins=--icf=all");
    }
}
