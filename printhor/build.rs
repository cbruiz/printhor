#[allow(unused)]
use std::io::Write;

fn main() {
    let doc_in = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap())
        .join("img")
        .join("contract-class-diagram.png");
    let out = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap())
        .join("..")
        .join("target")
        .join("doc")
        .join("img");
    std::fs::create_dir_all(&out).unwrap();
    std::fs::copy(doc_in, out.join("contract-class-diagram.png")).unwrap();

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
            let memory_x_path = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap())
                .join("..")
                .join("hwi-boards")
                .join("printhor-hwi_rp_2040")
                .join("rpi-pico")
                .join("memory.x");
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
                    let memory_x_path = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap())
                        .join("..")
                        .join("hwi-boards")
                        .join("printhor-hwi_mks_robin_nano")
                        .join("mks_robin_nano_3_1")
                        .join("memory.x");
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
                    let memory_x_path = std::path::PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").unwrap())
                        .join("..")
                        .join("hwi-boards")
                        .join("printhor-hwi_skr_mini_e3")
                        .join("skr_mini_e3_v3")
                        .join("memory.x");
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
