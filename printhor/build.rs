#[allow(unused)]
use std::io::Write;

fn main() {
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
        else if #[cfg(feature="rp_2040")] {
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
        else if #[cfg(feature="skr_mini_e3_v3")] {
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
}
