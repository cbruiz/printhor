#[allow(unused)]
use std::io::Write;

fn main() {

    cfg_if::cfg_if! {
        if #[cfg(feature="rp_2040")] {
            let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
            let memory_x_path = std::path::PathBuf::from(".")
                .join("hwi-boards")
                .join("printhor-hwi_rp_2040")
                .join("memory.x");
            let memory_x_path_str = memory_x_path.as_path().to_str().unwrap();
            std::fs::File::create(out.join("memory.x"))
                .unwrap()
                .write_all(std::fs::read(memory_x_path_str).unwrap().as_slice())
                .unwrap();
            println!("cargo:rustc-link-search={}", out.display());
            println!("cargo:rerun-if-changed={}", memory_x_path_str);
        }
        else if #[cfg(all(feature="skr_mini_e3_v2", feature="with-bootloader"))] {
            let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
            let memory_x_path = std::path::PathBuf::from(".")
                .join("hwi-boards")
                .join("printhor-hwi_skr_mini_e3")
                .join("skr_mini_e3_v2")
                .join("memory.x");
            let memory_x_path_str = memory_x_path.as_path().to_str().unwrap();
            std::fs::File::create(out.join("memory.x"))
                .unwrap()
                .write_all(std::fs::read(memory_x_path_str).unwrap().as_slice())
                .unwrap();
            println!("cargo:rustc-link-search={}", out.display());
            println!("cargo:rerun-if-changed={}", memory_x_path_str);
        }
        else if #[cfg(all(feature="skr_mini_e3_v3", feature="with-bootloader"))] {
            let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
            let memory_x_path = std::path::PathBuf::from(".")
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

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    #[cfg(not(feature = "native"))]
    {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
    }
    cfg_if::cfg_if! {
        if #[cfg(feature="rp_2040")] {
            println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
        }
    }

    #[cfg(feature = "with-defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    // make native lvgl more platform independent
    //#[cfg(feature="native")]
    //println!("cargo:rustc-link-arg-bins=-L/opt/homebrew/lib");

}
