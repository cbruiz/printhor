#[allow(unused)]
use std::io::Write;

fn main() {

    cfg_if::cfg_if! {
        if #[cfg(feature="rp_2040")] {
            let out = &std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
            let memory_x_path = std::path::PathBuf::from(".").join("hwi-boards").join("rp_2040_memory.x");
            std::fs::File::create(out.join("memory.x"))
                .unwrap()
                .write_all(std::fs::read(memory_x_path.as_path().to_str().unwrap()).unwrap().as_slice())
                .unwrap();
            println!("cargo:rustc-link-search={}", out.display());
            println!("cargo:rerun-if-changed=memory.x");
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
    #[cfg(feature = "with-defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    // make native lvgl more platform independent
    //#[cfg(feature="native")]
    //println!("cargo:rustc-link-arg-bins=-L/opt/homebrew/lib");

}
