fn main() {
    #[cfg(not(feature = "native"))]
    {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
    }
    #[cfg(feature = "with-defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    // TODO: make native lvgl more platform independent
    #[cfg(feature="native")]
    println!("cargo:rustc-link-arg-bins=-L/opt/homebrew/lib");

}
