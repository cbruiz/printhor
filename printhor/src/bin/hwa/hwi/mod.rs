//! The hardware/bare-metal *abstraction* interface
//! this is a simple strict-typing abstraction with adapters/proxies

//#region "Boards exports as HWI"

cfg_if::cfg_if! {
    if #[cfg(feature="skr_mini_e3")] {
        pub use printhor_hwi_skr_mini_e3::*;
    }
    else if #[cfg(feature="mks_robin_nano")] {
        pub use printhor_hwi_mks_robin_nano::*;
    }
    else if #[cfg(feature="nucleo_64_arduino_cnc_hat")] {
        pub use printhor_hwi_nucleo_64_arduino_cnc_hat::*;
    }
    else if #[cfg(feature="rp-2040")] {
        pub use printhor_hwi_rp_2040::*;
    }
    else if #[cfg(feature="native")] {
        pub use printhor_hwi_native::*;
    }
    else {
        compile_error!("You need to select a board");
    }
}
//#endregion
