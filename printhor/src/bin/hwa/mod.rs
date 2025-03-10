//! The Hardware Abstraction Module
pub use printhor_hwa_common::*;

/// HWI contains the exports of the lower layer (Hardware Interface)
pub(in crate::hwa) mod hwi;

pub mod controllers;
pub mod drivers;
pub mod types;

//#region Main exports

// Isolate/decouple HWI export from board crates

//#endregion

pub use hwi::Contract;
