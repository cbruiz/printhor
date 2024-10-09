pub use crate::control::GCodeProcessor;
pub use printhor_hwa_common::*;

/// HWI contains the exports of the lower layer (Hardware Interface)
pub(in crate::hwa) mod hwi;

pub mod controllers;
pub mod drivers;
pub mod types;

//#region Main exports

// Isolate/decouple HWI export from board crates

//#endregion

#[cfg(feature = "with-sd-card")]
pub struct DummyTimeSource {}

#[cfg(feature = "with-sd-card")]
impl embedded_sdmmc::TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub use hwi::Contract;
