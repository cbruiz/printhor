pub use crate::control::GCodeProcessor;
use crate::hwi;
pub use printhor_hwa_common::*;

pub mod controllers;
pub mod drivers;

//#region Main exports

pub use hwi::*;

//#endregion

pub mod mem {
    use crate::{hwa, hwi};
    pub fn heap_current_size() -> u32 {
        hwi::heap_current_size()
    }
    pub fn heap_max_size() -> usize {
        hwi::HEAP_SIZE_BYTES
    }

    pub fn stack_reservation_max_size() -> u32 {
        hwa::MAX_STATIC_ALLOC_BYTES as u32
    }

    pub fn stack_reservation_current_size() -> u32 {
        hwi::stack_reservation_current_size()
    }

    #[allow(unused)]
    pub fn stack_reservation_usage_percentage() -> f32 {
        let alloc = stack_reservation_current_size() as f32;
        let max = stack_reservation_max_size() as f32;
        (100.0f32 * alloc) / max
    }
}

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
