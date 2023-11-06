//! TODO: Still incomplete
//! Disclaimer: No generic interface (so far) for supporting more TFT/LCD drivers rather than ILI9341
pub(crate) mod display_task;

#[cfg(feature = "ili9341_spi")]
pub mod ili9341_spi;

#[cfg(feature = "ili9341_parallel")]
pub mod ili9341_parallel;

pub mod ui;