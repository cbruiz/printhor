#[cfg(all(feature = "with-spi", feature = "sdcard-uses-spi"))]
mod spi;

#[cfg(all(feature = "with-spi", feature = "sdcard-uses-spi"))]
pub use spi::SPIAdapter as SPIAdapter;
