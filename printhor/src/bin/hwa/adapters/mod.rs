#[cfg(all(feature = "with-spi", feature = "sd-card-uses-spi"))]
mod spi;

#[cfg(all(feature = "with-spi", feature = "sd-card-uses-spi"))]
pub use spi::SPIAdapter;
