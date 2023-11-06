#[cfg(feature = "with-spi")]
mod spi;

#[cfg(feature = "with-spi")]
pub use spi::SPIAdapter as SPIAdapter;
