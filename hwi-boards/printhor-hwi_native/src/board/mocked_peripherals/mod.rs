#[cfg(feature = "needs-adc")]
mod mocked_adc;
mod mocked_pin;
#[cfg(feature = "with-hotbed")]
mod mocked_pwm;
#[cfg(feature = "with-spi")]
mod mocked_spi;
#[cfg(feature = "with-sdcard")]
mod mocked_sdcard;
#[cfg(feature = "with-uart-port-1")]
mod mocked_uart;

#[cfg(feature = "with-display")]
mod mocked_display;

mod mocked_wdt;

#[allow(unused)]
pub use mocked_pin::*;
pub use mocked_wdt::*;

#[cfg(feature = "with-uart-port-1")]
pub use mocked_uart::*;

#[cfg(feature = "with-spi")]
pub use mocked_spi::*;

#[cfg(feature = "with-sdcard")]
pub use mocked_sdcard::*;

#[cfg(feature = "with-display")]
pub use mocked_display::*;
