#[cfg(any(feature = "with-hotbed", feature = "with-hotend"))]
mod mocked_adc;

mod mocked_pin;
pub(crate) use mocked_pin::init_pin_state;
#[cfg(any(feature = "with-hotbed", feature = "with-hotend", feature = "with-fan-layer"))]
mod mocked_pwm;
#[cfg(feature = "with-spi")]
mod mocked_spi;
#[cfg(feature = "with-sdcard")]
mod mocked_sdcard;
#[cfg(feature = "with-uart-port-1")]
mod mocked_uart;

#[cfg(feature = "with-trinamic")]
mod mocked_trinamic;

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

#[cfg(feature = "with-trinamic")]
pub use mocked_trinamic::*;

#[cfg(feature = "with-display")]
pub use mocked_display::*;

#[cfg(any(feature = "with-hotbed", feature = "with-hotend", feature = "with-fan-layer"))]
pub use mocked_pwm::*;

#[cfg(any(feature = "with-hotbed", feature = "with-hotend"))]
pub use mocked_adc::*;