#[cfg(any(feature = "with-hot-bed", feature = "with-hot-end"))]
mod mocked_adc;

mod mocked_pin;
pub(crate) use mocked_pin::init_pin_state;
#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
mod mocked_pwm;
#[cfg(feature = "with-spi")]
mod mocked_spi;
#[cfg(feature = "with-sdcard")]
mod mocked_sdcard;
#[cfg(any(feature = "with-serial-port-1",feature = "with-serial-port-usb"))]
mod mocked_uart;
#[cfg(feature = "with-serial-port-2")]
mod mocked_uart_sink;

#[cfg(feature = "with-trinamic")]
mod mocked_trinamic;

#[cfg(feature = "with-display")]
mod mocked_display;

mod mocked_wdt;

#[allow(unused)]
pub use mocked_pin::*;

pub use mocked_wdt::*;

#[cfg(feature = "with-serial-port-1")]
pub use mocked_uart::*;

#[cfg(feature = "with-serial-port-2")]
pub use mocked_uart_sink::*;

#[cfg(feature = "with-spi")]
pub use mocked_spi::*;

#[cfg(feature = "with-sdcard")]
pub use mocked_sdcard::*;

#[cfg(feature = "with-trinamic")]
pub use mocked_trinamic::*;

#[cfg(feature = "with-display")]
pub use mocked_display::*;

#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
pub use mocked_pwm::*;

#[cfg(any(feature = "with-hot-bed", feature = "with-hot-end"))]
pub use mocked_adc::*;