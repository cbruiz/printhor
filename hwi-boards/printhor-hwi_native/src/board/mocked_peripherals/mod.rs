#[cfg(any(feature = "with-hot-bed", feature = "with-hot-end"))]
mod mocked_adc;

pub(crate) mod mocked_pin;
#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
mod mocked_pwm;
#[cfg(feature = "with-spi")]
mod mocked_spi;
#[cfg(feature = "with-sd-card")]
mod mocked_sd_card;
#[cfg(any(feature = "with-serial-port-1",feature = "with-serial-port-usb"))]
mod mocked_uart;
#[cfg(feature = "with-serial-port-2")]
mod mocked_uart_sink;

#[cfg(feature = "with-trinamic")]
mod mocked_trinamic;

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

#[cfg(feature = "with-sd-card")]
pub use mocked_sd_card::*;

#[cfg(feature = "with-trinamic")]
pub use mocked_trinamic::*;

#[cfg(any(feature = "with-probe", feature = "with-hot-bed", feature = "with-hot-end", feature = "with-fan-layer", feature = "with-laser", feature = "with-fan-extra-1"))]
pub use mocked_pwm::*;

#[cfg(any(feature = "with-hot-bed", feature = "with-hot-end"))]
pub use mocked_adc::*;