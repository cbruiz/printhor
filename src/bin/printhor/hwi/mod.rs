/// The hardware/baremetal *abstraction* interface
/// this is a simple strict-typing abstraction with adapters/proxies

//#region boards

#[cfg(feature = "skr_mini_e3")]
pub use printhor_hwi_skr_mini_e3::*;

#[cfg(feature = "mks_robin_nano")]
pub use printhor_hwi_mks_robin_nano::*;

#[cfg(feature = "nucleo_64_arduino_cnc_hat")]
pub use printhor_hwi_nucleo_64_arduino_cnc_hat::*;

#[cfg(feature = "native")]
pub use printhor_hwi_native::*;

#[cfg(feature = "native")]
#[allow(unused)]
pub use printhor_hwi_native::*;

//#endregion

pub mod adapters;

#[allow(unused)]
use crate::hwi::adapters::*;

#[cfg(feature = "with-fan")]
use crate::hwi::fan::FanController;
