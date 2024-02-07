#[cfg(feature="with-hotend")]
pub use crate::hwa::device::{AdcHotendPeripheral, AdcHotendPin, PwmHotend};

#[cfg(feature="with-hotbed")]
pub use crate::hwa::device::{AdcHotbedPeripheral, AdcHotbedPin, PwmHotbed};

//////

#[cfg(feature="with-fan-layer")]
pub use crate::hwa::device::PwmFanLayer;

#[cfg(feature="with-fan-extra-1")]
pub use crate::hwa::device::PwmFanExtra1;

#[cfg(feature="with-laser")]
pub use crate::hwa::device::PwmLaser;
