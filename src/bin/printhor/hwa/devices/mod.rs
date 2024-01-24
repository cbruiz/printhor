#[cfg(feature="with-hotend")]
pub use crate::hwa::device::{AdcHotendPeripheral, AdcHotendPin, PwmHotend};

#[cfg(feature="with-hotbed")]
pub use crate::hwa::device::{AdcHotbedPeripheral, AdcHotbedPin, PwmHotbed};

//////

#[cfg(feature="with-fan0")]
pub use crate::hwa::device::PwmFan0;

#[cfg(feature="with-fan-layer")]
pub use crate::hwa::device::PwmLayerFan;

#[cfg(feature="with-laser")]
pub use crate::hwa::device::PwmLaser;
