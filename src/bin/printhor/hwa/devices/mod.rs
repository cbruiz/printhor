//#[cfg(feature="with-usbserial")]
//pub use crate::hwi::device::USBSerialDevice;

#[cfg(feature="with-usbserial")]
pub use crate::hwi::device::USBSerialDeviceInputStream;

#[cfg(feature="with-uart-port-1")]
pub use crate::hwi::device::UartPort1RxInputStream;

//#[cfg(any(feature="with-probe", feature = "with-hotbed", feature = "with-hotend"))]
//pub use crate::hwa::device::PwmChannel;

//#[cfg(feature="with-probe")]
//pub use crate::hwa::device::PwmServo;

#[cfg(any(feature="with-hotend", feature = "with-hotbed"))]
pub use crate::hwa::device::{AdcImpl, AdcTrait, AdcPinTrait};

#[cfg(feature="with-hotend")]
pub use crate::hwa::device::{AdcHotendPeripheral, AdcHotendPin, PwmHotend};

#[cfg(feature="with-hotbed")]
pub use crate::hwa::device::{AdcHotbedPeripheral, AdcHotbedPin, PwmHotbed};

//////

#[cfg(feature="with-fan0")]
pub use crate::hwa::device::PwmFan0;

#[cfg(feature="with-fan1")]
pub use crate::hwa::device::PwmFan1;

#[cfg(feature="with-fan1")]
pub use crate::hwa::device::PwmLaser;

//#[cfg(any(feature="with-hotend", feature = "with-hotbed", feature = "with-fan0", feature = "with-fan1"))]
//pub use crate::hwa::device::{PwmTrait, PwmImpl};


