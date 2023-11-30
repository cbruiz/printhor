#[cfg(feature = "with-sdcard")]
pub mod sdcard_controller;

#[cfg(feature = "with-sdcard")]
pub use sdcard_controller::CardController;
#[cfg(feature = "with-printjob")]
mod printer_controller;

#[cfg(feature = "with-printjob")]
pub use printer_controller::*;

#[cfg(feature = "with-motion")]
mod motion;
#[cfg(feature = "with-trinamic")]
mod trinamic_controller;
#[cfg(feature = "with-probe")]
mod servo_controller;

#[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
mod heater_controller;

#[cfg(any(feature = "with-hotend", feature = "with-hotbed", feature = "with-fan0", feature = "with-fan-layer"))]
mod pwm_controller;

// Use

#[cfg(feature = "with-trinamic")]
pub use trinamic_controller::TrinamicController;

#[cfg(feature = "with-motion")]
pub use motion::*;

#[cfg(feature = "with-probe")]
pub use servo_controller::ServoController;

#[cfg(feature = "with-probe")]
pub type ServoControllerRef = printhor_hwa_common::ControllerRef<ServoController>;

#[cfg(feature = "with-probe")]
pub use servo_controller::ProbeTrait;

#[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
pub use heater_controller::HeaterController;

////

#[cfg(any(feature = "with-hotend"))]
pub type HotendController = HeaterController<crate::hwa::devices::AdcHotendPeripheral, crate::hwa::devices::AdcHotendPin, crate::hwa::devices::PwmHotend>;

#[cfg(any(feature = "with-hotend"))]
pub type HotendControllerRef = printhor_hwa_common::ControllerRef<HotendController>;

#[cfg(any(feature = "with-hotend"))]
pub type HotendPwmController = pwm_controller::PwmController<crate::hwa::devices::PwmHotend>;

////

#[cfg(any(feature = "with-hotbed"))]
pub type HotbedController = HeaterController<crate::hwa::devices::AdcHotbedPeripheral, crate::hwa::devices::AdcHotbedPin, crate::hwa::devices::PwmHotbed>;

#[cfg(any(feature = "with-hotbed"))]
pub type HotbedControllerRef = printhor_hwa_common::ControllerRef<HotbedController>;

#[cfg(any(feature = "with-hotbed"))]
pub type HotbedPwmController = pwm_controller::PwmController<crate::hwa::devices::PwmHotbed>;

////
#[cfg(any(feature = "with-fan0"))]
pub type Fan0PwmController = pwm_controller::PwmController<crate::hwa::devices::PwmFan0>;

#[cfg(any(feature = "with-fan0"))]
pub type Fan0PwmControllerRef = printhor_hwa_common::ControllerRef<Fan0PwmController>;

#[cfg(any(feature = "with-fan-layer"))]
pub type LayerPwmController = pwm_controller::PwmController<crate::hwa::devices::PwmLayerFan>;

#[cfg(any(feature = "with-fan-layer"))]
pub type LayerPwmControllerRef = printhor_hwa_common::ControllerRef<LayerPwmController>;

#[cfg(any(feature = "with-laser"))]
pub type LaserPwmController = pwm_controller::PwmController<crate::hwa::devices::PwmLaser>;

#[cfg(any(feature = "with-laser"))]
pub type LaserPwmControllerRef = printhor_hwa_common::ControllerRef<LaserPwmController>;