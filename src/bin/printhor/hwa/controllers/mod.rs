#[allow(unused)]
use crate::hwa;

#[cfg(feature = "with-sd-card")]
pub mod sd_card_controller;

#[cfg(feature = "with-sd-card")]
pub use sd_card_controller::CardController;
#[cfg(feature = "with-print-job")]
mod printer_controller;

#[cfg(feature = "with-print-job")]
pub use printer_controller::*;

#[cfg(feature = "with-motion")]
pub mod motion;

#[cfg(feature = "with-probe")]
mod servo_controller;
#[cfg(feature = "with-trinamic")]
mod trinamic_controller;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
mod heater_controller;

#[cfg(any(
    feature = "with-hot-end",
    feature = "with-hot-bed",
    feature = "with-fan-layer",
    feature = "with-fan-extra-1",
    feature = "with-laser"
))]
mod pwm_controller;

// Use

#[cfg(feature = "with-trinamic")]
pub use trinamic_controller::TrinamicController;

#[cfg(feature = "with-motion")]
pub use motion::*;

#[cfg(feature = "with-probe")]
pub use servo_controller::ServoController;

#[cfg(feature = "with-probe")]
pub use servo_controller::ProbeTrait;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub use heater_controller::HeaterController;

////

#[cfg(any(feature = "with-hot-end"))]
pub type HotEndController = HeaterController<
    hwa::AdcHotEndMutexType,
    hwa::HotEndControllerMutexType,
    hwa::device::AdcHotEndPeripheral,
    hwa::device::AdcHotEndPin,
    hwa::device::PwmHotEnd,
>;

#[cfg(any(feature = "with-hot-end"))]
pub type HotEndPwmController =
    pwm_controller::PwmController<hwa::HotEndControllerHolderType<hwa::device::PwmHotEnd>,>;

////

#[cfg(any(feature = "with-hot-bed"))]
pub type HotBedController = HeaterController<
    hwa::AdcHotBedMutexType,
    hwa::PwmHotBedMutexType,
    hwa::device::AdcHotBedPeripheral,
    hwa::device::AdcHotBedPin,
    hwa::device::PwmHotBed,
>;

#[cfg(any(feature = "with-hot-bed"))]
pub type HotbedPwmController =
    pwm_controller::PwmController<hwa::PwmHotBedMutexType, hwa::device::PwmHotBed>;

#[cfg(any(feature = "with-fan-layer"))]
pub type FanLayerPwmController =
    pwm_controller::PwmController<hwa::PwmFanLayerMutexType, hwa::device::PwmFanLayer>;

#[cfg(any(feature = "with-fan-extra-1"))]
pub type FanExtra1PwmController =
    pwm_controller::PwmController<hwa::FanExtra1ControllerMutexType, hwa::device::PwmFanExtra1>;

#[cfg(any(feature = "with-laser"))]
pub type LaserPwmController =
    pwm_controller::PwmController<hwa::PwmLaserMutexType, hwa::device::PwmLaser>;
