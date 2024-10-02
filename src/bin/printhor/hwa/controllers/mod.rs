#[cfg(feature = "with-sdcard")]
pub mod sdcard_controller;

#[allow(unused)]
use printhor_hwa_common::ControllerMutexType;
#[cfg(feature = "with-sdcard")]
pub use sdcard_controller::CardController;
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

#[cfg(feature = "with-ps-on")]
pub type PsOnRef =
    printhor_hwa_common::ControllerRef<ControllerMutexType, crate::hwa::device::PsOnPin>;

#[cfg(feature = "with-probe")]
pub type ServoControllerRef = printhor_hwa_common::InterruptControllerRef<ServoController>;

#[cfg(feature = "with-probe")]
pub use servo_controller::ProbeTrait;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub use heater_controller::HeaterController;

////

#[cfg(any(feature = "with-hot-end"))]
pub type HotendController = HeaterController<
    crate::hwa::device::AdcHotendPeripheral,
    crate::hwa::device::AdcHotendPin,
    crate::hwa::device::PwmHotend,
>;

#[cfg(any(feature = "with-hot-end"))]
pub type HotendControllerRef = printhor_hwa_common::InterruptControllerRef<HotendController>;

#[cfg(any(feature = "with-hot-end"))]
pub type HotendPwmController = pwm_controller::PwmController<crate::hwa::device::PwmHotend>;

////

#[cfg(any(feature = "with-hot-bed"))]
pub type HotbedController = HeaterController<
    crate::hwa::device::AdcHotbedPeripheral,
    crate::hwa::device::AdcHotbedPin,
    crate::hwa::device::PwmHotbed,
>;

#[cfg(any(feature = "with-hot-bed"))]
pub type HotbedControllerRef = printhor_hwa_common::InterruptControllerRef<HotbedController>;

#[cfg(any(feature = "with-hot-bed"))]
pub type HotbedPwmController = pwm_controller::PwmController<crate::hwa::device::PwmHotbed>;

#[cfg(any(feature = "with-fan-layer"))]
pub type FanLayerPwmController = pwm_controller::PwmController<crate::hwa::device::PwmFanLayer>;

#[cfg(any(feature = "with-fan-layer"))]
pub type FanLayerPwmControllerRef =
    printhor_hwa_common::InterruptControllerRef<FanLayerPwmController>;

#[cfg(any(feature = "with-fan-extra-1"))]
pub type FanExtra1PwmController = pwm_controller::PwmController<crate::hwa::device::PwmFanExtra1>;

#[cfg(any(feature = "with-fan-extra-1"))]
pub type FanExtra1PwmControllerRef =
    printhor_hwa_common::InterruptControllerRef<FanExtra1PwmController>;

#[cfg(any(feature = "with-laser"))]
pub type LaserPwmController = pwm_controller::PwmController<crate::hwa::device::PwmLaser>;

#[cfg(any(feature = "with-laser"))]
pub type LaserPwmControllerRef = printhor_hwa_common::InterruptControllerRef<LaserPwmController>;
