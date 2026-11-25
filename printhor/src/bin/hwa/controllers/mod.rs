//! The printhor controllers. A set of abstractions over the [hwa::hwi] and HAL
#[allow(unused)]
use crate::hwa;

#[cfg(feature = "with-sd-card")]
pub mod sd_card_controller;

#[cfg(feature = "with-sd-card")]
pub use sd_card_controller::GenericSDCardController;
#[cfg(feature = "with-print-job")]
mod printer_controller;

#[cfg(feature = "with-print-job")]
pub use printer_controller::*;

#[cfg(feature = "with-motion")]
pub mod motion_control;

#[cfg(feature = "with-probe")]
mod servo_controller;
#[cfg(feature = "with-trinamic")]
mod trinamic_controller;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        mod adc_controller;
        pub use adc_controller::GenericAdcController;
        pub mod heater_controller;
        pub use heater_controller::{
            AutotuneCycle, AutotuneEnd, AutotuneOutcome, AutotuneRequest, HeaterConfig,
            HeaterController, HeaterError, HeaterFault, TuningRule,
        };
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed",
        feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))] {
        mod pwm_controller;
        pub use pwm_controller::GenericPwmController;
    }
}

#[cfg(feature = "with-trinamic")]
pub use trinamic_controller::{TrinamicController, TrinamicError, TrinamicHealth};

#[cfg(feature = "with-motion")]
pub use motion_control::*;

#[cfg(feature = "with-probe")]
pub use servo_controller::GenericServoController;

#[cfg(feature = "with-probe")]
pub use servo_controller::ProbeTrait;
