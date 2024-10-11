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

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        mod adc_controller;
        pub use adc_controller::AdcController;
        mod heater_controller;
        pub use heater_controller::HeaterController;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed",
        feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))] {
        mod pwm_controller;
        pub use pwm_controller::PwmController;
    }
}

#[cfg(feature = "with-trinamic")]
pub use trinamic_controller::TrinamicController;

#[cfg(feature = "with-motion")]
pub use motion::*;

#[cfg(feature = "with-probe")]
pub use servo_controller::ServoController;

#[cfg(feature = "with-probe")]
pub use servo_controller::ProbeTrait;
