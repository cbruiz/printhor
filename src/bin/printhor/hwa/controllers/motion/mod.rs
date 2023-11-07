pub(in crate::hwa) mod motion_controller;
pub(in crate::hwa) mod motion_segment;

#[cfg(feature = "with-motion")]
pub use motion_controller::*;