#[cfg(feature = "with-motion")]
pub(in crate::hwa) mod motion_driver;

#[cfg(feature = "with-motion")]
pub use motion_driver::MotionDriver;
#[cfg(feature = "with-motion")]
pub use motion_driver::MotionDriverParams;