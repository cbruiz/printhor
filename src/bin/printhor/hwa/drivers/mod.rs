#[cfg(feature = "with-motion")]
pub(in crate::hwa) mod motion_driver;
#[cfg(all(feature = "native", feature = "plot-timings"))]
mod timing_monitor;

#[cfg(feature = "with-motion")]
pub use motion_driver::MotionDriver;
#[cfg(feature = "with-motion")]
pub use motion_driver::MotionDriverParams;