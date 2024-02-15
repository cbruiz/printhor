#[cfg(feature = "with-motion")]
pub(in crate::hwa) mod motion_driver;
#[cfg(all(feature = "native", feature = "plot-timings"))]
mod timing_monitor;
#[cfg(all(feature = "timing-stats", feature = "with-motion"))]
pub mod timing_stats;

#[cfg(feature = "with-motion")]
pub use motion_driver::MotionDriver;

#[cfg(feature = "with-motion")]
pub use motion_driver::MotionDriverParams;