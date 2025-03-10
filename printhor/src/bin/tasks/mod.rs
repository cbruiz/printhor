//! Module providing concurrent tasks for system, I/O, motion, actuators and sensors

pub mod task_control;
#[cfg(any(
    feature = "with-motion",
    feature = "with-hot-end",
    feature = "with-hot-bed"
))]
pub mod task_defer;

#[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))]
pub mod task_motion_broadcast;

#[cfg(any(test, feature = "integration-test"))]
pub mod task_integration;
#[cfg(feature = "with-print-job")]
pub mod task_print_job;
#[cfg(feature = "with-motion")]
pub mod task_stepper;
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
pub mod task_temperature;
