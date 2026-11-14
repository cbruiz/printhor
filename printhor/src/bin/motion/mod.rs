//! The Motion profile trait and its implementations
//!
mod micro_segment_interpolation;
#[cfg(feature = "with-motion-chaining")]
mod motion_chaining;
mod motion_profile_hybrid;
mod motion_profile_traits;
mod segment_sampling;
#[cfg(all(test, feature = "perf-bench"))]
mod perf_bench;
// Re-exports

pub use micro_segment_interpolation::*;
#[cfg(feature = "with-motion-chaining")]
pub use motion_chaining::*;
pub use motion_profile_hybrid::*;
pub use motion_profile_traits::*;
pub use segment_sampling::*;

#[cfg(test)]
mod motion_test {

    #[test]
    fn discrete_positioning_case_1() {}

    #[test]
    fn discrete_positioning_case_2() {}

    #[test]
    fn discrete_positioning_case_3() {}
}
