//! The Motion profile trait and its implementations
//!
mod micro_segment_interpolation;
mod motion_chaining;
mod motion_profile_s_curve;
mod motion_profile_traits;
mod segment_sampling;
// Re-exports

pub use micro_segment_interpolation::*;
pub use motion_chaining::*;
pub use motion_profile_s_curve::*;
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
