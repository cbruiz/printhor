//! The Motion profile trait and its implementations
//!
mod motion_profile_traits;
mod motion_profile_s_curve;
mod micro_segment_interpolation;
mod motion_chaining;
mod segment_sampling;
// Re-exports

pub use motion_profile_traits::*;
pub use motion_profile_s_curve::*;
pub use segment_sampling::*;
pub use micro_segment_interpolation::*;
pub use motion_chaining::*;


#[cfg(test)]
mod motion_test {

    #[test]
    fn discrete_positioning_case_1() {
        
    }

    #[test]
    fn discrete_positioning_case_2() {
        
    }

    #[test]
    fn discrete_positioning_case_3() {
        
    }
    
}
