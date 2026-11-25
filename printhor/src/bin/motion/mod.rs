//! The Motion profile trait and its implementations
//!
mod micro_segment_interpolation;
#[cfg(feature = "with-motion-chaining")]
mod motion_chaining;
mod motion_profile_hybrid;
mod motion_profile_traits;
/// Input shaping (feature `with-input-shaping`, `M593`)
#[cfg(feature = "with-input-shaping")]
mod input_shaping;
/// Pressure advance (feature `with-pressure-advance`, `M900`)
#[cfg(feature = "with-pressure-advance")]
mod pressure_advance;
mod segment_sampling;
/// On-target benchmarks (M801)
#[cfg(feature = "mcu-bench")]
pub mod mcu_bench;
#[cfg(all(test, feature = "perf-bench"))]
mod perf_bench;
#[cfg(all(test, feature = "op-count"))]
mod op_cost;
// Re-exports

pub use micro_segment_interpolation::*;
#[cfg(feature = "with-motion-chaining")]
pub use motion_chaining::*;
#[cfg(test)]
pub use motion_chaining::counters as motion_chaining_counters;
pub use motion_profile_hybrid::*;
pub use motion_profile_traits::*;
#[cfg(feature = "with-input-shaping")]
pub use input_shaping::*;
#[cfg(feature = "with-pressure-advance")]
pub use pressure_advance::*;
pub use segment_sampling::*;

#[cfg(all(feature = "with-pressure-advance", not(feature = "with-e-axis")))]
compile_error!("feature `with-pressure-advance` requires `with-e-axis`");

#[cfg(test)]
mod motion_test {

    #[test]
    fn discrete_positioning_case_1() {}

    #[test]
    fn discrete_positioning_case_2() {}

    #[test]
    fn discrete_positioning_case_3() {}
}
