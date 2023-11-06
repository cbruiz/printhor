//! This module provides a numeric abstraction so anyone can use the proper backend for specific MCU
#[cfg(feature = "fixed-point-128-impl")]
mod real_fixedpoint;
#[cfg(feature = "fixed-point-128-impl")]
mod constants_fixedpoint;
#[cfg(feature = "fixed-point-128-impl")]
pub use real_fixedpoint::Real;
#[cfg(feature = "fixed-point-128-impl")]
pub use real_fixedpoint::RealInclusiveRange;
#[cfg(feature = "fixed-point-128-impl")]
pub use constants_fixedpoint::*;

#[cfg(feature = "float-point-f32-impl")]
mod real_f32;
#[cfg(feature = "float-point-f32-impl")]
mod constants_f32;
#[cfg(feature = "float-point-f32-impl")]
pub use real_f32::Real;
#[cfg(feature = "float-point-f32-impl")]
pub use real_f32::RealInclusiveRange;
#[cfg(feature = "float-point-f32-impl")]
pub use constants_f32::*;

#[cfg(feature = "float-point-f64-impl")]
mod real_f64;
#[cfg(feature = "float-point-f64-impl")]
mod constants_f64;

#[cfg(feature = "float-point-f64-impl")]
pub use real_f64::Real;

#[cfg(feature = "float-point-f64-impl")]
pub use real_f64::RealInclusiveRange;

#[cfg(feature = "float-point-f64-impl")]
pub use constants_f64::*;