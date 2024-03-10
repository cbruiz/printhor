use core::f64;
use crate::math::Real;

pub const ZERO: Real = Real::from_f64(0.0);
pub const HALF: Real = Real::from_f64(0.5);
pub const ONE: Real = Real::from_f64(1.0);
pub const TWO: Real = Real::from_f64(2.0);
pub const THREE: Real = Real::from_f64(3.0);
pub const FOUR: Real = Real::from_f64(4.0);
pub const SIX: Real = Real::from_f64(6.0);
pub const SIXTH: Real = Real::from_f64(1.0 / 6.0);
#[allow(unused)]
pub const ONE_HUNDRED: Real = Real::from_f64(100.0);
#[allow(unused)]
pub const ONE_THOUSAND: Real = Real::from_f64(1000.0);
#[allow(unused)]
pub const ONE_MILLION: Real = Real::from_f64(1000000.0);

pub const PI: Real = Real::from_f64(f64::consts::PI);

