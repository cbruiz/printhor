use crate::math::Real;
use rust_decimal_macros::dec;

pub const ZERO: Real = Real::from_fixed(dec!(0.0));
pub const HALF: Real = Real::from_fixed(dec!(0.5));
pub const ONE: Real = Real::from_fixed(dec!(1.0));
pub const ONE_AND_HALF: Real = Real::from_fixed(dec!(1.5));
pub const TWO: Real = Real::from_fixed(dec!(2.0));
pub const THREE: Real = Real::from_fixed(dec!(3.0));
pub const FOUR: Real = Real::from_fixed(dec!(4.0));
pub const SIX: Real = Real::from_fixed(dec!(6.0));
pub const SIXTH: Real = Real::from_fixed(dec!(0.1666666666666666666666666666));

/// A very small value used to determine the difference between two decimal point numbers.
/// ZERO in fixed point representation, for obvious reasons.
pub const EPSILON: Real = Real::from_fixed(dec!(0.0000000000000000000000000001));

#[allow(unused)]
pub const ONE_HUNDRED: Real = Real::from_fixed(dec!(100.0));
#[allow(unused)]
pub const ONE_THOUSAND: Real = Real::from_fixed(dec!(1000.0));
#[allow(unused)]
pub const ONE_MILLION: Real = Real::from_fixed(dec!(1000000.0));

pub const PI: Real = Real::from_fixed(rust_decimal::Decimal::PI);
