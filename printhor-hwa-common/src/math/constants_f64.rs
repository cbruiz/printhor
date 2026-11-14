use crate::math::Real;

pub const ZERO: Real = Real::from_f64(0.0);
pub const HALF: Real = Real::from_f64(0.5);
pub const ONE: Real = Real::from_f64(1.0);
pub const ONE_AND_HALF: Real = Real::from_f64(1.5);
pub const TWO: Real = Real::from_f64(2.0);
pub const THREE: Real = Real::from_f64(3.0);
pub const FOUR: Real = Real::from_f64(4.0);
pub const EIGHT: Real = Real::from_f64(8.0);
pub const NINE: Real = Real::from_f64(9.0);
pub const SIX: Real = Real::from_f64(6.0);
pub const SIXTH: Real = Real::from_f64(1.0 / 6.0);

pub const TWENTY_SEVEN: Real = Real::from_f64(27.0);


/// A very small value used to determine the difference between two floating point numbers.
/// This constant is useful for comparisons to handle floating point negligible differences.
pub const EPSILON: Real = Real::from_f64(f64::EPSILON);

#[allow(unused)]
pub const ONE_HUNDRED: Real = Real::from_f64(100.0);
#[allow(unused)]
pub const ONE_THOUSAND: Real = Real::from_f64(1000.0);
#[allow(unused)]
pub const ONE_MILLION: Real = Real::from_f64(1000000.0);

pub const PI: Real = Real::from_f64(core::f64::consts::PI);

// Motion-profile polynomial constants (kept here so every backend gets them as `const`).
/// 1/3
pub const THIRD: Real = Real::from_f64(1.0 / 3.0);
/// 2/3
pub const TWO_THIRDS: Real = Real::from_f64(2.0 / 3.0);
/// 1/10
pub const TENTH: Real = Real::from_f64(0.1);
/// 5
pub const FIVE: Real = Real::from_f64(5.0);
/// 10
pub const TEN: Real = Real::from_f64(10.0);
/// 12
pub const TWELVE: Real = Real::from_f64(12.0);
/// 15
pub const FIFTEEN: Real = Real::from_f64(15.0);
/// 20
pub const TWENTY: Real = Real::from_f64(20.0);
/// 30
pub const THIRTY: Real = Real::from_f64(30.0);
/// 60
pub const SIXTY: Real = Real::from_f64(60.0);
/// 7
pub const SEVEN: Real = Real::from_f64(7.0);
/// 1/4
pub const QUARTER: Real = Real::from_f64(0.25);
/// 1/5
pub const FIFTH: Real = Real::from_f64(0.2);
