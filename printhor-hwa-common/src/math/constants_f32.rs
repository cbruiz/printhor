use crate::math::Real;
pub const ZERO: Real = Real::from_f32(0.0);
pub const HALF: Real = Real::from_f32(0.5);
pub const ONE_AND_HALF: Real = Real::from_f32(1.5);
pub const ONE: Real = Real::from_f32(1.0);
pub const TWO: Real = Real::from_f32(2.0);
/// 3.0
pub const THREE: Real = Real::from_f32(3.0);
/// 4.0
pub const FOUR: Real = Real::from_f32(4.0);
/// 6.0
pub const SIX: Real = Real::from_f32(6.0);
pub const EIGHT: Real = Real::from_f32(8.0);
pub const NINE: Real = Real::from_f32(9.0);
/// 1.0/6.0
pub const SIXTH: Real = Real::from_f32(1.0 / 6.0);

pub const TWENTY_SEVEN: Real = Real::from_f32(27.0);

/// A very small value used to determine the difference between two floating point numbers.
/// This constant is useful for comparisons to handle floating point negligible differences.
pub const EPSILON: Real = Real::from_f32(f32::EPSILON);

#[allow(unused)]
pub const ONE_HUNDRED: Real = Real::from_f32(100.0);
#[allow(unused)]
pub const ONE_THOUSAND: Real = Real::from_f32(1000.0);
#[allow(unused)]
pub const ONE_MILLION: Real = Real::from_f32(1000000.0);
#[allow(unused)]
pub const PI: Real = Real::from_f32(core::f32::consts::PI);

// Motion-profile polynomial constants (kept here so every backend gets them as `const`).
/// 1/3
pub const THIRD: Real = Real::from_f32(1.0 / 3.0);
/// 2/3
pub const TWO_THIRDS: Real = Real::from_f32(2.0 / 3.0);
/// 1/10
pub const TENTH: Real = Real::from_f32(0.1);
/// 5
pub const FIVE: Real = Real::from_f32(5.0);
/// 10
pub const TEN: Real = Real::from_f32(10.0);
/// 12
pub const TWELVE: Real = Real::from_f32(12.0);
/// 15
pub const FIFTEEN: Real = Real::from_f32(15.0);
/// 20
pub const TWENTY: Real = Real::from_f32(20.0);
/// 30
pub const THIRTY: Real = Real::from_f32(30.0);
/// 60
pub const SIXTY: Real = Real::from_f32(60.0);
/// 7
pub const SEVEN: Real = Real::from_f32(7.0);
/// 1/4
pub const QUARTER: Real = Real::from_f32(0.25);
/// 1/5
pub const FIFTH: Real = Real::from_f32(0.2);
