use crate::math::Real;
use rust_decimal_macros::dec;

pub const ZERO: Real = Real::from_fixed(dec!(0.0));
pub const HALF: Real = Real::from_fixed(dec!(0.5));
pub const ONE: Real = Real::from_fixed(dec!(1.0));
pub const ONE_AND_HALF: Real = Real::from_fixed(dec!(1.5));
pub const TWO: Real = Real::from_fixed(dec!(2.0));
pub const THREE: Real = Real::from_fixed(dec!(3.0));
pub const THIRD: Real = Real::from_fixed(dec!(0.3333333333333333333333333333));
pub const FOUR: Real = Real::from_fixed(dec!(4.0));
pub const SIX: Real = Real::from_fixed(dec!(6.0));
pub const EIGHT: Real = Real::from_fixed(dec!(8.0));
pub const NINE: Real = Real::from_fixed(dec!(9.0));
pub const SIXTH: Real = Real::from_fixed(dec!(0.1666666666666666666666666666));

pub const TWENTY_SEVEN: Real = Real::from_fixed(dec!(27.0));


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

// Motion-profile polynomial constants (kept here so every backend gets them as `const`).
/// 2/3
pub const TWO_THIRDS: Real = Real::from_fixed(dec!(0.6666666666666666666666666667));
/// 1/10
pub const TENTH: Real = Real::from_fixed(dec!(0.1));
/// 5
pub const FIVE: Real = Real::from_fixed(dec!(5.0));
/// 10
pub const TEN: Real = Real::from_fixed(dec!(10.0));
/// 12
pub const TWELVE: Real = Real::from_fixed(dec!(12.0));
/// 15
pub const FIFTEEN: Real = Real::from_fixed(dec!(15.0));
/// 20
pub const TWENTY: Real = Real::from_fixed(dec!(20.0));
/// 30
pub const THIRTY: Real = Real::from_fixed(dec!(30.0));
/// 60
pub const SIXTY: Real = Real::from_fixed(dec!(60.0));
/// 7
pub const SEVEN: Real = Real::from_fixed(dec!(7.0));
/// 1/4
pub const QUARTER: Real = Real::from_fixed(dec!(0.25));
/// 1/5
pub const FIFTH: Real = Real::from_fixed(dec!(0.2));
