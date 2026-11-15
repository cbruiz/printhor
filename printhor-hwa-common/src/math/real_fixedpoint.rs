//! Real Fixed point module
#[allow(unused)]
use crate as hwa;
use crate::math;
use core::ops::*;
use rust_decimal::prelude::*;
use rust_decimal_macros::dec;

#[derive(Copy, Clone, Default)]
pub struct Real(pub Decimal);
pub type RealImpl = Decimal;

impl Real {
    pub fn new(num: i64, scale: u32) -> Self {
        Real(Decimal::new(num, scale))
    }
    pub const fn from_fixed(dec: Decimal) -> Self {
        Real(dec)
    }

    /// Never panics: NaN is 0 and out of range values saturate
    pub fn from_f32(num: f32) -> Self {
        match Decimal::from_f32_retain(num) {
            Some(d) => Real(d),
            None if num.is_nan() => Real(Decimal::ZERO),
            None if num > 0.0 => Real(Decimal::MAX),
            None => Real(Decimal::MIN),
        }
    }
    pub fn from_lit(num: i64, scale: u32) -> Self {
        Real(Decimal::new(num, scale))
    }
    pub fn inner(&self) -> Decimal {
        self.0
    }
    pub fn abs(self) -> Self {
        Real(self.0.abs())
    }
    pub fn powi(self, x: i32) -> Self {
        Real(self.0.powi(x as i64))
    }
    pub fn recip(self) -> Self {
        use num_traits::Inv;
        Real(self.0.inv())
    }
    pub const fn zero() -> Self {
        Real(Decimal::ZERO)
    }
    pub const fn is_zero(&self) -> bool {
        self.0.is_zero()
    }
    pub const fn epsilon() -> Self {
        crate::math::EPSILON
    }
    pub const fn is_positive(&self) -> bool {
        !self.0.is_sign_negative()
    }
    pub fn is_negligible(&self) -> bool {
        self.abs().lt(&math::EPSILON)
    }
    pub fn is_defined_positive(&self) -> bool {
        self.0 > dec!(0.0)
    }
    pub const fn one() -> Self {
        Real(dec!(1.0))
    }
    /// Rounds half away from zero (as the float backends)
    pub fn round(&self) -> Self {
        Real(self.0.round_dp_with_strategy(0, RoundingStrategy::MidpointAwayFromZero))
    }
    pub fn round_dp(&self, decimals: u32) -> Self {
        Real(self.0.round_dp_with_strategy(decimals, RoundingStrategy::MidpointAwayFromZero))
    }
    /// Round to decimal digits
    pub fn rdp(&self, digits: u32) -> Self {
        Real(self.0.round_dp_with_strategy(digits, RoundingStrategy::MidpointAwayFromZero))
    }
    pub fn ceil(&self) -> Self {
        Real(self.0.ceil())
    }
    pub fn floor(&self) -> Self {
        Real(self.0.floor())
    }
    pub fn to_f64(&self) -> f64 {
        // Every Decimal fits in f64 (with rounding): the fallback is never expected
        self.0.to_f64().unwrap_or(0.0)
    }
    /// Truncates toward zero (as the float backends and `int()`)
    pub fn to_i32(&self) -> Option<i32> {
        self.0.trunc().to_i32()
    }
    /// Truncates toward zero (as the float backends and `int()`)
    pub fn to_i64(&self) -> Option<i64> {
        self.0.trunc().to_i64()
    }
    /// Truncated integer part. Never panics: saturates out of range
    pub fn int(&self) -> i64 {
        match self.0.trunc().to_i64() {
            Some(v) => v,
            None if self.0.is_sign_negative() => i64::MIN,
            None => i64::MAX,
        }
    }

    /// Radians to degrees
    pub fn r2d(&self) -> Real {
        (*self) * hwa::make_real!(180.0) / hwa::math::PI
    }

    /// Degrees to radians
    /// 360 -> 2pi
    /// x ->
    pub fn d2r(&self) -> Real {
        (*self) * hwa::math::PI / hwa::make_real!(180.0)
    }

    pub fn cbrt(self) -> Self {
        Real::from_fixed(self.0.powd(math::THIRD.0))
    }

    pub fn sqrt(self) -> Option<Self> {
        if self.0.is_zero() {
            // Also a negated zero, whose sign rust_decimal keeps
            return Some(Real(Decimal::ZERO));
        }
        match self.0.sqrt() {
            None => None,
            Some(v) => Some(Real(v)),
        }
    }

    pub fn sin(self) -> Self {
        Real(self.0.sin())
    }

    pub fn cos(self) -> Self {
        Real(self.0.cos())
    }

    pub fn tan(self) -> Self {
        Real(self.0.tan())
    }

    /// Computes the four quadrant arctangent of self (y) and other (x) in radians.
    ///
    /// Native Decimal implementation: `f64::atan2` is not available in `core` (no_std).
    pub fn atan2(self, other: Real) -> Self {
        let (y, x) = (self.0, other.0);
        let r = if x.is_zero() {
            if y.is_zero() {
                Decimal::ZERO
            } else if y.is_sign_negative() {
                -Decimal::HALF_PI
            } else {
                Decimal::HALF_PI
            }
        } else {
            let a = Self::decimal_atan(y / x);
            if x.is_sign_positive() {
                a
            } else if y.is_sign_negative() {
                a - Decimal::PI
            } else {
                a + Decimal::PI
            }
        };
        Real(r)
    }

    /// Computes the arccosine of a number. Return value is in radians in the range [0, pi].
    /// The input is clamped to [-1, 1] (there is no NaN in fixed point).
    ///
    /// Native Decimal implementation: `f64::acos` is not available in `core` (no_std).
    pub fn acos(self) -> Self {
        let x = self.0.max(-Decimal::ONE).min(Decimal::ONE);
        let s = (Decimal::ONE - x * x).sqrt().unwrap_or(Decimal::ZERO);
        Real(s).atan2(Real(x))
    }

    /// Arctangent of a Decimal in radians, in the range [-pi/2, pi/2].
    fn decimal_atan(x: Decimal) -> Decimal {
        let negative = x.is_sign_negative();
        let mut a = x.abs();
        // atan(a) = pi/2 - atan(1/a) for a > 1
        let inverted = a > Decimal::ONE;
        if inverted {
            a = Decimal::ONE / a;
        }
        // Halve the angle 3 times: atan(a) = 2 * atan(a / (1 + sqrt(1 + a^2)))
        // leaving |a| <= tan(pi/32) ~= 0.098 so the Taylor series converges fast.
        const HALVINGS: u32 = 3;
        for _ in 0..HALVINGS {
            a = a / (Decimal::ONE + (Decimal::ONE + a * a).sqrt().unwrap_or(Decimal::ONE));
        }
        // Taylor series: a - a^3/3 + a^5/5 - ...
        let a2 = a * a;
        let mut power = a;
        let mut sum = Decimal::ZERO;
        let mut n = Decimal::ONE;
        let two = Decimal::TWO;
        for i in 0..40 {
            let term = power / n;
            if term.is_zero() {
                break;
            }
            if i % 2 == 0 {
                sum += term;
            } else {
                sum -= term;
            }
            power *= a2;
            n += two;
        }
        let mut r = sum * Decimal::from(1u32 << HALVINGS);
        if inverted {
            r = Decimal::HALF_PI - r;
        }
        if negative { -r } else { r }
    }

    pub fn ln(self) -> Self {
        Real(self.0.ln())
    }
    pub fn exp(self) -> Self {
        Real(self.0.exp())
    }

    pub fn sign(self) -> Self {
        let s = self.0.signum();
        if s.is_zero() { Real::one() } else { Real(s) }
    }

    pub fn vmin(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
        let mut m: Option<Real> = None;
        if let Some(x) = r1 {
            m = Some(x);
        }
        if let Some(y) = r2 {
            if let Some(mr) = &m {
                if mr.gt(&y) {
                    m = Some(y)
                }
            }
        }
        m
    }

    pub fn vmax(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
        let mut m: Option<Real> = None;
        if let Some(x) = r1 {
            m = Some(x);
        }
        if let Some(y) = r2 {
            if let Some(mr) = &m {
                if mr.lt(&y) {
                    m = Some(y)
                }
            }
        }
        m
    }
}

impl Add for Real {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Real(self.0.add(rhs.0))
    }
}

impl AddAssign for Real {
    fn add_assign(&mut self, rhs: Self) {
        self.0.add_assign(rhs.0)
    }
}

impl Sub for Real {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Real(self.0.sub(rhs.0))
    }
}

impl SubAssign for Real {
    fn sub_assign(&mut self, rhs: Self) {
        self.0.sub_assign(rhs.0)
    }
}

impl Mul for Real {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Real(self.0.mul(rhs.0))
    }
}

impl MulAssign for Real {
    fn mul_assign(&mut self, rhs: Self) {
        self.0.mul_assign(rhs.0)
    }
}

impl Div for Real {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Real(self.0.div(rhs.0))
    }
}

impl DivAssign for Real {
    fn div_assign(&mut self, rhs: Self) {
        self.0.div_assign(rhs.0)
    }
}

impl core::cmp::PartialOrd<Self> for Real {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        self.0.partial_cmp(&other.0)
    }

    fn lt(&self, other: &Self) -> bool {
        self.0.lt(&other.0)
    }

    fn le(&self, other: &Self) -> bool {
        self.0.le(&other.0)
    }

    fn gt(&self, other: &Self) -> bool {
        self.0.gt(&other.0)
    }

    fn ge(&self, other: &Self) -> bool {
        self.0.ge(&other.0)
    }
}

impl core::cmp::PartialEq<Self> for Real {
    fn eq(&self, other: &Self) -> bool {
        self.0.eq(&other.0)
    }
}

impl core::cmp::Eq for Real {}

impl core::cmp::Ord for Real {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.0.cmp(&other.0)
    }

    fn max(self, other: Self) -> Self
    where
        Self: Sized,
    {
        Real(self.0.max(other.0))
    }

    fn min(self, other: Self) -> Self
    where
        Self: Sized,
    {
        Real(self.0.min(other.0))
    }

    fn clamp(self, min: Self, max: Self) -> Self
    where
        Self: Sized,
        Self: core::cmp::PartialOrd,
    {
        Real(self.0.clamp(min.0, max.0))
    }
}

impl core::ops::Neg for Real {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Real(self.0.neg())
    }
}

impl core::fmt::Debug for Real {
    fn fmt(&self, _f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(_f, "{:?}", self.rdp(4).0)?;
        Ok(())
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for Real {
    fn format(&self, _f: defmt::Formatter) {
        defmt::write!(_f, "{:?}", self.round_dp(4).to_f64());
    }
}
