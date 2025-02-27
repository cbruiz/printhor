use crate::math;

use crate as hwa;
use micromath::F32Ext;
use num_traits::ToPrimitive;
use num_traits::Zero;
use num_traits::float::FloatCore;

#[derive(Copy, Clone, Default)]
pub struct Real(pub f32);
pub type RealImpl = f32;

impl Real {
    pub fn new(num: i64, scale: u32) -> Self {
        Self::from_lit(num, scale)
    }

    pub const fn from_inner(num: f32) -> Self {
        Self(num)
    }

    pub fn from_lit(num: i64, scale: u32) -> Self {
        let s = FloatCore::powi(10.0f32, scale as i32);
        Self((num as f32) / s)
    }

    pub const fn from_f32(num: f32) -> Self {
        Self(num)
    }

    pub fn inner(&self) -> f32 {
        self.0
    }

    pub fn abs(self) -> Self {
        Self(FloatCore::abs(self.0))
    }

    pub fn powi(self, x: i32) -> Self {
        Self(FloatCore::powi(self.0, x))
    }

    pub fn recip(self) -> Self {
        Self(FloatCore::recip(self.0))
    }

    pub const fn zero() -> Self {
        Self(0.0f32)
    }

    pub fn is_zero(&self) -> bool {
        f32::is_zero(&self.0)
    }

    pub const fn epsilon() -> Self {
        crate::math::EPSILON
    }

    pub fn is_negligible(&self) -> bool {
        FloatCore::abs(self.0) < <f32 as FloatCore>::epsilon()
    }

    pub fn is_defined_positive(&self) -> bool {
        self.0 > f32::zero()
    }

    pub fn is_positive(&self) -> bool {
        !self.0.is_sign_negative()
    }

    pub const fn one() -> Self {
        Self(1.0f32)
    }

    pub fn round(&self) -> Self {
        Real(FloatCore::round(self.0))
    }

    pub fn round_dp(&self, decimals: u32) -> Self {
        let s = FloatCore::powi(10.0f32, decimals as i32);
        Self(FloatCore::round(self.0 * s) / s)
    }
    /// Round to decimal digits

    pub fn rdp(&self, digits: u32) -> Self {
        self.round_dp(digits)
    }

    pub fn ceil(&self) -> Self {
        Real(FloatCore::ceil(self.0))
    }

    pub fn floor(&self) -> Self {
        Real(FloatCore::floor(self.0))
    }

    pub fn to_f64(&self) -> f64 {
        self.0 as f64
    }

    pub fn to_i32(&self) -> Option<i32> {
        self.0.to_i32()
    }

    pub fn to_i64(&self) -> Option<i64> {
        self.0.to_i64()
    }

    pub fn int(&self) -> i64 {
        self.0.to_i64().unwrap()
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

    pub fn sqrt(self) -> Option<Self> {
        if self < math::ZERO {
            None
        } else if self == math::ZERO {
            Some(math::ZERO)
        } else {
            // The famous inverse square root approximation of 'ID Software'
            let r = math::ONE / Real::from_inner(Self::quake_isqrt(self.inner()));
            if self < math::EPSILON {
                hwa::trace!("Very small sqrt calculation: sqrt({:?}) = {:?} ", self.0, r);
            }
            Some(r)
        }
    }

    /// The famous inverse square root approximation of 'ID Software'
    /// Hard-coded with 3 iterations
    fn quake_isqrt(number: f32) -> f32 {
        let xhalf = number * 0.5f32;
        let mut y =
            f32::from_bits(0x5F375A86_i32.wrapping_sub(number.to_bits() as i32 >> 1) as u32);
        y = y * (1.5f32 - xhalf * y * y);
        y = y * (1.5f32 - xhalf * y * y);
        y = y * (1.5f32 - xhalf * y * y);
        y
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
    pub fn atan2(self, other: Real) -> Self {
        Real(self.0.atan2(other.0))
    }

    /// Computes the arccosine of a number. Return value is in radians in the range [0, pi] or NaN if the number is outside the range [-1, 1].
    pub fn acos(self) -> Self {
        Real(self.0.acos())
    }

    pub fn ln(self) -> Self {
        Real(self.0.ln())
    }

    pub fn exp(self) -> Self {
        Real(self.0.exp())
    }

    pub fn sign(self) -> Self {
        let s = F32Ext::signum(self.0);
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

impl core::ops::Add for Real {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Real(self.0.add(rhs.0))
    }
}

impl core::ops::AddAssign for Real {
    fn add_assign(&mut self, rhs: Self) {
        self.0.add_assign(rhs.0)
    }
}

impl core::ops::Sub for Real {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Real(self.0.sub(rhs.0))
    }
}

impl core::ops::SubAssign for Real {
    fn sub_assign(&mut self, rhs: Self) {
        self.0.sub_assign(rhs.0)
    }
}

impl core::ops::Mul for Real {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Real(self.0.mul(rhs.0))
    }
}

impl core::ops::MulAssign for Real {
    fn mul_assign(&mut self, rhs: Self) {
        self.0.mul_assign(rhs.0)
    }
}

impl core::ops::Div for Real {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Real(self.0.div(rhs.0))
    }
}

impl core::ops::DivAssign for Real {
    fn div_assign(&mut self, rhs: Self) {
        self.0.div_assign(rhs.0)
    }
}

impl PartialOrd<Self> for Real {
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

impl PartialEq<Self> for Real {
    fn eq(&self, other: &Self) -> bool {
        self.0.eq(&other.0)
    }
}

impl Eq for Real {}

impl Ord for Real {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.0.total_cmp(&other.0)
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
        Self: PartialOrd,
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
        let mut buffer = ryu::Buffer::new();
        let slice = buffer.format(self.0);
        core::write!(_f, "{}", slice)?;
        Ok(())
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for Real {
    fn format(&self, _f: defmt::Formatter) {
        defmt::write!(_f, "{}", self.0);
    }
}
