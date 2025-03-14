#[allow(unused)]
use crate as hwa;
use hwa::math;
#[allow(unused)]
use num_traits::float::FloatCore;
use num_traits::{ToPrimitive, Zero};

type F = f64;
#[derive(Copy, Clone, Default)]
pub struct Real(pub F);
pub type RealImpl = f64;

impl Real {
    pub fn new(num: i64, scale: u32) -> Self {
        Self::from_lit(num, scale)
    }

    pub fn from_lit(num: i64, scale: u32) -> Self {
        let s = F::powi(10.0, scale as i32);
        Self((num as F) / s)
    }

    pub const fn from_f32(num: f32) -> Self {
        Self(num as f64)
    }

    pub const fn from_f64(num: f64) -> Self {
        Self(num)
    }

    pub const fn from_inner(num: f64) -> Self {
        Self(num)
    }

    pub fn inner(&self) -> f64 {
        self.0
    }

    pub fn abs(self) -> Self {
        Self(self.0.abs())
    }

    pub fn powi(self, x: i32) -> Self {
        Self(self.0.powi(x))
    }

    pub fn recip(self) -> Self {
        Self(FloatCore::recip(self.0))
    }

    pub const fn zero() -> Self {
        Self(0.0)
    }

    pub fn is_zero(&self) -> bool {
        F::is_zero(&self.0)
    }

    pub const fn epsilon() -> Self {
        crate::math::EPSILON
    }

    pub fn is_negligible(&self) -> bool {
        FloatCore::abs(self.0) < <f64 as FloatCore>::epsilon()
    }

    pub fn is_defined_positive(&self) -> bool {
        self.0 > 0.0
    }

    pub fn is_positive(&self) -> bool {
        !(self.0 < 0.0)
    }

    pub const fn one() -> Self {
        Self(1.0)
    }
    pub fn round(&self) -> Self {
        Real(self.0.round())
    }
    pub fn round_dp(&self, decimals: u32) -> Self {
        let s = F::powi(10.0, decimals as i32);
        Self((self.0 * s).round() / s)
    }
    /// Round to decimal digits
    pub fn rdp(&self, digits: u32) -> Self {
        self.round_dp(digits)
    }
    pub fn ceil(&self) -> Self {
        Real(self.0.ceil())
    }
    pub fn floor(&self) -> Self {
        Real(self.0.floor())
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

    //cbrt implementation from rust libm
    pub fn libm_cbrt(x: f64) -> f64 {
        const B1: u32 = 715094163; /* B1 = (1023-1023/3-0.03306235651)*2**20 */
        const B2: u32 = 696219795; /* B2 = (1023-1023/3-54/3-0.03306235651)*2**20 */

        const P0: f64 = 1.87595182427177009643; /* 0x3ffe03e6, 0x0f61e692 */
        const P1: f64 = -1.88497979543377169875; /* 0xbffe28e0, 0x92f02420 */
        const P2: f64 = 1.621429720105354466140; /* 0x3ff9f160, 0x4a49d6c2 */
        const P3: f64 = -0.758397934778766047437; /* 0xbfe844cb, 0xbee751d9 */
        const P4: f64 = 0.145996192886612446982; /* 0x3fc2b000, 0xd4e4edd7 */

        let x1p54 = f64::from_bits(0x4350000000000000); // 0x1p54 === 2 ^ 54

        let mut ui: u64 = x.to_bits();
        let mut r: f64;
        let s: f64;
        let mut t: f64;
        let w: f64;
        let mut hx: u32 = (ui >> 32) as u32 & 0x7fffffff;

        if hx >= 0x7ff00000 {
            /* cbrt(NaN,INF) is itself */
            return x + x;
        }

        /*
         * Rough cbrt to 5 bits:
         *    cbrt(2**e*(1+m) ~= 2**(e/3)*(1+(e%3+m)/3)
         * where e is integral and >= 0, m is real and in [0, 1), and "/" and
         * "%" are integer division and modulus with rounding towards minus
         * infinity.  The RHS is always >= the LHS and has a maximum relative
         * error of about 1 in 16.  Adding a bias of -0.03306235651 to the
         * (e%3+m)/3 term reduces the error to about 1 in 32. With the IEEE
         * floating point representation, for finite positive normal values,
         * ordinary integer divison of the value in bits magically gives
         * almost exactly the RHS of the above provided we first subtract the
         * exponent bias (1023 for doubles) and later add it back.  We do the
         * subtraction virtually to keep e >= 0 so that ordinary integer
         * division rounds towards minus infinity; this is also efficient.
         */
        if hx < 0x00100000 {
            /* zero or subnormal? */
            ui = (x * x1p54).to_bits();
            hx = (ui >> 32) as u32 & 0x7fffffff;
            if hx == 0 {
                return x; /* cbrt(0) is itself */
            }
            hx = hx / 3 + B2;
        } else {
            hx = hx / 3 + B1;
        }
        ui &= 1 << 63;
        ui |= (hx as u64) << 32;
        t = f64::from_bits(ui);

        /*
         * New cbrt to 23 bits:
         *    cbrt(x) = t*cbrt(x/t**3) ~= t*P(t**3/x)
         * where P(r) is a polynomial of degree 4 that approximates 1/cbrt(r)
         * to within 2**-23.5 when |r - 1| < 1/10.  The rough approximation
         * has produced t such than |t/cbrt(x) - 1| ~< 1/32, and cubing this
         * gives us bounds for r = t**3/x.
         *
         * Try to optimize for parallel evaluation as in __tanf.c.
         */
        r = (t * t) * (t / x);
        t = t * ((P0 + r * (P1 + r * P2)) + ((r * r) * r) * (P3 + r * P4));

        /*
         * Round t away from zero to 23 bits (sloppily except for ensuring that
         * the result is larger in magnitude than cbrt(x) but not much more than
         * 2 23-bit ulps larger).  With rounding towards zero, the error bound
         * would be ~5/6 instead of ~4/6.  With a maximum error of 2 23-bit ulps
         * in the rounded t, the infinite-precision error in the Newton
         * approximation barely affects third digit in the final error
         * 0.667; the error in the rounded t can be up to about 3 23-bit ulps
         * before the final error is larger than 0.667 ulps.
         */
        ui = t.to_bits();
        ui = (ui + 0x80000000) & 0xffffffffc0000000;
        t = f64::from_bits(ui);

        /* one step Newton iteration to 53 bits with error < 0.667 ulps */
        s = t * t; /* t*t is exact */
        r = x / s; /* error <= 0.5 ulps; |r| < |t| */
        w = t + t; /* t+t is exact */
        r = (r - t) / (w + r); /* r-t is exact; w+r ~= 3*t */
        t = t + t * r; /* error <= 0.5 + 0.5/3 + epsilon */
        t
    }

    /// Computes the cube root of the argument.
    pub fn cbrt(self) -> Self {
        let r = Real::from_inner(Real::libm_cbrt(self.0));
        r
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

    fn quake_isqrt(number: f64) -> f64 {
        let xhalf = number * 0.5;
        let mut y = f64::from_bits(
            0x5fe6ec85e7de30da_i64.wrapping_sub(number.to_bits() as i64 >> 1) as u64,
        );
        y = y * (1.5 - xhalf * y * y);
        y = y * (1.5 - xhalf * y * y);
        y = y * (1.5 - xhalf * y * y);
        y
    }

    pub fn sin(self) -> Self {
        Real(micromath::F32(self.0 as f32).sin().0 as f64)
    }

    pub fn cos(self) -> Self {
        Real(micromath::F32(self.0 as f32).cos().0 as f64)
    }

    pub fn tan(self) -> Self {
        Real(micromath::F32(self.0 as f32).tan().0 as f64)
    }

    /// Computes the four quadrant arctangent of self (y) and other (x) in radians.
    pub fn atan2(self, other: Real) -> Self {
        Real(
            micromath::F32(self.0 as f32)
                .atan2(micromath::F32(other.0 as f32))
                .0 as f64,
        )
    }

    /// Computes the arccosine of a number. Return value is in radians in the range [0, pi] or NaN if the number is outside the range [-1, 1].
    pub fn acos(self) -> Self {
        Real(self.0.acos())
    }

    #[inline]
    pub fn ln(self) -> Self {
        Real(micromath::F32(self.0 as f32).ln().0 as f64)
    }

    #[inline]
    pub fn exp(self) -> Self {
        Real(micromath::F32(self.0 as f32).exp().0 as f64)
    }

    #[inline]
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

impl PartialEq<Self> for Real {
    fn eq(&self, other: &Self) -> bool {
        self.0.eq(&other.0)
    }
}

impl Eq for Real {}

impl core::cmp::Ord for Real {
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
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:?}", self.0.to_f64());
    }
}
