cfg_if::cfg_if! {
    if #[cfg(feature="float-point-f64-impl")] {
        #[allow(unused)]
        use crate as hwa;
        use core::cmp::Ordering;
        use core::ops::*;
        use core::fmt::{Debug, Formatter};
        #[cfg(feature = "with-defmt")]
        use defmt::Format;
        use num_traits::{ToPrimitive, Zero};
        #[allow(unused)]
        use num_traits::float::FloatCore;

        type F = f64;
        #[derive(Copy, Clone, Default, Debug)]
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

            pub fn sqrt(self) -> Option<Self> {
                //#[cfg(feature = "native")]
                //let v = self.0.sqrt();
                let v = 1.0f64 / Self::quake_isqrt(self.0);
                //let v = 1.0f64 / core::intrinsics::sqrt(self.0);
                //let x = micromath::F32Ext::sqrt(self.0 as f32);
                //#[cfg(not(feature = "native"))]
                //let v = micromath::F32(self.0 as f32).sqrt().0; // Pfffff.....
                if v.is_nan() {
                    None
                }
                else {
                    Some(Self(v as f64))
                }
            }

            fn quake_isqrt(number: f64) -> f64 {
                let mut i: i64 = number.to_bits() as i64;
                i = 0x5fe6ec85e7de30da_i64.wrapping_sub(i >> 1);
                let y = f64::from_bits(i as u64);
                y * (1.5 - (number * 0.5 * y * y))
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
            pub fn atan2(self, other:Real) -> Self {
                Real(micromath::F32(self.0 as f32).atan2(micromath::F32(other.0 as f32)).0 as f64)
            }

            /// Computes the arccosine of a number. Return value is in radians in the range [0, pi] or NaN if the number is outside the range [-1, 1].
            pub fn acos(self) -> Self {
                //Real(micromath::F32(self.0 as f32).acos().0 as f64)
                todo!("....")
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
                if s.is_zero() {Real::one()} else {Real(s)}
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

        impl PartialOrd<Self> for Real {

            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
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

        impl Eq for Real {

        }

        impl Ord for Real {
            fn cmp(&self, other: &Self) -> Ordering {
                self.0.total_cmp(&other.0)
            }

            fn max(self, other: Self) -> Self where Self: Sized {
                Real(self.0.max(other.0))
            }

            fn min(self, other: Self) -> Self where Self: Sized {
                Real(self.0.min(other.0))
            }

            fn clamp(self, min: Self, max: Self) -> Self where Self: Sized, Self: PartialOrd {
                Real(self.0.clamp(min.0, max.0))
            }
        }

        impl Neg for Real {
            type Output = Self;

            fn neg(self) -> Self::Output {
                Real(self.0.neg())
            }
        }

        #[cfg(feature = "with-defmt")]
        impl Format for Real {
            fn format(&self, fmt: defmt::Formatter) {
                defmt::write!(fmt, "{:?}", self.0.to_f64());
            }
        }
    }
}
