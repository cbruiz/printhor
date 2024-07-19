cfg_if::cfg_if! {
    if #[cfg(feature="float-point-f64-impl")] {
        use core::cmp::Ordering;
        use core::ops::*;
        use core::fmt::{Debug, Display, Formatter};
        #[cfg(feature = "with-defmt")]
        use defmt::Format;
        use num_traits::{ToPrimitive, Zero};
        #[allow(unused)]
        use num_traits::float::FloatCore;

        type F = f64;
        #[derive(Copy, Clone, Default, Debug)]
        pub struct Real(pub F);
        pub type RealImpl = f64;

        #[allow(dead_code)]
        impl Real {
            #[inline]
            pub(crate) fn new(num: i64, scale: u32) -> Self {
                Self::from_lit(num, scale)
            }
            #[inline]
            pub(crate) fn from_lit(num: i64, scale: u32) -> Self {
                let s = F::powi(10.0, scale as i32);
                Self((num as F) / s)
            }
            #[inline]
            pub const fn from_f32(num: f32) -> Self {
                Self(num as f64)
            }
            #[inline]
            pub const fn from_f64(num: f64) -> Self {
                Self(num)
            }
            #[inline]
            pub fn inner(&self) -> f64 {
                self.0
            }
            #[inline]
            pub(crate) fn abs(self) -> Self {
                Self(self.0.abs())
            }
            #[inline]
            pub(crate) fn powi(self, x: i32) -> Self {
                Self(self.0.powi(x))
            }
            #[inline]
            pub const fn zero() -> Self {
                Self(0.0)
            }
            #[inline]
            pub(crate) fn is_zero(&self) -> bool {
                F::is_zero(&self.0)
            }

            #[inline]
            pub fn is_negligible(&self) -> bool {
                FloatCore::abs(self.0) < <f64 as FloatCore>::epsilon()
            }

            pub(crate) fn is_defined_positive(&self) -> bool {
                self.0 > 0.0
            }

            #[inline]
            pub(crate) fn is_positive(&self) -> bool {
                !(self.0 < 0.0)
            }
            #[inline]
            pub(crate) const fn one() -> Self {
                Self(1.0)
            }
            #[inline]
            pub(crate) fn round(&self) -> Self {
                Real(self.0.round())
            }
            #[inline]
            pub(crate) fn round_dp(&self, decimals: u32) -> Self {
                let s = F::powi(10.0, decimals as i32);
                Self((self.0 * s).round() / s)
            }
            /// Round to decimal digits
            #[inline]
            pub fn rdp(&self, digits: u32) -> Self {
                self.round_dp(digits)
            }
            #[inline]
            pub(crate) fn ceil(&self) -> Self {
                Real(self.0.ceil())
            }
            #[inline]
            pub(crate) fn floor(&self) -> Self {
                Real(self.0.floor())
            }
            #[inline]
            pub fn to_f64(&self) -> f64 {
                self.0 as f64
            }

            #[inline]
            pub fn to_i32(&self) -> Option<i32> {
                self.0.to_i32()
            }

            #[inline]
            pub fn to_i64(&self) -> Option<i64> {
                self.0.to_i64()
            }

            #[inline]
            pub fn int(&self) -> i64 {
                self.0.to_i64().unwrap()
            }

            #[inline]
            pub(crate) fn sqrt(self) -> Option<Self> {
                //#[cfg(feature = "native")]
                let v = self.0.sqrt();
                //let v = 1.0f64 / Self::quake_isqrt(self.0);
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

            #[inline]
            pub(crate) fn cos(self) -> Self {
                Real(micromath::F32(self.0 as f32).cos().0 as f64)
            }

            #[inline]
            pub(crate) fn ln(self) -> Self {
                Real(micromath::F32(self.0 as f32).ln().0 as f64)
            }

            #[inline]
            pub(crate) fn exp(self) -> Self {
                Real(micromath::F32(self.0 as f32).exp().0 as f64)
            }

            #[inline]
            pub(crate) fn sign(self) -> Self {
                let s = self.0.signum();
                if s.is_zero() {Real::one()} else {Real(s)}
            }

            pub(crate) fn vmin(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
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

            pub(crate) fn vmax(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
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

        impl Display for Real {
            fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
                core::fmt::Display::fmt(&self.0, f)
            }
        }

        #[cfg(feature = "with-defmt")]
        impl Format for Real {
            fn format(&self, fmt: defmt::Formatter) {
                defmt::write!(fmt, "{:?}", self.0.to_f64());
            }
        }

        #[derive(Clone)]
        pub struct RealInclusiveRange {
            current: Real,
            current_back: Real,
            step_size: Real,
            start: Real,
            end: Real,
            num_steps: Real,
        }

        impl RealInclusiveRange {
            pub fn new(start: Real, end: Real, step_size: Real) -> Self {
                let num_steps = ((end - start) / step_size).ceil();
                #[cfg(feature = "native")]
                println!("RealInclusiveRange: start={} end={} step_size={}, num_steps={}", start, end, step_size, num_steps);
                RealInclusiveRange {
                    current: start,
                    current_back: start,
                    step_size,
                    start,
                    end,
                    num_steps,
                }
            }

            #[allow(unused)]
            #[inline]
            pub fn width(&self) -> Real {
                self.end - self.start
            }

            #[inline]
            #[allow(unused)]
            pub fn step_size(&self) -> Real {
                self.step_size
            }

            #[allow(unused)]
            pub fn reset(&mut self) {
                self.current = self.start;
                self.current_back = self.start;
            }

            /// panics (in debug) when len doesn't fit in usize
            fn usize_len(&self) -> usize {
                self.num_steps.0.to_usize().unwrap()
            }

            /// Does scale
            #[allow(unused)]
            pub fn scale(&self, scale: Real) -> Self {
                RealInclusiveRange::new(self.start * scale, self.end * scale, self.num_steps)
            }
        }

        impl Iterator for RealInclusiveRange {
            type Item = Real;

            fn next(&mut self) -> Option<Self::Item> {
                if !self.step_size.is_zero() && self.current <= self.end {
                    self.current_back = self.current.clone();
                    self.current += self.step_size.clone();
                    Some(self.current_back.clone())
                }
                else {
                    None
                }
            }

            fn count(self) -> usize {
                self.usize_len()
            }
        }

        impl DoubleEndedIterator for RealInclusiveRange {
            fn next_back(&mut self) -> Option<Self::Item> {
                if self.current_back >= self.end {
                    self.current = self.current_back;
                    self.current_back -= self.step_size;
                    Some(self.current)
                }
                else {
                    None
                }
            }
        }
    }
}