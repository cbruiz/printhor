cfg_if::cfg_if! {
    if #[cfg(feature="float-point-f32-impl")] {
        #[allow(unused)]
        use crate as hwa;
        use core::cmp::Ordering;
        use core::ops::*;
        use core::fmt::{Debug, Formatter};
        #[cfg(feature = "with-defmt")]
        use defmt::Format;
        use num_traits::float::FloatCore;
        use num_traits::{Zero};
        use num_traits::ToPrimitive;
        use micromath::F32Ext;

        #[derive(Copy, Clone, Default)]
        pub struct Real(pub f32);
        pub type RealImpl = f32;

        #[allow(dead_code)]
        impl Real {
            #[inline]
            pub fn new(num: i64, scale: u32) -> Self {
                Self::from_lit(num, scale)
            }
            #[inline]
            pub const fn from_inner(num: f32) -> Self {
                Self(num)
            }
            #[inline]
            pub fn from_lit(num: i64, scale: u32) -> Self {
                let s = FloatCore::powi(10.0f32, scale as i32);
                Self((num as f32) / s)
            }
            #[inline]
            pub const fn from_f32(num: f32) -> Self {
                Self(num)
            }
            #[inline]
            pub fn inner(&self) -> f32 {
                self.0
            }
            #[inline]
            pub fn abs(self) -> Self {
                Self(FloatCore::abs(self.0))
            }
            #[inline]
            pub fn powi(self, x: i32) -> Self {
                Self(FloatCore::powi(self.0, x))
            }

            #[inline]
            pub fn recip(self) -> Self {
                Self(FloatCore::recip(self.0))
            }

            #[inline]
            pub const fn zero() -> Self {
                Self(0.0f32)
            }
            #[inline]
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
                let num = self.inner();
                // The famous inverse square root approximation of 'ID Software'
                let _r = 1.0f32 / Self::quake_isqrt(num);

                // Micromath crate, based on https://bits.stephan-brumme.com/squareRoot.html
                //let _r = micromath::F32(num).sqrt().0;
                // ARM intrinsics, https://developer.arm.com/architectures/instruction-sets/intrinsics/vrsqrtes_f32
                //let _r3 = unsafe { Inv::inv(core::arch::aarch64::vrsqrtes_f32(num)) };

                //let _r = 0.1f32 / Self::quake_isqrt(100.0f32 * num);

                if _r.is_nan() {
                    None
                }
                else {
                    Some(Self(_r))
                }


            }

            fn quake_isqrt(number: f32) -> f32 {

                let xhalf = number * 0.5f32;
                let mut y = f32::from_bits(0x5F375A86_i32.wrapping_sub(number.to_bits() as i32 >> 1) as u32);
                y = y * (1.5f32 - xhalf * y * y);
                //log::info!("sqrt({}) = {} [i1]", number, y);
                y = y * (1.5f32 - xhalf * y * y);
                //log::info!("sqrt({}) = {} [i2]", number, y);
                y = y * (1.5f32 - xhalf * y * y);
                //log::info!("sqrt({}) = {} [i3]", number, 1.0f32 / y);
                y
            }

            fn iterative_sqrt(_number: f32) -> f32 {
                /* TODO: migrate
                https://www.pertinentdetail.org/sqrt
                https://github.com/SolraBizna/ieee-apsqrt/blob/main/src/lib.rs
                https://github.com/ARM-software/CMSIS_4/blob/master/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.c
                https://forum.mikroe.com/viewtopic.php?t=65263
                https://www.ganssle.com/approx-2.htm
                https://www.reddit.com/r/rust/comments/1722v9d/help_with_sqrt_hardware_implementation_on_arm/
                uint32 isqrt3(uint32 n)
                {
                    uint32 root = 0, bit, trial;
                    bit = (n >= 0x10000) ? 1<<30 : 1<<14;
                    do
                    {
                        trial = root+bit;
                        if (n >= trial)
                        {
                        n -= trial;
                        root = trial+bit;
                        }
                        root >>= 1;
                        bit >>= 2;
                    } while (bit);
                    return root;
                }

                 */
                todo!("Not implemented")
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
            pub fn atan2(self, other:Real) -> Self {
                Real(self.0.atan2(other.0))
            }

            /// Computes the arccosine of a number. Return value is in radians in the range [0, pi] or NaN if the number is outside the range [-1, 1].
            pub fn acos(self) -> Self {
                Real(self.0.acos())
            }

            #[inline]
            pub fn ln(self) -> Self {
                Real(self.0.ln())
            }

            #[inline]
            pub fn exp(self) -> Self {Real(self.0.exp()) }

            #[inline]
            pub fn sign(self) -> Self {
                let s = F32Ext::signum(self.0);
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

        impl Debug for Real {
            fn fmt(&self, _f: &mut Formatter<'_>) -> core::fmt::Result {
                /*
                use lexical_core::BUFFER_SIZE;
                let mut buffer = [b'0'; BUFFER_SIZE];

                const FORMAT: u128 = lexical_core::format::STANDARD;
                let mut options = lexical_core::WriteFloatOptions::new();
                //options.set_trim_floats(true);
                options.set_max_significant_digits(core::num::NonZero::new(6));
                let slice = lexical_core::write_with_options::<_, FORMAT>(self.0, &mut buffer, &options);
                core::write!(_f, "{}", core::str::from_utf8(&slice).map_err(|_| core::fmt::Error)?)?;

                 */
                let mut buffer = ryu::Buffer::new();
                let slice = buffer.format(self.0);
                core::write!(_f, "{}", slice)?;

                Ok(())
            }
        }

        #[cfg(feature = "with-defmt")]
        impl Format for Real {
            fn format(&self, _f: defmt::Formatter) {
                defmt::write!(_f, "{}", self.0);
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
            exhausted: bool,
        }

        impl RealInclusiveRange {
            pub fn new(start: Real, end: Real, step_size: Real) -> Self {
                let num_steps = ((end - start) / step_size).ceil();
                RealInclusiveRange {
                    current: start,
                    current_back: start,
                    step_size,
                    start,
                    end,
                    num_steps,
                    exhausted: false,
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
                self.exhausted = false;
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
                if self.step_size.is_defined_positive() {
                    if self.current <= self.end {
                        self.current_back = self.current.clone();
                        self.current += self.step_size.clone();
                        Some(self.current_back.clone())
                    }
                    else {
                        if self.exhausted {
                            None
                        }
                        else {
                            self.exhausted = true;
                            self.current_back = self.current.clone();
                            self.current += self.step_size.clone();
                            Some(self.current_back.clone())
                        }
                    }
                }
                else {
                    None
                }
            }

            /*
            fn size_hint(&self) -> (usize, Option<usize>) {
                let l = self.usize_len();
                (l.clone(), Some(l))
            }
             */

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
