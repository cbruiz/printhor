use core::cmp::Ordering;
use core::ops::*;
use core::fmt::{Debug, Display, Formatter};
use rust_decimal::prelude::*;
use rust_decimal_macros::dec;

#[derive(Copy, Clone, Default, Debug)]
pub struct Real(Decimal);

#[allow(dead_code)]
impl Real {
    #[inline]
    pub(crate) fn new(num: i64, scale: u32) -> Self {
        Real(Decimal::new(num, scale))
    }
    #[inline]
    pub const fn from_fixed(dec: Decimal) -> Self {
        Real(dec)
    }

    #[inline]
    pub fn from_f32(num: f32) -> Self {
        Real(Decimal::from_f32_retain(num).unwrap())
    }
    #[inline]
    pub(crate) fn from_lit(num: i64, scale: u32) -> Self {
        Real(Decimal::new(num, scale))
    }
    #[inline]
    pub fn inner(&self) -> Decimal {
        self.0
    }
    #[inline]
    pub(crate) fn abs(self) -> Self {
        Real(self.0.abs())
    }
    #[inline]
    pub(crate) fn powi(self, x: i32) -> Self {
        Real(self.0.powi(x as i64))
    }
    #[inline]
    pub const fn zero() -> Self {
        Real(dec!(0.0))
    }
    #[inline]
    pub(crate) const fn is_zero(&self) -> bool {
        self.0.is_zero()
    }
    #[inline]
    pub(crate) const fn one() -> Self {
        Real(dec!(1.0))
    }
    #[inline]
    pub(crate) fn round(&self) -> Self {
        Real(self.0.round())
    }
    #[inline]
    pub(crate) fn round_dp(&self, decimals: u32) -> Self {
        Real(self.0.round_dp(decimals))
    }
    /// Round to decimal digits
    #[inline]
    pub fn rdp(&self, digits: u32) -> Self {
        Real(self.0.round_dp(digits))
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
        self.0.to_f64().unwrap()
    }

    #[inline]
    pub fn to_i32(&self) -> Option<i32> {
        self.0.round().to_i32()
    }

    #[inline]
    pub fn to_i64(&self) -> Option<i64> {
        self.0.round().to_i64()
    }

    #[inline]
    pub fn int(&self) -> i64 {
        self.0.trunc().to_i64().unwrap()
    }

    #[inline]
    pub(crate) fn sqrt(self) -> Option<Self> {
        match self.0.sqrt() {
            None => {None}
            Some(v) => {Some(Real(v))}
        }
    }

    #[inline]
    pub(crate) fn cos(self) -> Self {
        Real(self.0.cos())
    }

    #[inline]
    pub(crate) fn ln(self) -> Self {
        Real(self.0.ln())
    }

    #[inline]
    pub(crate) fn exp(self) -> Self {Real(self.0.exp()) }

    #[inline]
    pub(crate) fn sign(self) -> Self {
        let s = self.0.signum();
        if s.is_zero() {Real::one()} else {Real(s)}
    }

    pub(crate) fn min(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
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

    pub(crate) fn max(r1: Option<Real>, r2: Option<Real>) -> Option<Self> {
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
        self.0.cmp(&other.0)
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
impl crate::hwa::defmt::Format for Real {
    fn format(&self, fmt: crate::hwa::defmt::Formatter) {
        crate::hwa::defmt::write!(fmt, "{:?}", self.0.to_f64());
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
/*
impl ExactSizeIterator for RealInclusiveRange {
    fn len(&self) -> usize {
        self.num_steps.0.to_usize().unwrap()
    }
}
*/