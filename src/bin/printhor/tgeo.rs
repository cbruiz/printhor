//! A computing geometry Q&D API to make facilitate vector operations and provide numerically stability (undefs, etc).
//! It implies a cost, of course
use core::ops::Mul;
#[allow(unused)]
use crate::hwa;
#[cfg(not(feature = "native"))]
use crate::alloc::string::ToString;
use crate::math::{Real};
use num_traits::float::FloatCore;
use bitflags::bitflags;
use printhor_hwa_common::StepperChannel;

bitflags! {
    #[derive(PartialEq, Clone, Copy, Eq, Debug)]
    pub struct CoordSel: u8 {
        const X = 0b00000001;
        const Y = 0b00000010;
        const Z = 0b00000100;
        const E = 0b00001000;
        const XYZ = Self::X.bits() | Self::Y.bits() | Self::Z.bits();
        const XYZE = Self::X.bits() | Self::Y.bits() | Self::Z.bits() | Self::E.bits();
    }
}

impl From<StepperChannel> for CoordSel {
    fn from(_value: StepperChannel) -> Self {
        #[allow(unused_mut)]
        let mut coordsel = CoordSel::empty();
        #[cfg(feature = "with-x-axis")]
        coordsel.set(CoordSel::X, _value.contains(StepperChannel::X));
        #[cfg(feature = "with-y-axis")]
        coordsel.set(CoordSel::Y, _value.contains(StepperChannel::Y));
        #[cfg(feature = "with-z-axis")]
        coordsel.set(CoordSel::Z, _value.contains(StepperChannel::Z));
        #[cfg(feature = "with-e-axis")]
        coordsel.set(CoordSel::E, _value.contains(StepperChannel::E));
        coordsel
    }
}


pub trait ArithmeticOps: Copy
    + Clone
    + core::ops::Add<Self, Output=Self>
    + core::ops::Mul<Self, Output=Self>
    + core::cmp::PartialEq<Self>
    + core::cmp::PartialOrd<Self>
{
    fn zero() -> Self;
    fn one() -> Self;
    fn is_zero(&self) -> bool;
    fn is_defined_positive(&self) -> bool;
    fn abs(&self) -> Self;
}

pub trait RealOps
{
    fn pow(&self, power: i32) -> Self;
    fn sqrt(&self) -> Option<Self> where Self: Sized;
    fn rdp(&self, digits: u32) -> Self;
    fn ceil(&self) -> Self;
    fn floor(&self) -> Self;

}

#[derive(Copy, Clone, PartialEq)]
pub struct TVector<T>
    where T: ArithmeticOps
{
    pub x: Option<T>,
    pub y: Option<T>,
    pub z: Option<T>,
    pub e: Option<T>,
}

#[allow(unused)]
impl<T> TVector<T>
where T: ArithmeticOps
{
    #[inline]
    pub const fn new() -> Self {
        Self::nan()
    }

    #[inline]
    pub const fn from_coords(x: Option<T>, y: Option<T>, z: Option<T>, e: Option<T>) -> Self {
        Self {
            x,y,z,e
        }
    }

    #[inline]
    pub fn map_coords<U, F>(&self, f: F) -> TVector<U>
    where F: Fn(T) -> Option<U>, U: ArithmeticOps
    {
        TVector {
            x: self.x.and_then(|v| f(v)),
            y: self.y.and_then(|v| f(v)),
            z: self.z.and_then(|v| f(v)),
            e: self.e.and_then(|v| f(v)),
        }
    }

    #[inline]
    pub fn map_all<U, F>(&self, f: F) -> TVector<U>
        where F: Fn(Option<T>) -> Option<U>, U: ArithmeticOps
    {
        TVector {
            x: f(self.x),
            y: f(self.y),
            z: f(self.z),
            e: f(self.e),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn apply_coords<F>(&self, mut f: F)
        where F: FnMut((CoordSel, &T)) -> ()
    {
        if let Some(v) = &self.x {
            f((CoordSel::X, v))
        }
        if let Some(v) = &self.y {
            f((CoordSel::Y, v))
        }
        if let Some(v) = &self.z {
            f((CoordSel::Z, v))
        }
        if let Some(v) = &self.e {
            f((CoordSel::E, v))
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn apply(&self, rhs: &TVector<T>) -> TVector<T> {
        TVector {
            x: match rhs.x {
                None => self.x,
                Some(lv) => Some(lv),
            },
            y: match rhs.y {
                None => self.y,
                Some(lv) => Some(lv),
            },
            z: match rhs.z {
                None => self.z,
                Some(lv) => Some(lv),
            },
            e: match rhs.e {
                None => self.e,
                Some(lv) => Some(lv),
            },
        }
    }

    #[inline]
    pub fn map_coord<F>(&self, coord_idx: CoordSel, f: F) -> TVector<T>
        where F: Fn(T, CoordSel) -> Option<T>, T: ArithmeticOps
    {
        TVector {
            x: if coord_idx.contains(CoordSel::X) { self.x.and_then(|v| f(v, CoordSel::X)) } else { self.x },
            y: if coord_idx.contains(CoordSel::Y) { self.y.and_then(|v| f(v, CoordSel::Y)) } else { self.y },
            z: if coord_idx.contains(CoordSel::Z) { self.z.and_then(|v| f(v, CoordSel::Z)) } else { self.z },
            e: if coord_idx.contains(CoordSel::E) { self.e.and_then(|v| f(v, CoordSel::E)) } else { self.e },
        }
    }

    #[inline]
    #[allow(unused)]
    pub fn with_coord(&self, coord_idx: CoordSel, val: Option<T>) -> Self {
        Self{
            x: if coord_idx.contains(CoordSel::X) { val } else { self.x },
            y: if coord_idx.contains(CoordSel::Y) { val } else { self.y },
            z: if coord_idx.contains(CoordSel::Z) { val } else { self.z },
            e: if coord_idx.contains(CoordSel::E) { val } else { self.e },
        }
    }

    #[inline]
    #[allow(unused)]
    pub fn set_coord(&mut self, coord_idx: CoordSel, val: Option<T>) -> &Self {
        if coord_idx.contains(CoordSel::X) { self.x = val }
        if coord_idx.contains(CoordSel::Y) { self.y = val }
        if coord_idx.contains(CoordSel::Z) { self.z = val }
        if coord_idx.contains(CoordSel::E) { self.e = val }
        self
    }

    #[inline]
    #[allow(unused)]
    pub fn increment(&mut self, coord_idx: CoordSel, val: T) -> &Self {
        if coord_idx.contains(CoordSel::X) { self.x = self.x.and_then(|v| Some(v + val)) }
        if coord_idx.contains(CoordSel::Y) { self.y = self.y.and_then(|v| Some(v + val)) }
        if coord_idx.contains(CoordSel::Z) { self.z = self.z.and_then(|v| Some(v + val)) }
        if coord_idx.contains(CoordSel::E) { self.e = self.e.and_then(|v| Some(v + val)) }
        self
    }

    #[inline]
    #[allow(unused)]
    pub fn decrement_if_positive(&mut self, coord_idx: CoordSel) -> bool
    where T: ArithmeticOps + core::ops::Sub<Output=T>
    {
        let mut changed = false;
        if coord_idx.contains(CoordSel::X) {
            self.x = self.x.and_then(|v|
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                }
                else {
                    self.x
                }
            )
        }
        else if coord_idx.contains(CoordSel::Y) {
            self.y = self.y.and_then(|v|
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                }
                else {
                    self.y
                }
            )
        }
        else if coord_idx.contains(CoordSel::Z) {
            self.z = self.z.and_then(|v|
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                }
                else {
                    self.z
                }
            )
        }
        else if coord_idx.contains(CoordSel::E) {
            self.e = self.e.and_then(|v|
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                }
                else {
                    self.e
                }
            )
        }
        changed
    }

    #[inline]
    #[allow(unused)]
    pub fn assign(&mut self, coord_idx: CoordSel, other: &Self) -> &Self {
        if coord_idx.contains(CoordSel::X) { self.x = other.x }
        if coord_idx.contains(CoordSel::Y) { self.y = other.y }
        if coord_idx.contains(CoordSel::Z) { self.z = other.z }
        if coord_idx.contains(CoordSel::E) { self.e = other.e }
        self
    }

    #[inline]
    #[allow(unused)]
    pub fn assign_if_set(&mut self, coord_idx: CoordSel, other: &Self) -> &Self {
        if coord_idx.contains(CoordSel::X) && other.x.is_some() { self.x = other.x }
        if coord_idx.contains(CoordSel::Y) && other.y.is_some() { self.y = other.y }
        if coord_idx.contains(CoordSel::Z) && other.z.is_some() { self.z = other.z }
        if coord_idx.contains(CoordSel::E) && other.e.is_some() { self.e = other.e }
        self
    }

    #[inline]
    #[allow(unused)]
    pub fn with_coord_if_set(&self, coord_idx: CoordSel, val: Option<T>) -> Self {
        Self {
            x: if coord_idx.contains(CoordSel::X) && self.x.is_some() { val } else { self.x },
            y: if coord_idx.contains(CoordSel::Y) && self.y.is_some() { val } else { self.y },
            z: if coord_idx.contains(CoordSel::Z) && self.z.is_some() { val } else { self.z },
            e: if coord_idx.contains(CoordSel::E) && self.e.is_some() { val } else { self.e },
        }
    }

    pub fn clamp_coord(&self, coord_idx: CoordSel, upper_bound: T) -> Self
    where T: core::cmp::PartialOrd<T>
    {
        Self {
            x: if coord_idx.contains(CoordSel::X) { self.x.and_then(|v| if v > upper_bound {Some(v)} else {Some(upper_bound)})} else {self.x},
            y: if coord_idx.contains(CoordSel::Y) { self.y.and_then(|v| if v > upper_bound {Some(v)} else {Some(upper_bound)})} else {self.y},
            z: if coord_idx.contains(CoordSel::Z) { self.z.and_then(|v| if v > upper_bound {Some(v)} else {Some(upper_bound)})} else {self.z},
            e: if coord_idx.contains(CoordSel::E) { self.e.and_then(|v| if v > upper_bound {Some(v)} else {Some(upper_bound)})} else {self.e},
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn clamp(&self, rhs: TVector<T>) -> TVector<T> {
        TVector {
            x: match self.x {
                None => None,
                Some(lv) => match rhs.x {
                    None => Some(lv),
                    Some(rv) => match lv > rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
            y: match self.y {
                None => None,
                Some(lv) => match rhs.y {
                    None => Some(lv),
                    Some(rv) => match lv > rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
            z: match self.z {
                None => None,
                Some(lv) => match rhs.z {
                    None => Some(lv),
                    Some(rv) => match lv > rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
            e: match self.e {
                None => None,
                Some(lv) => match rhs.e {
                    None => Some(lv),
                    Some(rv) => match lv > rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn clamp_min(&self, rhs: TVector<T>) -> TVector<T> {
        TVector {
            x: match self.x {
                None => None,
                Some(lv) => match rhs.x {
                    None => Some(lv),
                    Some(rv) => match lv < rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
            y: match self.y {
                None => None,
                Some(lv) => match rhs.y {
                    None => Some(lv),
                    Some(rv) => match lv < rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
            z: match self.z {
                None => None,
                Some(lv) => match rhs.z {
                    None => Some(lv),
                    Some(rv) => match lv < rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
            e: match self.e {
                None => None,
                Some(lv) => match rhs.e {
                    None => Some(lv),
                    Some(rv) => match lv < rv {
                        true => Some(rv),
                        false => Some(lv),
                    }
                }
            },
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn vmax(&self) -> Option<T> {
        let mut m: Option<T> = None;
        if let Some(x) = self.x {
            m = Some(x);
        }
        if let Some(y) = self.y {
            if let Some(mr) = m {
                if mr.lt(&y) {
                    m = Some(y);
                }
            }
            else {
                m = Some(y);
            }
        }
        if let Some(z) = self.z {
            if let Some(mr) = m {
                if mr.lt(&z) {
                    m = Some(z);
                }
            }
            else {
                m = Some(z);
            }
        }
        if let Some(e) = self.e {
            if let Some(mr) = m {
                if mr.lt(&e) {
                    m = Some(e);
                }
            }
            else {
                m = Some(e);
            }
        }
        m
    }

    #[allow(unused)]
    #[inline]
    pub fn vmin(&self) -> Option<T> {
        let mut m: Option<T> = None;
        if let Some(x) = self.x {
            m = Some(x);
        }
        if let Some(y) = self.y {
            if let Some(mr) = m {
                if mr.gt(&y) {
                    m = Some(y);
                }
            }
            else {
                m = Some(y);
            }
        }
        if let Some(z) = self.z {
            if let Some(mr) = m {
                if mr.gt(&z) {
                    m = Some(z);
                }
            }
            else {
                m = Some(z);
            }
        }
        if let Some(e) = self.e {
            if let Some(mr) = m {
                if mr.gt(&e) {
                    m = Some(e);
                }
            }
            else {
                m = Some(e);
            }
        }
        m
    }

    /// Evaluates when a vector has all coordinates bounded by other
    /// Meaning that every x, y, z ... of Self is lower than other's
    #[allow(unused)]
    #[inline]
    pub fn bounded_by(&self, rhs: &TVector<T>) -> bool {
        let mut matching_point = true;
        match self.x {
            None => {},
            Some(lv) => match rhs.x {
                None => if !lv.is_zero() {return false},
                Some(rv) => {
                    if lv > rv { // coordinate exceeding
                        return false;
                    }
                    if lv < rv { // coordinate preceeding
                        matching_point = false;
                    }
                }
            }
        }
        match self.y {
            None => {},
            Some(lv) => match rhs.y {
                None => if !lv.is_zero() {return false},
                Some(rv) => {
                    if lv > rv { // coordinate exceeding
                        return false;
                    }
                    if lv < rv { // coordinate preceeding
                        matching_point = false;
                    }
                }
            }
        }
        match self.z {
            None => {},
            Some(lv) => match rhs.z {
                None => if !lv.is_zero() {return false},
                Some(rv) => {
                    if lv > rv { // coordinate exceeding
                        return false;
                    }
                    if lv < rv { // coordinate preceeding
                        matching_point = false;
                    }
                }
            }
        }
        match self.e {
            None => {},
            Some(lv) => match rhs.e {
                None => if !lv.is_zero() {return false},
                Some(rv) => {
                    if lv > rv { // coordinate exceeding
                        return false;
                    }
                    if lv < rv { // coordinate preceeding
                        matching_point = false;
                    }
                }
            }
        }
        return !matching_point;
    }

    pub fn is_nan_or_zero(&self) -> bool {
        if let Some(v) = self.x {
            if !v.is_zero() { return false }
        }
        if let Some(v) = self.y {
            if !v.is_zero() { return false }
        }
        if let Some(v) = self.z {
            if !v.is_zero() { return false }
        }
        if let Some(v) = self.e {
            if !v.is_zero() { return false }
        }
        return true;
    }

    #[inline]
    pub const fn nan() -> Self {
        Self {
            x: None,
            y: None,
            z: None,
            e: None,
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn zero() -> Self {
        Self {
            x: Some(T::zero()),
            y: Some(T::zero()),
            z: Some(T::zero()),
            e: Some(T::zero()),
        }
    }

    #[inline]
    pub fn one() -> Self {
        Self {
            x: Some(T::one()),
            y: Some(T::one()),
            z: Some(T::one()),
            e: Some(T::one()),
        }
    }

    pub fn map_nan(&self, value: &T) -> Self {
        Self {
            x: self.x.map_or_else(|| Some(*value), |cv| Some(cv)),
            y: self.y.map_or_else(|| Some(*value), |cv| Some(cv)),
            z: self.z.map_or_else(|| Some(*value), |cv| Some(cv)),
            e: self.e.map_or_else(|| Some(*value), |cv| Some(cv)),
        }
    }

    pub fn map_val(&self, value: &T) -> Self {
        Self {
            x: self.x.and(Some(*value)),
            y: self.y.and(Some(*value)),
            z: self.z.and(Some(*value)),
            e: self.e.and(Some(*value)),
        }
    }

    pub fn sum(&self) -> T
        where T: core::ops::AddAssign<T>
    {
        let mut acc_sum = T::zero();
        acc_sum.add_assign(self.x.unwrap_or(T::zero()));
        acc_sum.add_assign(self.y.unwrap_or(T::zero()));
        acc_sum.add_assign(self.z.unwrap_or(T::zero()));
        acc_sum.add_assign(self.e.unwrap_or(T::zero()));
        acc_sum
    }

    pub fn abs(&self) -> Self
    {
        Self {
            x: self.x.and_then(|v| Some(v.abs())),
            y: self.y.and_then(|v| Some(v.abs())),
            z: self.z.and_then(|v| Some(v.abs())),
            e: self.e.and_then(|v| Some(v.abs())),
        }
    }

}

impl<T> TVector<T>
    where T: ArithmeticOps + RealOps + core::ops::AddAssign,
          TVector<T>: core::ops::Div<T, Output = TVector<T>>
          + core::ops::Div<TVector<T>, Output = TVector<T>>

{
    pub fn pow(&self, power: i32) -> Self {
        Self {
            x: self.x.map_or_else(|| None, |v| Some(v.pow(power))),
            y: self.y.map_or_else(|| None, |v| Some(v.pow(power))),
            z: self.z.map_or_else(|| None, |v| Some(v.pow(power))),
            e: self.e.map_or_else(|| None, |v| Some(v.pow(power))),
        }
    }

    #[allow(unused)]
    pub fn sqrt(&self) -> Self {
        Self {
            x: self.x.map_or_else(|| None, |v| v.sqrt()),
            y: self.y.map_or_else(|| None, |v| v.sqrt()),
            z: self.z.map_or_else(|| None, |v| v.sqrt()),
            e: self.e.map_or_else(|| None, |v| v.sqrt()),
        }
    }

    pub fn norm2(&self) -> Option<T>
    where T: RealOps + core::ops::AddAssign<T>
    {
        let n = self.mul(*self).sum();
        let r = n.sqrt();
        return r;
    }

    pub fn scalar_product(&self, rhs: TVector<T>) -> T
        where T: RealOps + core::ops::AddAssign<T>, TVector<T>: core::ops::Mul<TVector<T>, Output=TVector<T>>
    {
        ((*self) * rhs).sum()
    }

    /// Computes the orthogonal projection of [other] over this
    /// proj(self, other) = \frac{self \cdot other}{|self|^(2)}
    pub fn orthogonal_projection(&self, other: TVector<T>) -> T
    where T: RealOps + core::ops::AddAssign<T> + core::ops::Div<Output = T>, TVector<T>: core::ops::Mul<TVector<T>, Output=TVector<T>>
    {
        (*self).scalar_product(other) / (self.pow(2).sum().abs())
    }

    #[allow(unused)]
    pub fn unit(&self) -> Self
    {
        match self.norm2() {
            None => Self::nan(),
            Some(norm) => match norm.is_zero() {
                true => Self::nan(),
                false => {
                    let t1 = *self;
                    let t2 = norm;
                    t1 / t2
                }
            }
        }
    }
    /***
    custom behavior
     */
    #[allow(unused)]
    pub fn decompose_normal(&self) -> (Self, T)
    where TVector<T>: core::ops::Div<T, Output=TVector<T>>, T: ArithmeticOps + RealOps
    {
        match self.norm2() {
            None => (Self::nan(), T::zero()),
            Some(norm) => match norm.is_zero() {
                true => (Self::nan(), T::zero()),
                false => {
                    ((*self) / norm, norm)
                }
            }
        }
    }

    #[allow(unused)]
    pub fn rdp(&self, digits: u32) -> TVector<T> {
        Self {
            x: self.x.map_or_else(|| None, |v| Some(v.rdp(digits))),
            y: self.y.map_or_else(|| None, |v| Some(v.rdp(digits))),
            z: self.z.map_or_else(|| None, |v| Some(v.rdp(digits))),
            e: self.e.map_or_else(|| None, |v| Some(v.rdp(digits))),
        }
    }

    #[allow(unused)]
    pub fn floor(&self) -> TVector<T>
    where T: RealOps
    {
        Self {
            x: self.x.map_or_else(|| None, |v| Some(v.floor())),
            y: self.y.map_or_else(|| None, |v| Some(v.floor())),
            z: self.z.map_or_else(|| None, |v| Some(v.floor())),
            e: self.e.map_or_else(|| None, |v| Some(v.floor())),
        }
    }

    #[allow(unused)]
    pub fn ceil(&self) -> TVector<T>
        where T: RealOps
    {
        Self {
            x: self.x.map_or_else(|| None, |v| Some(v.ceil())),
            y: self.y.map_or_else(|| None, |v| Some(v.ceil())),
            z: self.z.map_or_else(|| None, |v| Some(v.ceil())),
            e: self.e.map_or_else(|| None, |v| Some(v.ceil())),
        }
    }

    #[allow(unused)]
    pub fn round(&self) -> TVector<T> {
        Self {
            x: self.x.map_or_else(|| None, |v| Some(v.rdp(0))),
            y: self.y.map_or_else(|| None, |v| Some(v.rdp(0))),
            z: self.z.map_or_else(|| None, |v| Some(v.rdp(0))),
            e: self.e.map_or_else(|| None, |v| Some(v.rdp(0))),
        }
    }


}

impl<T> Default for TVector<T>
where T: ArithmeticOps
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<T> core::fmt::Display for TVector<T>
where T: ArithmeticOps + ToString
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut spacing = false;
        if let Some(v) = &self.x {
            core::write!(f, "X {}", v.to_string())?;
            spacing = true;
        }
        if let Some(v) = &self.y {
            core::write!(f, "{}Y {}", if spacing {" "} else {""}, v.to_string())?;
            spacing = true;
        }
        if let Some(v) = &self.z {
            core::write!(f, "{}Z {}", if spacing {" "} else {""}, v.to_string())?;
            spacing = true;
        }
        if let Some(v) = &self.e {
            core::write!(f, "{}E {}", if spacing {" "} else {""}, v.to_string())?;
        }
        Ok(())
    }
}

#[cfg(feature = "with-defmt")]
impl<T> hwa::defmt::Format for TVector<T>
where T: ArithmeticOps + ToString
{
    fn format(&self, fmt: hwa::defmt::Formatter) {

        let mut spacing = false;
        if let Some(v) = &self.x {
            hwa::defmt::write!(fmt, "X {}", v.to_string().as_str());
            spacing = true;
        }
        if let Some(v) = &self.y {
            hwa::defmt::write!(fmt, "{}Y {}", if spacing {" "} else {""}, v.to_string().as_str());
            spacing = true;
        }
        if let Some(v) = &self.z {
            hwa::defmt::write!(fmt, "{}Z {}", if spacing {" "} else {""}, v.to_string().as_str());
            spacing = true;
        }
        if let Some(v) = &self.e {
            hwa::defmt::write!(fmt, "{}E {}", if spacing {" "} else {""}, v.to_string().as_str());
        }
    }
}


impl<T> core::ops::Add for TVector<T>
where T: ArithmeticOps + core::ops::Add<Output=T>
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::Output {
            x: self.x.and_then(|x0| rhs.x.and_then(|x1| Some(x0 + x1))),
            y: self.y.and_then(|y0| rhs.y.and_then(|y1| Some(y0 + y1))),
            z: self.z.and_then(|z0| rhs.z.and_then(|z1| Some(z0 + z1))),
            e: self.e.and_then(|e0| rhs.e.and_then(|e1| Some(e0 + e1))),
        }

    }
}

impl<T> core::ops::AddAssign for TVector<T>
    where T: ArithmeticOps + core::ops::AddAssign
{

    fn add_assign(&mut self, rhs: Self) {
        self.x.as_mut().map(|x0| rhs.x.map(|x1| (*x0) += x1));
        self.y.as_mut().map(|y0| rhs.y.map(|y1| (*y0) += y1));
        self.z.as_mut().map(|z0| rhs.z.map(|z1| (*z0) += z1));
        self.e.as_mut().map(|e0| rhs.e.map(|e1| (*e0) += e1));
    }
}

impl<T> core::ops::Sub for TVector<T>
    where T: ArithmeticOps + core::ops::Sub<Output=T>
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::Output {
            x: self.x.and_then(|x0| rhs.x.and_then(|x1| Some(x0 - x1))),
            y: self.y.and_then(|y0| rhs.y.and_then(|y1| Some(y0 - y1))),
            z: self.z.and_then(|z0| rhs.z.and_then(|z1| Some(z0 - z1))),
            e: self.e.and_then(|e0| rhs.e.and_then(|e1| Some(e0 - e1))),
        }
    }
}

impl<T> core::ops::SubAssign for TVector<T>
    where T: ArithmeticOps + core::ops::SubAssign
{

    fn sub_assign(&mut self, rhs: Self) {
        self.x.as_mut().map(|x0| rhs.x.map(|x1| (*x0) -= x1));
        self.y.as_mut().map(|y0| rhs.y.map(|y1| (*y0) -= y1));
        self.z.as_mut().map(|z0| rhs.z.map(|z1| (*z0) -= z1));
        self.e.as_mut().map(|e0| rhs.e.map(|e1| (*e0) -= e1));
    }
}

impl<T> core::ops::Mul<TVector<T>> for TVector<T>
    where T: ArithmeticOps + core::ops::Mul<T, Output=T>
{
    type Output = Self;

    fn mul(self, rhs: TVector<T>) -> Self::Output {
        Self::Output {
            x: self.x.and_then(|x0| rhs.x.and_then(|x1| Some(x0 * x1))),
            y: self.y.and_then(|y0| rhs.y.and_then(|y1| Some(y0 * y1))),
            z: self.z.and_then(|z0| rhs.z.and_then(|z1| Some(z0 * z1))),
            e: self.e.and_then(|e0| rhs.e.and_then(|e1| Some(e0 * e1))),
        }

    }
}

impl<T> core::ops::Mul<T> for TVector<T>
    where T: ArithmeticOps + core::ops::Mul<T, Output=T>
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self::Output {
            x: self.x.and_then(|v| Some(v * rhs)),
            y: self.y.and_then(|v| Some(v * rhs)),
            z: self.z.and_then(|v| Some(v * rhs)),
            e: self.e.and_then(|v| Some(v * rhs)),
        }

    }
}

impl<T> core::ops::Div<TVector<T>> for TVector<T>
    where T: ArithmeticOps + RealOps + core::ops::Div<T, Output=T>
{
    type Output = TVector<T>;

    fn div(self, rhs: TVector<T>) -> Self::Output
    {
        Self::Output {
            x: self.x.map_or_else(|| None, |v| {
                rhs.x.map_or_else(|| None, |divisor| {
                    if !divisor.is_zero() { Some(v.div(divisor)) }
                    else { None }
                })
            }),
            y: self.y.map_or_else(|| None, |v| {
                rhs.y.map_or_else(|| None, |divisor| {
                    if !divisor.is_zero() { Some(v.div(divisor)) }
                    else { None }
                })
            }),
            z: self.z.map_or_else(|| None, |v| {
                rhs.z.map_or_else(|| None, |divisor| {
                    if !divisor.is_zero() { Some(v.div(divisor)) }
                    else { None }
                })
            }),
            e: self.e.map_or_else(|| None, |v| {
                rhs.e.map_or_else(|| None, |divisor| {
                    if !divisor.is_zero() { Some(v.div(divisor)) }
                    else { None }
                })
            }),
        }
    }
}

impl<T> core::ops::Div<T> for TVector<T>
    where T: ArithmeticOps + RealOps + core::ops::Div<T, Output=T>
{
    type Output = TVector<T>;

    fn div(self, rhs: T) -> Self::Output
    {
        if rhs.is_zero() {
            Self::Output::nan()
        }
        else {
            Self::Output {
                x: self.x.and_then(|x0| Some(x0 / rhs)),
                y: self.y.and_then(|y0| Some(y0 / rhs)),
                z: self.z.and_then(|z0| Some(z0 / rhs)),
                e: self.e.and_then(|e0| Some(e0 / rhs)),
            }
        }
    }
}

//////////////

impl ArithmeticOps for i32 {
    #[inline]
    fn zero() -> Self {
        0
    }
    #[inline]
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
    where Self: core::cmp::PartialEq
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
        where Self: core::cmp::PartialOrd
    {
        self.gt(&0)
    }

    fn abs(&self) -> Self {
        i32::abs(*self)
    }
}

impl ArithmeticOps for u32 {
    #[inline]
    fn zero() -> Self {
        0
    }
    #[inline]
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.gt(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for u64 {
    #[inline]
    fn zero() -> Self {
        0
    }
    #[inline]
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.gt(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for u16 {
    #[inline]
    fn zero() -> Self {
        0
    }
    #[inline]
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.gt(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for u8 {
    #[inline]
    fn zero() -> Self {
        0
    }
    #[inline]
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.gt(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for f32 {
    #[inline]
    fn zero() -> Self {
        0.0f32
    }
    #[inline]
    fn one() -> Self {
        1.0f32
    }

    fn is_zero(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.eq(&0.0f32)
    }

    fn is_defined_positive(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.gt(&0.0f32)
    }

    fn abs(&self) -> Self {
        <f32 as FloatCore>::abs(*self)
    }
}

impl RealOps for f32 {

    fn pow(&self, p: i32) -> Self {
        self.powi(p)
    }

    fn sqrt(&self) -> Option<Self> where Self: Sized {
        if !self.is_sign_negative() {
            Some(micromath::F32(*self).sqrt().0)
            //f32::sqrt(self)
        }
        else {
            None
        }

    }

    fn rdp(&self, digits: u32) -> Self {
        let dd =  10.0f32.powi(digits as i32);
        (self * dd).round() * dd
    }
    fn floor(&self) -> Self {
        <f32 as FloatCore>::floor(*self)
    }

    fn ceil(&self) -> Self {
        <f32 as FloatCore>::ceil(*self)
    }

}


impl ArithmeticOps for Real {
    #[inline]
    fn zero() -> Self {
        Real::zero()
    }

    #[inline]
    fn one() -> Self {
        Real::one()
    }

    fn is_zero(&self) -> bool {
        Real::is_zero(self)
    }

    fn is_defined_positive(&self) -> bool
        where Self: core::cmp::PartialEq
    {
        self.gt(&Real::zero())
    }

    fn abs(&self) -> Self {
        Real::abs(*self)
    }
}

impl RealOps for Real {

    fn pow(&self, p: i32) -> Real {
        self.powi(p)
    }

    fn sqrt(&self) -> Option<Self> where Self: Sized {
        Real::sqrt(*self)
    }

    fn rdp(&self, digits: u32) -> Self {
        Real::round_dp(self, digits)
    }

    fn ceil(&self) -> Self {
        Real::ceil(self)
    }

    fn floor(&self) -> Self {
        Real::floor(self)
    }
}

#[allow(unused)]
pub fn test() {
    let pos: TVector<i32> = TVector::new();
    let p1: TVector<i32> = TVector::one();

    let p0 = pos.map_nan(&0);

    let r = p0 + p1;
    crate::hwa::info!("{}", r);
}


