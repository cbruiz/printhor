//! FIXME [Obsolete]
//! A computing geometry Q&D API to make facilitate vector operations and provide numerically stability (undefs, etc).
//! It implies a cost, of course
/*
use alloc::string::ToString;
use core::fmt::{Debug, Display, Formatter};
use core::ops::{Sub, SubAssign, Div};
use core::ops::{Add, Mul};

use crate::math::Real;
use crate::tgeo::CoordSel;

#[derive(Default, Copy, Clone)]
pub struct Vector2 {
    pub x: Option<Real>,
    pub y: Option<Real>,
}

#[allow(unused)]
#[derive(Default, Copy, Clone)]
pub struct Rect {
    pub p_0: Vector2,
    pub v_dir: Vector2,
}

impl Rect {
    #[allow(unused)]
    pub fn new_from_points(p_0: Vector2, p_1: Vector2) -> Self {
        let vdir = p_1 - p_0;
        Self{
            p_0,
            v_dir: vdir.unit(),
        }
    }

    #[allow(unused)]
    pub fn intersection(&self, rhs: Rect) -> Option<Vector2> {

        match (self.v_dir * rhs.v_dir.invert()).diff() {
            None => None,
            Some(d) => {
                if d.is_zero() {
                    None
                }
                else {
                    let pre = (self.p_0 * (self.p_0 - self.v_dir).invert()).diff();
                    let post = (rhs.p_0 * (rhs.p_0 - rhs.v_dir).invert()).diff();
                    Some(( ( rhs.v_dir * pre ) - (self.v_dir * post)) / Some(d))
                }
            }
        }
    }
}

impl Vector2 {
    #[allow(unused)]
    pub const fn new() -> Self {
        Self {
            x: None,
            y: None,
        }
    }

    #[allow(unused)]
    pub const fn from_reals(x: Real, y: Real) -> Self {
        Self {
            x: Some(x),
            y: Some(y),
        }
    }

    #[allow(unused)]
    pub const fn from_optional_reals(x: Option<Real>, y: Option<Real>) -> Self {
        Self {
            x,
            y,
        }
    }

    pub fn unit(&self) -> Self {
        self.div(self.norm2())
    }

    pub fn norm2(&self) -> Option<Real> {
        self.x.and_then(|x|
            self.y.and_then(|y|
                ((x*x) + (y*y)).sqrt().and_then(|n| Some(n)))
        )
    }

    #[allow(unused)]
    pub fn rdp(&self, digits: u32) -> Self {
        Self {
            x: self.x.and_then(|v| Some(v.rdp(digits))),
            y: self.y.and_then(|v| Some(v.rdp(digits))),
        }
    }

    pub fn invert(&self) -> Self {
        Self {
            x: self.y,
            y: self.x,
        }
    }

    pub fn diff(&self) -> Option<Real> {
        self.x.and_then(|x|
            self.y.and_then(|y|
                    Some(y - x)))
    }

    #[allow(unused)]
    pub fn slope(&self) -> Option<Real> {
        self.y.and_then(|y|
            self.x.and_then(|x|
                if x.is_zero() {
                    Some(Real::zero())
                }
                else {
                    Some(y.div(x))
                }
            )
        )

    }

    #[allow(unused)]
    pub fn x_lt(&self, rhs: &Real) -> bool {
        self.x.and_then(|x| Some(x.lt(rhs))).unwrap_or(false)

    }

    #[allow(unused)]
    pub fn x_gt(&self, rhs: &Real) -> bool {
        self.x.and_then(|x| Some(x.gt(rhs))).unwrap_or(false)

    }
}

impl Add<Vector2> for Vector2 {
    type Output = Self;

    fn add(self, rhs: Vector2) -> Self {
        Self {
            x: self.x.and_then(|x0| rhs.x.and_then(|x1| Some(x0 + x1))),
            y: self.y.and_then(|y0| rhs.y.and_then(|y1| Some(y0 + y1))),
        }
    }
}

impl Sub<Vector2> for Vector2 {
    type Output = Self;

    fn sub(self, rhs: Vector2) -> Self {
        Self {
            x: self.x.and_then(|x0| rhs.x.and_then(|x1| Some(x0 - x1))),
            y: self.y.and_then(|y0| rhs.y.and_then(|y1| Some(y0 - y1))),
        }
    }
}

impl Mul<Vector2> for Vector2 {
    type Output = Self;

    fn mul(self, rhs: Vector2) -> Self::Output {
        Self {
            x: self.x.and_then(|x0| rhs.x.and_then(|x1| Some(x0 * x1))),
            y: self.y.and_then(|y0| rhs.y.and_then(|y1| Some(y0 * y1))),
        }
    }
}

impl Mul<Real> for Vector2 {
    type Output = Self;

    fn mul(self, rhs: Real) -> Self::Output {
        Self {
            x: self.x.and_then(|x0| Some(x0 * rhs.clone())),
            y: self.y.and_then(|y0| Some(y0 * rhs)),
        }
    }
}

impl Mul<Option<Real>> for Vector2 {
    type Output = Self;

    fn mul(self, rhs: Option<Real>) -> Self::Output {
        Self {
            x: self.x.and_then(|x| rhs.and_then(|m| Some(x * m))),
            y: self.y.and_then(|y| rhs.and_then(|m| Some(y * m))),
        }
    }
}

impl Div<Option<Real>> for Vector2 {
    type Output = Self;

    fn div(self, rhs: Option<Real>) -> Self::Output {
        let x = self.x.and_then(|x| rhs.and_then(|d| {
            if d.is_zero() {
                None
            }
            else {
                Some(x.div(d))
            }
        }));
        let y = self.y.and_then(|y| rhs.and_then(|d| {
            if d.is_zero() {
                None
            }
            else {
                Some(y.div(d))
            }
        }));
        Self {
            x,
            y,
        }
    }
}

#[cfg(feature = "native")]
impl Display for Vector2 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f,
               "X {} Y {}",
               self.x.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
               self.y.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
        )
    }
}

///////////////////////////////

#[derive(Default, Copy, Clone)]
pub struct Vector {
    pub x: Option<Real>,
    pub y: Option<Real>,
    pub z: Option<Real>,
    pub e: Option<Real>,
}

impl Vector {

    #[allow(unused)]
    #[inline]
    pub const fn new(x: Option<Real>, y: Option<Real>, z: Option<Real>, e: Option<Real>) -> Self {
        Vector{x, y, z, e}
    }

    #[allow(unused)]
    #[inline]
    pub fn assign(&mut self, other: &Self) -> () {
        *self = *other;
    }



    #[allow(unused)]
    #[inline]
    pub const fn zero() -> Self {
        Vector{x: Some(Real::zero()), y: Some(Real::zero()), z: Some(Real::zero()), e: Some(Real::zero())}
    }

    #[allow(unused)]
    #[inline]
    pub const fn one() -> Self {
        Vector{x: Some(Real::one()), y: Some(Real::one()), z: Some(Real::one()), e: Some(Real::one())}
    }


    //#[inline]
    //pub fn from(v: (f64, f64, f64, f64)) -> Self {
    //    Vector{x: Real::from_num(v.0), y: Real::from_num(v.1), z: Real::from_num(v.2), e: Real::from_num(v.3)}
    //}

    //#[inline]
    //pub fn from_reals(v: (Real, Real, Real, Real)) -> Self {
    //    Vector{x: v.0, y: v.1, z: v.2, e: v.3}
    //}

    #[allow(unused)]
    #[inline]
    pub fn pow<T: Into<i32>>(&self, power: T) -> Self {
        let p = power.into();
        Vector {
            x: self.x.map_or_else(|| None, |v| Some(v.powi(p.clone()))),
            y: self.y.map_or_else(|| None, |v| Some(v.powi(p.clone()))),
            z: self.z.map_or_else(|| None, |v| Some(v.powi(p.clone()))),
            e: self.e.map_or_else(|| None, |v| Some(v.powi(p as i32)))
        }
    }

    #[allow(unused)]
    #[inline]
    pub const fn default() -> Self {
        Vector{x: None, y: None, z: None, e: None}
    }

    #[inline]
    #[allow(unused)]
    pub fn with(&self, coord_idx: CoordSel, val: Option<Real>) -> Self {
        Vector{
            x: if coord_idx.contains(CoordSel::X) { val.clone() } else { self.x },
            y: if coord_idx.contains(CoordSel::Y) { val.clone() } else { self.y },
            z: if coord_idx.contains(CoordSel::Z) { val.clone() } else { self.z },
            e: if coord_idx.contains(CoordSel::E) { val.clone() } else { self.e },
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn clamp(&self, coord_idx: CoordSel, val: &Real) -> Self {
        Vector{
            x: if coord_idx.contains(CoordSel::X) { self.x.and_then(|v| if v.gt(val) {Some(v)} else {Some(*val)})} else {self.x},
            y: if coord_idx.contains(CoordSel::Y) { self.y.and_then(|v| if v.gt(val) {Some(v)} else {Some(*val)})} else {self.y},
            z: if coord_idx.contains(CoordSel::Z) { self.z.and_then(|v| if v.gt(val) {Some(v)} else {Some(*val)})} else {self.z},
            e: if coord_idx.contains(CoordSel::E) { self.e.and_then(|v| if v.gt(val) {Some(v)} else {Some(*val)})} else {self.e},
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn floor(&self) -> Self {
        Vector {
            x: self.x.map_or_else(|| None, |v| Some(v.floor())),
            y: self.y.map_or_else(|| None, |v| Some(v.floor())),
            z: self.z.map_or_else(|| None, |v| Some(v.floor())),
            e: self.e.map_or_else(|| None, |v| Some(v.floor())),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn clampv(&self, rhs: &Vector) -> Self {
        Vector{
            x: if self.x < rhs.x { self.x } else { rhs.x },
            y: if self.y < rhs.y { self.y } else { rhs.y },
            z: if self.z < rhs.z { self.z } else { rhs.z },
            e: if self.e < rhs.e { self.e } else { rhs.e },
        }
    }

    /// Round to decimal digits
    #[allow(unused)]
    #[inline]
    pub fn rdp(self, digits: u32) -> Vector {
        Vector {
            x: self.x.map(|v| v.rdp(digits.clone())),
            y: self.y.map(|v| v.rdp(digits.clone())),
            z: self.z.map(|v| v.rdp(digits.clone())),
            e: self.e.map(|v| v.rdp(digits)),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn sum(&self) -> Real {
        self.x.unwrap_or(Real::zero()) + self.y.unwrap_or(Real::zero()) +
        self.z.unwrap_or(Real::zero()) + self.e.unwrap_or(Real::zero())
    }
    #[allow(unused)]
    #[inline]
    pub fn max(&self) -> Option<Real> {
        let mut m: Option<Real> = None;
        if let Some(x) = &self.x {
            m = Some(x.clone());
        }
        if let Some(y) = &self.y {
            if let Some(mr) = &m {
                if mr.lt(y) {
                    m = Some(y.clone());
                }
            }
        }
        if let Some(z) = &self.z {
            if let Some(mr) = &m {
                if mr.lt(z) {
                    m = Some(z.clone());
                }
            }
        }
        if let Some(e) = &self.e {
            if let Some(mr) = &m {
                if mr.lt(e) {
                    m = Some(e.clone());
                }
            }
        }
        m
     }

    #[allow(unused)]
    #[inline]
    pub fn min(&self) -> Option<Real> {
        let mut m: Option<Real> = None;
        if let Some(x) = &self.x {
            m = Some(x.clone());
        }
        if let Some(y) = &self.y {
            if let Some(mr) = &m {
                if mr.gt(y) {
                    m = Some(y.clone());
                }
            }
        }
        if let Some(z) = &self.z {
            if let Some(mr) = &m {
                if mr.gt(z) {
                    m = Some(z.clone());
                }
            }
        }
        if let Some(e) = &self.e {
            if let Some(mr) = &m {
                if mr.gt(e) {
                    m = Some(e.clone());
                }
            }
        }
        m
    }

    #[allow(unused)]
    #[inline]
    pub fn norm2(&self) -> Option<Real> {
        self.pow(2).sum().sqrt()
    }

    #[allow(unused)]
    #[inline]
    pub fn div(self, rhs: Real) -> Vector {
        if !rhs.is_zero() {
            Vector {
                x: self.x.map_or_else(|| None, |v| Some(v.div(rhs))),
                y: self.y.map_or_else(|| None, |v| Some(v.div(rhs))),
                z: self.z.map_or_else(|| None, |v| Some(v.div(rhs))),
                e: self.e.map_or_else(|| None, |v| Some(v.div(rhs)))
            }
        }
        else {
            Vector::default()
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn safe_div(self, rhs: &Option<Real>) -> Vector {
        rhs.map_or(Vector::default(), |divisor| {
            if !divisor.is_zero() {
                Vector {
                    x: self.x.map_or_else(|| None, |v| Some(v.div(divisor))),
                    y: self.y.map_or_else(|| None, |v| Some(v.div(divisor))),
                    z: self.z.map_or_else(|| None, |v| Some(v.div(divisor))),
                    e: self.e.map_or_else(|| None, |v| Some(v.div(divisor)))
                }
            }
            else {
                Vector::default()
            }
        })
    }

    #[allow(unused)]
    #[inline]
    pub fn safe_vdiv(self, rhs: &Vector) -> Vector {
        //Some()
        Vector {
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

    #[allow(unused)]
    #[inline]
    pub fn mul(self, rhs: Real) -> Vector {
        Vector {
            x: self.x.map_or_else(|| None, |v| Some(v.mul(rhs))),
            y: self.y.map_or_else(|| None, |v| Some(v.mul(rhs))),
            z: self.z.map_or_else(|| None, |v| Some(v.mul(rhs))),
            e: self.e.map_or_else(|| None, |v| Some(v.mul(rhs)))
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn dot(self, rhs: &Vector) -> Vector {
        Vector {
            x: self.x.map_or(None, |m1| {rhs.x.map_or(None, |m2| Some(m1.mul(m2)))}),
            y: self.y.map_or(None, |m1| {rhs.y.map_or(None, |m2| Some(m1.mul(m2)))}),
            z: self.z.map_or(None, |m1| {rhs.z.map_or(None, |m2| Some(m1.mul(m2)))}),
            e: self.e.map_or(None, |m1| {rhs.e.map_or(None, |m2| Some(m1.mul(m2)))}),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn replacing_nans(&self, rhs: &Real) -> Vector {
        Vector {
            x: self.x.map_or_else(|| Some(rhs.clone()), |v| Some(v)),
            y: self.y.map_or_else(|| Some(rhs.clone()), |v| Some(v)),
            z: self.z.map_or_else(|| Some(rhs.clone()), |v| Some(v)),
            e: self.e.map_or_else(|| Some(rhs.clone()), |v| Some(v)),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn vreplacing_nans(&self, rhs: &Vector) -> Vector {
        Vector {
            x: self.x.map_or_else(|| rhs.x, |v| Some(v)),
            y: self.y.map_or_else(|| rhs.y, |v| Some(v)),
            z: self.z.map_or_else(|| rhs.z, |v| Some(v)),
            e: self.e.map_or_else(|| rhs.e, |v| Some(v)),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn abs(&self) -> Vector {
        Vector {
            x: self.x.map(|v| v.abs()),
            y: self.y.map(|v| v.abs()),
            z: self.z.map(|v| v.abs()),
            e: self.e.map(|v| v.abs()),
        }
    }

    #[cfg(feature = "TODO")]
    #[allow(unused)]
    #[inline]
    pub fn set(&mut self, pos: &XYZ) -> () {
        self.x = pos.x;
        self.y = pos.y;
        self.z = pos.z;
    }

    #[allow(unused)]
    #[inline]
    pub fn unit(&self) -> Self {
        self.safe_div(&self.norm2())
    }
}

impl Add<&Vector> for Vector {
    type Output = Self;

    fn add(self, _other: &Vector) -> Self {
        Self {
            x: self.x.and_then(|x0| _other.x.and_then(|x1| Some(x0 + x1))),
            y: self.y.and_then(|y0| _other.y.and_then(|y1| Some(y0 + y1))),
            z: self.z.and_then(|z0| _other.z.and_then(|z1| Some(z0 + z1))),
            e: self.e.and_then(|e0| _other.e.and_then(|e1| Some(e0 + e1))),
        }
    }
}

impl Sub<&Vector> for Vector {
    type Output = Self;

    fn sub(self, _other: &Vector) -> Self {
        Self {
            x: self.x.and_then(|x0| _other.x.and_then(|x1| Some(x0 - x1))),
            y: self.y.and_then(|y0| _other.y.and_then(|y1| Some(y0 - y1))),
            z: self.z.and_then(|z0| _other.z.and_then(|z1| Some(z0 - z1))),
            e: self.e.and_then(|e0| _other.e.and_then(|e1| Some(e0 - e1))),
        }
    }
}

impl SubAssign<Vector> for Vector {
    fn sub_assign(&mut self, _rhs: Self) {
        /*
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
        self.e -= rhs.e;
         */
        todo!()
    }
}

impl Div<Vector> for Vector {
    type Output = Vector;

    fn div(self, _rhs: Vector) -> Self::Output {
        //Self {x: self.x / rhs.x, y: self.y / rhs.y, z: self.z / rhs.z, e: self.e / rhs.e}
        todo!()
    }
}

impl Mul<Vector> for Vector {
    type Output = Vector;

    fn mul(self, _rhs: Vector) -> Self::Output {
        //Self {x: self.x * rhs.x, y: self.y * rhs.y, z: self.z * rhs.z, e: self.e * rhs.e}
        todo!()
    }
}


impl Debug for Vector {

    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("[{}, {}, {}, {}]",
                                   self.x.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
                                 self.y.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
                                 self.z.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
                                 self.e.map_or_else(|| "NaN".to_string(), |v| v.to_string())
        ))
    }
}

#[cfg(feature = "with-defmt")]
impl crate::hwa::defmt::Format for Vector {
    fn format(&self, fmt: defmt::Formatter) {
        crate::hwa::defmt::write!(
            fmt,
            "[{}, {}, {}, {}]",
            self.x.map_or_else(|| "NaN".to_string(), |v| v.to_string()).as_bytes(),
            self.y.map_or_else(|| "NaN".to_string(), |v| v.to_string()).as_bytes(),
            self.z.map_or_else(|| "NaN".to_string(), |v| v.to_string()).as_bytes(),
            self.e.map_or_else(|| "NaN".to_string(), |v| v.to_string()).as_bytes(),
        )
    }
}

impl Display for Vector {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f,
            "X {} Y {} Z {} E {}",
            self.x.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
            self.y.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
            self.z.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
            self.e.map_or_else(|| "NaN".to_string(), |v| v.to_string()),
        )
    }
}
*/