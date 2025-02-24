//! A computing geometry Q&D API to make facilitate vector operations and provide numerically stability ('undef', etc...).
//! It implies a cost, of course

use crate as hwa;
use bitflags::bitflags;
use core::marker::PhantomData;
use hwa::math::Real;
use num_traits::float::FloatCore;
#[allow(unused)]
use num_traits::ToPrimitive;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-c-axis", feature = "with-i-axis", feature = "with-j-axis", feature = "with-k-axis",
        feature = "with-u-axis", feature = "with-v-axis", feature = "with-w-axis"
    ))] {
        pub type AxisNativeType = u16;
    }
    else {
        pub type AxisNativeType = u8;
    }
}

bitflags! {
    #[derive(PartialEq, Clone, Copy, Eq)]
    pub struct CoordSel: AxisNativeType {

        const UNSET = 0b1 << 0;
        const ALTERNATE_NAME = 0b1 << 1;

        #[cfg(feature = "with-e-axis")]
        /// E (Extruder) coordinate
        const E = 0b1 << 2;

        #[cfg(feature = "with-x-axis")]
        /// X axis coordinate
        const X = 0b1 << 3;
        #[cfg(feature = "with-y-axis")]
        /// Y axis coordinate
        const Y = 0b1 << 4;
        #[cfg(feature = "with-z-axis")]
        /// Z axis coordinate
        const Z = 0b1 << 5;

        #[cfg(feature = "with-a-axis")]
        const A = 0b1 << 6;
        /// A axis coordinate
        #[cfg(feature = "with-b-axis")]
        const B = 0b1 << 7;
        /// B axis coordinate
        #[cfg(feature = "with-c-axis")]
        /// C axis coordinate
        const C = 0b1 << 8;

        #[cfg(feature = "with-i-axis")]
        /// I axis coordinate
        const I = 0b1 << 9;
        #[cfg(feature = "with-j-axis")]
        /// J axis coordinate
        const J = 0b1 << 10;
        #[cfg(feature = "with-k-axis")]
        /// K axis coordinate
        const K = 0b1 << 11;

        #[cfg(feature = "with-u-axis")]
        /// U axis coordinate
        const U = 0b1 << 12;
        #[cfg(feature = "with-v-axis")]
        /// V axis coordinate
        const V = 0b1 << 13;
        #[cfg(feature = "with-w-axis")]
        /// W axis coordinate
        const W = 0b1 << 14;
    }
}
impl CoordSel {
    pub const fn index(&self) -> usize {
        #[allow(unused_mut)]
        let mut next_axis_id: usize = 0;

        #[cfg(feature = "with-e-axis")]
        {
            if self.contains(Self::E) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-x-axis")]
        {
            if self.contains(Self::X) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-y-axis")]
        {
            if self.contains(Self::Y) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-z-axis")]
        {
            if self.contains(Self::Z) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-a-axis")]
        {
            if self.contains(Self::A) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-b-axis")]
        {
            if self.contains(Self::B) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-c-axis")]
        {
            if self.contains(Self::C) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-i-axis")]
        {
            if self.contains(Self::I) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-j-axis")]
        {
            if self.contains(Self::J) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-k-axis")]
        {
            if self.contains(Self::K) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-u-axis")]
        {
            if self.contains(Self::U) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-v-axis")]
        {
            if self.contains(Self::V) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        #[cfg(feature = "with-w-axis")]
        {
            if self.contains(Self::W) {
                return next_axis_id;
            }
            next_axis_id += 1;
        }
        next_axis_id
    }

    pub const fn num_axis() -> usize {
        #[allow(unused_mut)]
        let mut axis_count: usize = 0;
        #[cfg(feature = "with-e-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-x-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-y-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-z-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-a-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-b-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-c-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-i-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-j-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-k-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-u-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-v-axis")]
        {
            axis_count += 1;
        }
        #[cfg(feature = "with-w-axis")]
        {
            axis_count += 1;
        }

        axis_count
    }

    pub const fn is_alternate(&self) -> bool {
        self.contains(Self::ALTERNATE_NAME)
    }

    pub const fn alternative_name(&self) -> &'static str {
        cfg_if::cfg_if! {
            if #[cfg(any(feature = "with-motion-anthropomorphic-kinematics", feature = "with-motion-delta-kinematics"))] {
                match *self {
                    #[cfg(feature = "with-e-axis")]
                    CoordSel::E => "E",
                    //
                    #[cfg(feature = "with-x-axis")]
                    CoordSel::X => "θ(1)",
                    #[cfg(feature = "with-y-axis")]
                    CoordSel::Y => "ϕ(1)",
                    #[cfg(feature = "with-z-axis")]
                    CoordSel::Z => "ψ(1)",
                    //
                    #[cfg(feature = "with-a-axis")]
                    CoordSel::A => "θ(2)",
                    #[cfg(feature = "with-b-axis")]
                    CoordSel::B => "ϕ(2)",
                    #[cfg(feature = "with-c-axis")]
                    CoordSel::C => "ψ(2)",
                    //
                    #[cfg(feature = "with-i-axis")]
                    CoordSel::I => "θ(3)",
                    #[cfg(feature = "with-j-axis")]
                    CoordSel::J => "ϕ(3)",
                    #[cfg(feature = "with-j-axis")]
                    CoordSel::K => "ψ(3)",
                    //
                    #[cfg(feature = "with-u-axis")]
                    CoordSel::U => "θ(4)",
                    #[cfg(feature = "with-v-axis")]
                    CoordSel::V => "ϕ(4)",
                    #[cfg(feature = "with-w-axis")]
                    CoordSel::W => "ψ(4)",
                    _ => "?",
                }
            }
            else {
                self.name()
            }
        }
    }
    pub const fn name(&self) -> &'static str {
        match *self {
            #[cfg(feature = "with-e-axis")]
            CoordSel::E => "E",
            //
            #[cfg(feature = "with-x-axis")]
            CoordSel::X => "X",
            #[cfg(feature = "with-y-axis")]
            CoordSel::Y => "Y",
            #[cfg(feature = "with-z-axis")]
            CoordSel::Z => "Z",
            //
            #[cfg(feature = "with-a-axis")]
            CoordSel::A => "A",
            #[cfg(feature = "with-b-axis")]
            CoordSel::B => "B",
            #[cfg(feature = "with-c-axis")]
            CoordSel::C => "C",
            //
            #[cfg(feature = "with-i-axis")]
            CoordSel::I => "I",
            #[cfg(feature = "with-j-axis")]
            CoordSel::J => "J",
            #[cfg(feature = "with-j-axis")]
            CoordSel::K => "K",
            //
            #[cfg(feature = "with-u-axis")]
            CoordSel::U => "U",
            #[cfg(feature = "with-v-axis")]
            CoordSel::V => "V",
            #[cfg(feature = "with-w-axis")]
            CoordSel::W => "W",
            _w => "?",
        }
    }

    /// All motion relevant axis. Meaning INCLUDING Extruder (E)
    pub const fn all_axis() -> CoordSel {
        CoordSel::all().difference(CoordSel::from_bits_truncate(
            CoordSel::UNSET.bits() | CoordSel::ALTERNATE_NAME.bits(),
        ))
    }

    /// All motion relevant axis EXCLUDING Extruder (E), if exists.
    pub const fn motion_relevant_axis() -> CoordSel {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-e-axis")] {
                CoordSel::all().difference(CoordSel::UNSET.union(CoordSel::E))
            }
            else {
                CoordSel::all().difference(CoordSel::UNSET)
            }
        }
    }
}

impl core::fmt::Debug for CoordSel {
    fn fmt(&self, _fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut _spacing = false;
        for _v in CoordSel::all_axis().iter() {
            if self.contains(_v) {
                if _spacing {
                    core::write!(_fmt, " | ")?
                } else {
                    _spacing = true;
                }
                if _fmt.alternate() {
                    core::write!(_fmt, "{}", _v.alternative_name())?
                } else {
                    core::write!(_fmt, "{}", _v.name())?
                }
            }
        }
        Ok(())
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for CoordSel {
    fn format(&self, _fmt: defmt::Formatter) {
        let mut _spacing = false;
        for _v in CoordSel::all_axis().iter() {
            if self.contains(_v) {
                if _spacing {
                    defmt::write!(_fmt, " | ")
                } else {
                    _spacing = true;
                }
                if self.is_alternate() {
                    defmt::write!(_fmt, "{:a}", _v.alternative_name())
                } else {
                    defmt::write!(_fmt, "{:a}", _v.name())
                }
            }
        }
    }
}

pub trait ArithmeticOps:
    core::fmt::Debug
    + Copy
    + Clone
    + core::ops::Add<Self, Output = Self>
    + core::ops::Mul<Self, Output = Self>
    + core::cmp::PartialEq<Self>
    + core::cmp::PartialOrd<Self>
{
    fn zero() -> Self;
    fn one() -> Self;
    fn is_zero(&self) -> bool;
    fn is_defined_positive(&self) -> bool;
    fn is_negligible(&self) -> bool;
    fn abs(&self) -> Self;
}

pub trait RealOps {
    fn pow(&self, power: i32) -> Self;
    fn sqrt(&self) -> Option<Self>
    where
        Self: Sized;
    fn rdp(&self, digits: u32) -> Self;
    fn ceil(&self) -> Self;
    fn floor(&self) -> Self;
}

#[derive(Copy, Clone, PartialEq)]
pub struct TVector<T>
where
    T: ArithmeticOps + core::fmt::Debug,
{
    _phantom: PhantomData<T>,

    #[cfg(feature = "with-x-axis")]
    pub x: Option<T>,
    #[cfg(feature = "with-y-axis")]
    pub y: Option<T>,
    #[cfg(feature = "with-z-axis")]
    pub z: Option<T>,
    #[cfg(feature = "with-e-axis")]
    pub e: Option<T>,

    #[cfg(feature = "with-a-axis")]
    pub a: Option<T>,
    #[cfg(feature = "with-b-axis")]
    pub b: Option<T>,
    #[cfg(feature = "with-c-axis")]
    pub c: Option<T>,

    #[cfg(feature = "with-i-axis")]
    pub i: Option<T>,
    #[cfg(feature = "with-j-axis")]
    pub j: Option<T>,
    #[cfg(feature = "with-k-axis")]
    pub k: Option<T>,

    #[cfg(feature = "with-u-axis")]
    pub u: Option<T>,
    #[cfg(feature = "with-v-axis")]
    pub v: Option<T>,
    #[cfg(feature = "with-w-axis")]
    pub w: Option<T>,
}

impl<T> TVector<T>
where
    T: ArithmeticOps + core::fmt::Debug,
{
    pub const fn new() -> Self {
        Self::nan()
    }

    pub const fn from_coords(
        #[cfg(feature = "with-e-axis")] e: Option<T>,
        //
        #[cfg(feature = "with-x-axis")] x: Option<T>,
        #[cfg(feature = "with-y-axis")] y: Option<T>,
        #[cfg(feature = "with-z-axis")] z: Option<T>,
        //
        #[cfg(feature = "with-a-axis")] a: Option<T>,
        #[cfg(feature = "with-b-axis")] b: Option<T>,
        #[cfg(feature = "with-c-axis")] c: Option<T>,
        //
        #[cfg(feature = "with-i-axis")] i: Option<T>,
        #[cfg(feature = "with-j-axis")] j: Option<T>,
        #[cfg(feature = "with-k-axis")] k: Option<T>,
        //
        #[cfg(feature = "with-u-axis")] u: Option<T>,
        #[cfg(feature = "with-v-axis")] v: Option<T>,
        #[cfg(feature = "with-w-axis")] w: Option<T>,
    ) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e,
            //
            #[cfg(feature = "with-x-axis")]
            x,
            #[cfg(feature = "with-y-axis")]
            y,
            #[cfg(feature = "with-z-axis")]
            z,
            //
            #[cfg(feature = "with-a-axis")]
            a,
            #[cfg(feature = "with-b-axis")]
            b,
            #[cfg(feature = "with-c-axis")]
            c,
            //
            #[cfg(feature = "with-i-axis")]
            i,
            #[cfg(feature = "with-j-axis")]
            j,
            #[cfg(feature = "with-k-axis")]
            k,
            //
            #[cfg(feature = "with-u-axis")]
            u,
            #[cfg(feature = "with-v-axis")]
            v,
            #[cfg(feature = "with-w-axis")]
            w,
        }
    }

    pub const fn copy_with_coords(&self, _coord_idx: CoordSel, _val: Option<T>) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: if _coord_idx.contains(CoordSel::E) {
                _val
            } else {
                self.e
            },
            //
            #[cfg(feature = "with-x-axis")]
            x: if _coord_idx.contains(CoordSel::X) {
                _val
            } else {
                self.x
            },
            #[cfg(feature = "with-y-axis")]
            y: if _coord_idx.contains(CoordSel::Y) {
                _val
            } else {
                self.y
            },
            #[cfg(feature = "with-z-axis")]
            z: if _coord_idx.contains(CoordSel::Z) {
                _val
            } else {
                self.z
            },
            //
            #[cfg(feature = "with-a-axis")]
            a: if _coord_idx.contains(CoordSel::A) {
                _val
            } else {
                self.a
            },
            #[cfg(feature = "with-b-axis")]
            b: if _coord_idx.contains(CoordSel::B) {
                _val
            } else {
                self.b
            },
            #[cfg(feature = "with-c-axis")]
            c: if _coord_idx.contains(CoordSel::C) {
                _val
            } else {
                self.c
            },
            //
            #[cfg(feature = "with-i-axis")]
            i: if _coord_idx.contains(CoordSel::I) {
                _val
            } else {
                self.i
            },
            #[cfg(feature = "with-j-axis")]
            j: if _coord_idx.contains(CoordSel::J) {
                _val
            } else {
                self.j
            },
            #[cfg(feature = "with-k-axis")]
            k: if _coord_idx.contains(CoordSel::K) {
                _val
            } else {
                self.k
            },
            //
            #[cfg(feature = "with-u-axis")]
            u: if _coord_idx.contains(CoordSel::U) {
                _val
            } else {
                self.u
            },
            #[cfg(feature = "with-v-axis")]
            v: if _coord_idx.contains(CoordSel::V) {
                _val
            } else {
                self.v
            },
            #[cfg(feature = "with-w-axis")]
            w: if _coord_idx.contains(CoordSel::W) {
                _val
            } else {
                self.w
            },
        }
    }

    pub const fn new_with_coord(_coord_idx: CoordSel, _val: Option<T>) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: if _coord_idx.contains(CoordSel::E) {
                _val
            } else {
                None
            },
            //
            #[cfg(feature = "with-x-axis")]
            x: if _coord_idx.contains(CoordSel::X) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-y-axis")]
            y: if _coord_idx.contains(CoordSel::Y) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-z-axis")]
            z: if _coord_idx.contains(CoordSel::Z) {
                _val
            } else {
                None
            },
            //
            #[cfg(feature = "with-a-axis")]
            a: if _coord_idx.contains(CoordSel::A) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-b-axis")]
            b: if _coord_idx.contains(CoordSel::B) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-c-axis")]
            c: if _coord_idx.contains(CoordSel::C) {
                _val
            } else {
                None
            },
            //
            #[cfg(feature = "with-i-axis")]
            i: if _coord_idx.contains(CoordSel::I) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-j-axis")]
            j: if _coord_idx.contains(CoordSel::J) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-k-axis")]
            k: if _coord_idx.contains(CoordSel::K) {
                _val
            } else {
                None
            },
            //
            #[cfg(feature = "with-u-axis")]
            u: if _coord_idx.contains(CoordSel::U) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-v-axis")]
            v: if _coord_idx.contains(CoordSel::V) {
                _val
            } else {
                None
            },
            #[cfg(feature = "with-w-axis")]
            w: if _coord_idx.contains(CoordSel::W) {
                _val
            } else {
                None
            },
        }
    }

    pub fn sign(&self) -> TVector<T>
    where
        T: ArithmeticOps + core::ops::Sub<Output = T>,
    {
        self.map_values(|_c, _v| {
            if !_v.is_defined_positive() {
                Some(T::zero() - T::one())
            } else {
                Some(T::one())
            }
        })
    }

    pub fn map<U, F>(&self, _f: F) -> TVector<U>
    where
        F: Fn(CoordSel, &Option<T>) -> Option<U>,
        U: ArithmeticOps,
    {
        TVector {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: _f(CoordSel::E, &self.e),
            //
            #[cfg(feature = "with-x-axis")]
            x: _f(CoordSel::X, &self.x),
            #[cfg(feature = "with-y-axis")]
            y: _f(CoordSel::Y, &self.y),
            #[cfg(feature = "with-z-axis")]
            z: _f(CoordSel::Z, &self.z),
            //
            #[cfg(feature = "with-a-axis")]
            a: _f(CoordSel::A, &self.a),
            #[cfg(feature = "with-b-axis")]
            b: _f(CoordSel::B, &self.b),
            #[cfg(feature = "with-c-axis")]
            c: _f(CoordSel::C, &self.c),
            //
            #[cfg(feature = "with-i-axis")]
            i: _f(CoordSel::I, &self.i),
            #[cfg(feature = "with-j-axis")]
            j: _f(CoordSel::J, &self.j),
            #[cfg(feature = "with-k-axis")]
            k: _f(CoordSel::K, &self.k),
            //
            #[cfg(feature = "with-u-axis")]
            u: _f(CoordSel::U, &self.u),
            #[cfg(feature = "with-v-axis")]
            v: _f(CoordSel::V, &self.v),
            #[cfg(feature = "with-w-axis")]
            w: _f(CoordSel::W, &self.w),
        }
    }

    pub fn with_coord(&self, selected_coords: CoordSel, new_value: Option<T>) -> TVector<T>
    where
        T: ArithmeticOps,
    {
        self.map(|coord, current_value| {
            if selected_coords.contains(coord) {
                new_value
            } else {
                *current_value
            }
        })
    }

    pub fn map_values<U, F>(&self, f: F) -> TVector<U>
    where
        F: Fn(CoordSel, T) -> Option<U>,
        U: ArithmeticOps,
    {
        self.map(|coord, value| {
            if let Some(v) = value {
                f(coord, *v)
            } else {
                None
            }
        })
    }

    pub fn apply<F>(&mut self, mut _f: F)
    where
        F: FnMut(CoordSel, &Option<T>) -> Option<T>,
    {
        #[cfg(feature = "with-e-axis")]
        {
            self.e = _f(CoordSel::E, &self.e);
        }
        //
        #[cfg(feature = "with-x-axis")]
        {
            self.x = _f(CoordSel::X, &self.x);
        }
        #[cfg(feature = "with-y-axis")]
        {
            self.y = _f(CoordSel::Y, &self.y);
        }
        #[cfg(feature = "with-z-axis")]
        {
            self.z = _f(CoordSel::Z, &self.z);
        }
        //
        #[cfg(feature = "with-a-axis")]
        {
            self.a = _f(CoordSel::A, &self.a);
        }
        #[cfg(feature = "with-b-axis")]
        {
            self.b = _f(CoordSel::B, &self.b);
        }
        #[cfg(feature = "with-c-axis")]
        {
            self.c = _f(CoordSel::C, &self.c);
        }
        //
        #[cfg(feature = "with-i-axis")]
        {
            self.i = _f(CoordSel::I, &self.i);
        }
        #[cfg(feature = "with-j-axis")]
        {
            self.j = _f(CoordSel::J, &self.j);
        }
        #[cfg(feature = "with-k-axis")]
        {
            self.k = _f(CoordSel::K, &self.k);
        }
        //
        #[cfg(feature = "with-u-axis")]
        {
            self.u = _f(CoordSel::U, &self.u);
        }
        #[cfg(feature = "with-v-axis")]
        {
            self.v = _f(CoordSel::V, &self.v);
        }
        #[cfg(feature = "with-w-axis")]
        {
            self.w = _f(CoordSel::W, &self.w);
        }
    }

    pub fn apply_values<F>(&mut self, mut f: F) -> ()
    where
        F: FnMut(CoordSel, &T) -> Option<T>,
    {
        self.apply(|coord, value| {
            if let Some(v) = value {
                f(coord, v)
            } else {
                None
            }
        })
    }

    pub fn foreach<F>(&self, mut _f: F) -> ()
    where
        F: FnMut(CoordSel, &Option<T>) -> (),
    {
        #[cfg(feature = "with-e-axis")]
        _f(CoordSel::E, &self.e);
        //
        #[cfg(feature = "with-x-axis")]
        _f(CoordSel::X, &self.x);
        #[cfg(feature = "with-y-axis")]
        _f(CoordSel::Y, &self.y);
        #[cfg(feature = "with-z-axis")]
        _f(CoordSel::Z, &self.z);
        //
        #[cfg(feature = "with-a-axis")]
        _f(CoordSel::A, &self.a);
        #[cfg(feature = "with-b-axis")]
        _f(CoordSel::B, &self.b);
        #[cfg(feature = "with-c-axis")]
        _f(CoordSel::C, &self.c);
        //
        #[cfg(feature = "with-i-axis")]
        _f(CoordSel::I, &self.i);
        #[cfg(feature = "with-j-axis")]
        _f(CoordSel::J, &self.j);
        #[cfg(feature = "with-k-axis")]
        _f(CoordSel::K, &self.k);
        //
        #[cfg(feature = "with-u-axis")]
        _f(CoordSel::U, &self.u);
        #[cfg(feature = "with-v-axis")]
        _f(CoordSel::V, &self.v);
        #[cfg(feature = "with-w-axis")]
        _f(CoordSel::W, &self.w);
    }

    pub fn foreach_values<F>(&self, mut f: F) -> ()
    where
        F: FnMut(CoordSel, &T) -> (),
    {
        self.foreach(|coord, value| {
            if let Some(v) = value {
                f(coord, v)
            }
        });
    }

    pub fn set_coord(&mut self, _coord_idx: CoordSel, _val: Option<T>) -> &Self {
        #[cfg(feature = "with-e-axis")]
        if _coord_idx.contains(CoordSel::E) {
            self.e = _val
        }
        //
        #[cfg(feature = "with-x-axis")]
        if _coord_idx.contains(CoordSel::X) {
            self.x = _val
        }
        #[cfg(feature = "with-y-axis")]
        if _coord_idx.contains(CoordSel::Y) {
            self.y = _val
        }
        #[cfg(feature = "with-z-axis")]
        if _coord_idx.contains(CoordSel::Z) {
            self.z = _val
        }
        //
        #[cfg(feature = "with-a-axis")]
        if _coord_idx.contains(CoordSel::A) {
            self.a = _val
        }
        #[cfg(feature = "with-b-axis")]
        if _coord_idx.contains(CoordSel::B) {
            self.b = _val
        }
        #[cfg(feature = "with-c-axis")]
        if _coord_idx.contains(CoordSel::C) {
            self.c = _val
        }
        //
        #[cfg(feature = "with-i-axis")]
        if _coord_idx.contains(CoordSel::I) {
            self.i = _val
        }
        #[cfg(feature = "with-j-axis")]
        if _coord_idx.contains(CoordSel::J) {
            self.j = _val
        }
        #[cfg(feature = "with-k-axis")]
        if _coord_idx.contains(CoordSel::K) {
            self.k = _val
        }
        //
        #[cfg(feature = "with-u-axis")]
        if _coord_idx.contains(CoordSel::U) {
            self.u = _val
        }
        #[cfg(feature = "with-v-axis")]
        if _coord_idx.contains(CoordSel::V) {
            self.v = _val
        }
        #[cfg(feature = "with-w-axis")]
        if _coord_idx.contains(CoordSel::W) {
            self.w = _val
        }

        self
    }

    pub const fn get_coord(&self, coord_idx: CoordSel) -> Option<T> {
        match coord_idx {
            #[cfg(feature = "with-e-axis")]
            CoordSel::E => self.e,
            //
            #[cfg(feature = "with-x-axis")]
            CoordSel::X => self.x,
            #[cfg(feature = "with-y-axis")]
            CoordSel::Y => self.y,
            #[cfg(feature = "with-z-axis")]
            CoordSel::Z => self.z,
            //
            #[cfg(feature = "with-a-axis")]
            CoordSel::A => self.a,
            #[cfg(feature = "with-b-axis")]
            CoordSel::B => self.b,
            #[cfg(feature = "with-c-axis")]
            CoordSel::C => self.c,
            //
            #[cfg(feature = "with-i-axis")]
            CoordSel::I => self.i,
            #[cfg(feature = "with-j-axis")]
            CoordSel::J => self.j,
            #[cfg(feature = "with-k-axis")]
            CoordSel::K => self.k,
            //
            #[cfg(feature = "with-u-axis")]
            CoordSel::U => self.u,
            #[cfg(feature = "with-v-axis")]
            CoordSel::V => self.v,
            #[cfg(feature = "with-w-axis")]
            CoordSel::W => self.w,
            _ => None,
        }
    }

    pub fn increment(&mut self, _coord_idx: CoordSel, _val: T) -> &Self {
        #[cfg(feature = "with-e-axis")]
        if _coord_idx.contains(CoordSel::E) {
            self.e = self.e.and_then(|v| Some(v + _val))
        }
        //
        #[cfg(feature = "with-x-axis")]
        if _coord_idx.contains(CoordSel::X) {
            self.x = self.x.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-y-axis")]
        if _coord_idx.contains(CoordSel::Y) {
            self.y = self.y.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-z-axis")]
        if _coord_idx.contains(CoordSel::Z) {
            self.z = self.z.and_then(|v| Some(v + _val))
        }
        //
        #[cfg(feature = "with-a-axis")]
        if _coord_idx.contains(CoordSel::A) {
            self.a = self.a.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-b-axis")]
        if _coord_idx.contains(CoordSel::B) {
            self.b = self.b.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-c-axis")]
        if _coord_idx.contains(CoordSel::C) {
            self.c = self.c.and_then(|v| Some(v + _val))
        }
        //
        #[cfg(feature = "with-i-axis")]
        if _coord_idx.contains(CoordSel::I) {
            self.i = self.i.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-j-axis")]
        if _coord_idx.contains(CoordSel::J) {
            self.j = self.j.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-k-axis")]
        if _coord_idx.contains(CoordSel::K) {
            self.k = self.k.and_then(|v| Some(v + _val))
        }
        //
        #[cfg(feature = "with-u-axis")]
        if _coord_idx.contains(CoordSel::U) {
            self.u = self.u.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-v-axis")]
        if _coord_idx.contains(CoordSel::V) {
            self.v = self.v.and_then(|v| Some(v + _val))
        }
        #[cfg(feature = "with-w-axis")]
        if _coord_idx.contains(CoordSel::W) {
            self.w = self.w.and_then(|v| Some(v + _val))
        }
        self
    }

    pub fn decrement_if_positive(&mut self, _coord_idx: CoordSel) -> bool
    where
        T: ArithmeticOps + core::ops::Sub<Output = T>,
    {
        #[allow(unused_mut)]
        let mut changed = false;

        #[cfg(feature = "with-e-axis")]
        if _coord_idx.contains(CoordSel::E) {
            self.e = self.e.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.e
                }
            })
        }

        #[cfg(feature = "with-x-axis")]
        if _coord_idx.contains(CoordSel::X) {
            self.x = self.x.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.x
                }
            })
        }
        #[cfg(feature = "with-y-axis")]
        if _coord_idx.contains(CoordSel::Y) {
            self.y = self.y.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.y
                }
            })
        }
        #[cfg(feature = "with-z-axis")]
        if _coord_idx.contains(CoordSel::Z) {
            self.z = self.z.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.z
                }
            })
        }

        #[cfg(feature = "with-a-axis")]
        if _coord_idx.contains(CoordSel::A) {
            self.a = self.a.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.a
                }
            })
        }
        #[cfg(feature = "with-b-axis")]
        if _coord_idx.contains(CoordSel::B) {
            self.b = self.b.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.b
                }
            })
        }
        #[cfg(feature = "with-c-axis")]
        if _coord_idx.contains(CoordSel::C) {
            self.c = self.c.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.c
                }
            })
        }

        #[cfg(feature = "with-i-axis")]
        if _coord_idx.contains(CoordSel::I) {
            self.i = self.i.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.i
                }
            })
        }
        #[cfg(feature = "with-j-axis")]
        if _coord_idx.contains(CoordSel::J) {
            self.j = self.j.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.j
                }
            })
        }
        #[cfg(feature = "with-k-axis")]
        if _coord_idx.contains(CoordSel::K) {
            self.k = self.k.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.k
                }
            })
        }

        #[cfg(feature = "with-u-axis")]
        if _coord_idx.contains(CoordSel::U) {
            self.u = self.u.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.u
                }
            })
        }
        #[cfg(feature = "with-v-axis")]
        if _coord_idx.contains(CoordSel::V) {
            self.v = self.v.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.v
                }
            })
        }
        #[cfg(feature = "with-w-axis")]
        if _coord_idx.contains(CoordSel::W) {
            self.w = self.w.and_then(|v| {
                if v.is_defined_positive() {
                    changed = true;
                    Some(v - T::one())
                } else {
                    self.w
                }
            })
        }
        changed
    }

    pub fn assign(&mut self, _coord_idx: CoordSel, _other: &Self) -> &Self {
        #[cfg(feature = "with-e-axis")]
        if _coord_idx.contains(CoordSel::E) {
            self.e = _other.e
        }
        //
        #[cfg(feature = "with-x-axis")]
        if _coord_idx.contains(CoordSel::X) {
            self.x = _other.x
        }
        #[cfg(feature = "with-y-axis")]
        if _coord_idx.contains(CoordSel::Y) {
            self.y = _other.y
        }
        #[cfg(feature = "with-z-axis")]
        if _coord_idx.contains(CoordSel::Z) {
            self.z = _other.z
        }
        //
        #[cfg(feature = "with-a-axis")]
        if _coord_idx.contains(CoordSel::A) {
            self.a = _other.a
        }
        #[cfg(feature = "with-b-axis")]
        if _coord_idx.contains(CoordSel::B) {
            self.b = _other.b
        }
        #[cfg(feature = "with-c-axis")]
        if _coord_idx.contains(CoordSel::C) {
            self.c = _other.c
        }
        //
        #[cfg(feature = "with-i-axis")]
        if _coord_idx.contains(CoordSel::I) {
            self.i = _other.i
        }
        #[cfg(feature = "with-j-axis")]
        if _coord_idx.contains(CoordSel::J) {
            self.j = _other.j
        }
        #[cfg(feature = "with-k-axis")]
        if _coord_idx.contains(CoordSel::K) {
            self.k = _other.k
        }
        //
        #[cfg(feature = "with-u-axis")]
        if _coord_idx.contains(CoordSel::U) {
            self.u = _other.u
        }
        #[cfg(feature = "with-v-axis")]
        if _coord_idx.contains(CoordSel::V) {
            self.v = _other.v
        }
        #[cfg(feature = "with-w-axis")]
        if _coord_idx.contains(CoordSel::W) {
            self.w = _other.w
        }
        self
    }

    pub fn assign_if_set(&mut self, _coord_idx: CoordSel, _other: &Self) -> &Self {
        #[cfg(feature = "with-e-axis")]
        if _coord_idx.contains(CoordSel::E) && _other.e.is_some() {
            self.e = _other.e
        }
        //
        #[cfg(feature = "with-x-axis")]
        if _coord_idx.contains(CoordSel::X) && _other.x.is_some() {
            self.x = _other.x
        }
        #[cfg(feature = "with-y-axis")]
        if _coord_idx.contains(CoordSel::Y) && _other.y.is_some() {
            self.y = _other.y
        }
        #[cfg(feature = "with-z-axis")]
        if _coord_idx.contains(CoordSel::Z) && _other.z.is_some() {
            self.z = _other.z
        }
        //
        #[cfg(feature = "with-a-axis")]
        if _coord_idx.contains(CoordSel::A) && _other.a.is_some() {
            self.a = _other.a
        }
        #[cfg(feature = "with-b-axis")]
        if _coord_idx.contains(CoordSel::B) && _other.b.is_some() {
            self.b = _other.b
        }
        #[cfg(feature = "with-c-axis")]
        if _coord_idx.contains(CoordSel::C) && _other.c.is_some() {
            self.c = _other.c
        }
        //
        #[cfg(feature = "with-i-axis")]
        if _coord_idx.contains(CoordSel::I) && _other.i.is_some() {
            self.i = _other.i
        }
        #[cfg(feature = "with-j-axis")]
        if _coord_idx.contains(CoordSel::J) && _other.j.is_some() {
            self.j = _other.j
        }
        #[cfg(feature = "with-k-axis")]
        if _coord_idx.contains(CoordSel::K) && _other.k.is_some() {
            self.k = _other.k
        }
        //
        #[cfg(feature = "with-u-axis")]
        if _coord_idx.contains(CoordSel::U) && _other.u.is_some() {
            self.u = _other.u
        }
        #[cfg(feature = "with-v-axis")]
        if _coord_idx.contains(CoordSel::V) && _other.v.is_some() {
            self.v = _other.v
        }
        #[cfg(feature = "with-w-axis")]
        if _coord_idx.contains(CoordSel::W) && _other.w.is_some() {
            self.w = _other.w
        }
        self
    }

    pub fn with_coord_if_set(&self, _coord_idx: CoordSel, _val: Option<T>) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: if _coord_idx.contains(CoordSel::E) && self.e.is_some() {
                _val
            } else {
                self.e
            },
            //
            #[cfg(feature = "with-x-axis")]
            x: if _coord_idx.contains(CoordSel::X) && self.x.is_some() {
                _val
            } else {
                self.x
            },
            #[cfg(feature = "with-y-axis")]
            y: if _coord_idx.contains(CoordSel::Y) && self.y.is_some() {
                _val
            } else {
                self.y
            },
            #[cfg(feature = "with-z-axis")]
            z: if _coord_idx.contains(CoordSel::Z) && self.z.is_some() {
                _val
            } else {
                self.z
            },
            //
            #[cfg(feature = "with-a-axis")]
            a: if _coord_idx.contains(CoordSel::A) && self.a.is_some() {
                _val
            } else {
                self.a
            },
            #[cfg(feature = "with-b-axis")]
            b: if _coord_idx.contains(CoordSel::B) && self.b.is_some() {
                _val
            } else {
                self.b
            },
            #[cfg(feature = "with-c-axis")]
            c: if _coord_idx.contains(CoordSel::C) && self.c.is_some() {
                _val
            } else {
                self.c
            },
            //
            #[cfg(feature = "with-i-axis")]
            i: if _coord_idx.contains(CoordSel::I) && self.i.is_some() {
                _val
            } else {
                self.i
            },
            #[cfg(feature = "with-j-axis")]
            j: if _coord_idx.contains(CoordSel::J) && self.j.is_some() {
                _val
            } else {
                self.j
            },
            #[cfg(feature = "with-k-axis")]
            k: if _coord_idx.contains(CoordSel::K) && self.k.is_some() {
                _val
            } else {
                self.k
            },
            //
            #[cfg(feature = "with-u-axis")]
            u: if _coord_idx.contains(CoordSel::U) && self.u.is_some() {
                _val
            } else {
                self.u
            },
            #[cfg(feature = "with-v-axis")]
            v: if _coord_idx.contains(CoordSel::V) && self.v.is_some() {
                _val
            } else {
                self.v
            },
            #[cfg(feature = "with-w-axis")]
            w: if _coord_idx.contains(CoordSel::W) && self.w.is_some() {
                _val
            } else {
                self.w
            },
        }
    }

    pub fn with_coords_if_set(&self, _coord_idx: CoordSel, _other: &Self) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: if _coord_idx.contains(CoordSel::E) && _other.e.is_some() {
                _other.e
            } else {
                self.e
            },
            //
            #[cfg(feature = "with-x-axis")]
            x: if _coord_idx.contains(CoordSel::X) && _other.x.is_some() {
                _other.x
            } else {
                self.x
            },
            #[cfg(feature = "with-y-axis")]
            y: if _coord_idx.contains(CoordSel::Y) && _other.y.is_some() {
                _other.y
            } else {
                self.y
            },
            #[cfg(feature = "with-z-axis")]
            z: if _coord_idx.contains(CoordSel::Z) && _other.z.is_some() {
                _other.z
            } else {
                self.z
            },
            //
            #[cfg(feature = "with-a-axis")]
            a: if _coord_idx.contains(CoordSel::A) && _other.a.is_some() {
                _other.a
            } else {
                self.a
            },
            #[cfg(feature = "with-b-axis")]
            b: if _coord_idx.contains(CoordSel::B) && _other.b.is_some() {
                _other.b
            } else {
                self.b
            },
            #[cfg(feature = "with-c-axis")]
            c: if _coord_idx.contains(CoordSel::C) && _other.c.is_some() {
                _other.c
            } else {
                self.c
            },
            //
            #[cfg(feature = "with-i-axis")]
            i: if _coord_idx.contains(CoordSel::I) && _other.i.is_some() {
                _other.i
            } else {
                self.i
            },
            #[cfg(feature = "with-j-axis")]
            j: if _coord_idx.contains(CoordSel::J) && _other.j.is_some() {
                _other.j
            } else {
                self.j
            },
            #[cfg(feature = "with-k-axis")]
            k: if _coord_idx.contains(CoordSel::K) && _other.k.is_some() {
                _other.k
            } else {
                self.k
            },
            //
            #[cfg(feature = "with-u-axis")]
            u: if _coord_idx.contains(CoordSel::U) && _other.u.is_some() {
                _other.u
            } else {
                self.u
            },
            #[cfg(feature = "with-v-axis")]
            v: if _coord_idx.contains(CoordSel::V) && _other.v.is_some() {
                _other.v
            } else {
                self.v
            },
            #[cfg(feature = "with-w-axis")]
            w: if _coord_idx.contains(CoordSel::W) && _other.w.is_some() {
                _other.w
            } else {
                self.w
            },
        }
    }

    pub fn clamp_higher_than(&self, rhs: TVector<T>) -> TVector<T> {
        self.map_values(|coord, lv| {
            if let Some(rv) = rhs.get_coord(coord) {
                if lv < rv { Some(rv) } else { Some(lv) }
            } else {
                Some(lv)
            }
        })
    }

    pub fn clamp_lower_than(&self, rhs: TVector<T>) -> TVector<T> {
        self.map_values(|coord, lv| {
            if let Some(rv) = rhs.get_coord(coord) {
                if lv > rv { Some(rv) } else { Some(lv) }
            } else {
                Some(lv)
            }
        })
    }

    #[allow(unused)]
    pub fn vmax(&self) -> Option<T> {
        let mut m: Option<T> = None;
        #[cfg(feature = "with-e-axis")]
        if let Some(v) = self.e {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-x-axis")]
        if let Some(v) = self.x {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-y-axis")]
        if let Some(v) = self.y {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-z-axis")]
        if let Some(v) = self.z {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-a-axis")]
        if let Some(v) = self.a {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-b-axis")]
        if let Some(v) = self.b {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-c-axis")]
        if let Some(v) = self.c {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-i-axis")]
        if let Some(v) = self.i {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-j-axis")]
        if let Some(v) = self.j {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-k-axis")]
        if let Some(v) = self.k {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-u-axis")]
        if let Some(v) = self.u {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-v-axis")]
        if let Some(v) = self.v {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-w-axis")]
        if let Some(v) = self.w {
            if let Some(mr) = m {
                if mr.lt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        m
    }

    #[allow(unused)]
    pub fn vmin(&self) -> Option<T> {
        let mut m: Option<T> = None;
        #[cfg(feature = "with-e-axis")]
        if let Some(v) = self.e {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-x-axis")]
        if let Some(v) = self.x {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-y-axis")]
        if let Some(v) = self.y {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-z-axis")]
        if let Some(v) = self.z {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-a-axis")]
        if let Some(v) = self.a {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-b-axis")]
        if let Some(v) = self.b {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-c-axis")]
        if let Some(v) = self.c {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-i-axis")]
        if let Some(v) = self.i {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-j-axis")]
        if let Some(v) = self.j {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-k-axis")]
        if let Some(v) = self.k {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        //
        #[cfg(feature = "with-u-axis")]
        if let Some(v) = self.u {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-v-axis")]
        if let Some(v) = self.v {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }
        #[cfg(feature = "with-w-axis")]
        if let Some(v) = self.w {
            if let Some(mr) = m {
                if mr.gt(&v) {
                    m = Some(v);
                }
            } else {
                m = Some(v);
            }
        }

        m
    }

    /// Evaluates when a vector has all coordinates bounded by other
    /// Meaning that every x, y, z ... of Self is lower than other's
    pub fn bounded_by(&self, _rhs: &TVector<T>) -> bool {
        #[allow(unused_mut)]
        let mut matching_point = true;
        #[cfg(feature = "with-e-axis")]
        match self.e {
            None => {}
            Some(lv) => match _rhs.e {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        //
        #[cfg(feature = "with-x-axis")]
        match self.x {
            None => {}
            Some(lv) => match _rhs.x {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-y-axis")]
        match self.y {
            None => {}
            Some(lv) => match _rhs.y {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-z-axis")]
        match self.z {
            None => {}
            Some(lv) => match _rhs.z {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        //
        #[cfg(feature = "with-a-axis")]
        match self.a {
            None => {}
            Some(lv) => match _rhs.a {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-b-axis")]
        match self.b {
            None => {}
            Some(lv) => match _rhs.b {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-c-axis")]
        match self.c {
            None => {}
            Some(lv) => match _rhs.c {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        //
        #[cfg(feature = "with-i-axis")]
        match self.i {
            None => {}
            Some(lv) => match _rhs.i {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-j-axis")]
        match self.j {
            None => {}
            Some(lv) => match _rhs.j {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-k-axis")]
        match self.k {
            None => {}
            Some(lv) => match _rhs.k {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        //
        #[cfg(feature = "with-u-axis")]
        match self.u {
            None => {}
            Some(lv) => match _rhs.u {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-v-axis")]
        match self.v {
            None => {}
            Some(lv) => match _rhs.v {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        #[cfg(feature = "with-w-axis")]
        match self.w {
            None => {}
            Some(lv) => match _rhs.w {
                None => {
                    if !lv.is_zero() {
                        return false;
                    }
                }
                Some(rv) => {
                    if lv > rv {
                        // coordinate exceeding
                        return false;
                    }
                    if lv < rv {
                        // coordinate preceeding
                        matching_point = false;
                    }
                }
            },
        }
        !matching_point
    }

    pub fn is_nan_or_zero(&self) -> bool {
        #[cfg(feature = "with-e-axis")]
        if let Some(v) = self.e {
            if !v.is_zero() {
                return false;
            }
        }
        //
        #[cfg(feature = "with-x-axis")]
        if let Some(v) = self.x {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-y-axis")]
        if let Some(v) = self.y {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-z-axis")]
        if let Some(v) = self.z {
            if !v.is_zero() {
                return false;
            }
        }
        //
        #[cfg(feature = "with-a-axis")]
        if let Some(v) = self.a {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-b-axis")]
        if let Some(v) = self.b {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-c-axis")]
        if let Some(v) = self.c {
            if !v.is_zero() {
                return false;
            }
        }
        //
        #[cfg(feature = "with-i-axis")]
        if let Some(v) = self.i {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-j-axis")]
        if let Some(v) = self.j {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-k-axis")]
        if let Some(v) = self.k {
            if !v.is_zero() {
                return false;
            }
        }
        //
        #[cfg(feature = "with-u-axis")]
        if let Some(v) = self.u {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-v-axis")]
        if let Some(v) = self.v {
            if !v.is_zero() {
                return false;
            }
        }
        #[cfg(feature = "with-w-axis")]
        if let Some(v) = self.w {
            if !v.is_zero() {
                return false;
            }
        }
        true
    }

    pub const fn nan() -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: None,
            //
            #[cfg(feature = "with-x-axis")]
            x: None,
            #[cfg(feature = "with-y-axis")]
            y: None,
            #[cfg(feature = "with-z-axis")]
            z: None,
            //
            #[cfg(feature = "with-a-axis")]
            a: None,
            #[cfg(feature = "with-b-axis")]
            b: None,
            #[cfg(feature = "with-c-axis")]
            c: None,
            //
            #[cfg(feature = "with-i-axis")]
            i: None,
            #[cfg(feature = "with-j-axis")]
            j: None,
            #[cfg(feature = "with-k-axis")]
            k: None,
            //
            #[cfg(feature = "with-u-axis")]
            u: None,
            #[cfg(feature = "with-v-axis")]
            v: None,
            #[cfg(feature = "with-w-axis")]
            w: None,
        }
    }

    pub fn zero() -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: Some(T::zero()),
            //
            #[cfg(feature = "with-x-axis")]
            x: Some(T::zero()),
            #[cfg(feature = "with-y-axis")]
            y: Some(T::zero()),
            #[cfg(feature = "with-z-axis")]
            z: Some(T::zero()),
            //
            #[cfg(feature = "with-a-axis")]
            a: Some(T::zero()),
            #[cfg(feature = "with-b-axis")]
            b: Some(T::zero()),
            #[cfg(feature = "with-c-axis")]
            c: Some(T::zero()),
            //
            #[cfg(feature = "with-i-axis")]
            i: Some(T::zero()),
            #[cfg(feature = "with-j-axis")]
            j: Some(T::zero()),
            #[cfg(feature = "with-k-axis")]
            k: Some(T::zero()),
            //
            #[cfg(feature = "with-u-axis")]
            u: Some(T::zero()),
            #[cfg(feature = "with-v-axis")]
            v: Some(T::zero()),
            #[cfg(feature = "with-w-axis")]
            w: Some(T::zero()),
        }
    }

    pub fn one() -> Self {
        TVector::new_with_coord(CoordSel::all_axis(), Some(T::one()))
    }

    pub const fn new_with_const_value(val: T) -> Self {
        TVector::new_with_coord(CoordSel::all_axis(), Some(val))
    }

    /// Map all NaN values of any coordinate to specific value
    pub fn map_nan(&self, _value: &T) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(|| Some(*_value), |cv| Some(cv)),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(|| Some(*_value), |cv| Some(cv)),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(|| Some(*_value), |cv| Some(cv)),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(|| Some(*_value), |cv| Some(cv)),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(|| Some(*_value), |cv| Some(cv)),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(|| Some(*_value), |cv| Some(cv)),
        }
    }

    /// Map coords restricted to `_coords`
    pub fn map_coords(&self, _coords: CoordSel, _value: &Option<T>) -> Self {
        self.map(|_c, _v| if _coords.contains(_c) { *_value } else { *_v })
    }

    /// Select only coords in set (assign nan)
    pub fn selecting(&self, _coords: CoordSel) -> Self {
        self.map(|_c, _v| if _coords.contains(_c) { *_v } else { None })
    }

    /// Select only coords in set (assign nan)
    pub fn selecting_negligible(&self, _coords: CoordSel) -> Self {
        self.map(|_c, _v| match _v {
            Some(_value) => {
                if _value.is_negligible() {
                    *_v
                } else {
                    None
                }
            }
            None => *_v,
        })
    }

    /// Filter coords in set (assign nan)
    pub fn excluding(&self, _coords: CoordSel) -> Self {
        self.map(|_c, _v| if _coords.contains(_c) { None } else { *_v })
    }

    /// Filter negligible
    pub fn excluding_negligible(&self) -> Self {
        self.map(|_c, _v| match _v {
            Some(_value) => {
                if _value.is_negligible() {
                    None
                } else {
                    *_v
                }
            }
            None => *_v,
        })
    }

    /// Map NaN values of coordinates restricted to `_coords`
    pub fn map_nan_coords(&self, _coords: CoordSel, _value: &T) -> Self {
        self.map(|_c, _v| {
            if _v.is_none() && _coords.contains(_c) {
                Some(*_value)
            } else {
                *_v
            }
        })
    }

    pub fn map_val(&self, _value: &T) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.and(Some(*_value)),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.and(Some(*_value)),
            #[cfg(feature = "with-y-axis")]
            y: self.y.and(Some(*_value)),
            #[cfg(feature = "with-z-axis")]
            z: self.z.and(Some(*_value)),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.and(Some(*_value)),
            #[cfg(feature = "with-b-axis")]
            b: self.b.and(Some(*_value)),
            #[cfg(feature = "with-c-axis")]
            c: self.c.and(Some(*_value)),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.and(Some(*_value)),
            #[cfg(feature = "with-j-axis")]
            j: self.j.and(Some(*_value)),
            #[cfg(feature = "with-k-axis")]
            k: self.k.and(Some(*_value)),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.and(Some(*_value)),
            #[cfg(feature = "with-v-axis")]
            v: self.v.and(Some(*_value)),
            #[cfg(feature = "with-w-axis")]
            w: self.w.and(Some(*_value)),
        }
    }

    /// Gets the bitset of coords that have a negligible value (Nan, 0 or lower than epsilon)
    pub fn negligible_coords(&self) -> CoordSel
    where
        T: ArithmeticOps,
    {
        #[allow(unused_mut)]
        let mut negligible_set = CoordSel::empty();
        self.foreach(|_c, _v| {
            if let Some(v) = _v {
                negligible_set.set(_c, v.is_negligible());
            } else {
                negligible_set.set(_c, true);
            }
        });
        negligible_set
    }

    /// Gets the bitset of coords that have a NOT negligible value (Nan, 0 or lower than epsilon)
    pub fn not_negligible_coords(&self) -> CoordSel
    where
        T: ArithmeticOps,
    {
        self.negligible_coords()
            .complement()
            .intersection(CoordSel::all_axis())
    }

    pub fn nan_coords(&self) -> CoordSel
    where
        T: ArithmeticOps,
    {
        #[allow(unused_mut)]
        let mut nan_set = CoordSel::empty();
        self.foreach(|_c, _v| {
            nan_set.set(_c, _v.is_none());
        });
        nan_set
    }

    pub fn not_nan_coords(&self) -> CoordSel
    where
        T: ArithmeticOps,
    {
        self.nan_coords()
            .complement()
            .intersection(CoordSel::all_axis())
    }

    pub fn sum(&self) -> T
    where
        T: core::ops::AddAssign<T>,
    {
        #[allow(unused_mut)]
        let mut acc_sum = T::zero();
        #[cfg(feature = "with-e-axis")]
        acc_sum.add_assign(self.e.unwrap_or(T::zero()));
        //
        #[cfg(feature = "with-x-axis")]
        acc_sum.add_assign(self.x.unwrap_or(T::zero()));
        #[cfg(feature = "with-y-axis")]
        acc_sum.add_assign(self.y.unwrap_or(T::zero()));
        #[cfg(feature = "with-z-axis")]
        acc_sum.add_assign(self.z.unwrap_or(T::zero()));
        //
        #[cfg(feature = "with-a-axis")]
        acc_sum.add_assign(self.a.unwrap_or(T::zero()));
        #[cfg(feature = "with-b-axis")]
        acc_sum.add_assign(self.b.unwrap_or(T::zero()));
        #[cfg(feature = "with-c-axis")]
        acc_sum.add_assign(self.c.unwrap_or(T::zero()));
        //
        #[cfg(feature = "with-i-axis")]
        acc_sum.add_assign(self.i.unwrap_or(T::zero()));
        #[cfg(feature = "with-j-axis")]
        acc_sum.add_assign(self.j.unwrap_or(T::zero()));
        #[cfg(feature = "with-k-axis")]
        acc_sum.add_assign(self.k.unwrap_or(T::zero()));
        //
        #[cfg(feature = "with-u-axis")]
        acc_sum.add_assign(self.u.unwrap_or(T::zero()));
        #[cfg(feature = "with-v-axis")]
        acc_sum.add_assign(self.v.unwrap_or(T::zero()));
        #[cfg(feature = "with-w-axis")]
        acc_sum.add_assign(self.w.unwrap_or(T::zero()));
        acc_sum
    }

    pub fn abs(&self) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.and_then(|v| Some(v.abs())),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-y-axis")]
            y: self.y.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-z-axis")]
            z: self.z.and_then(|v| Some(v.abs())),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-b-axis")]
            b: self.b.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-c-axis")]
            c: self.c.and_then(|v| Some(v.abs())),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-j-axis")]
            j: self.j.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-k-axis")]
            k: self.k.and_then(|v| Some(v.abs())),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-v-axis")]
            v: self.v.and_then(|v| Some(v.abs())),
            #[cfg(feature = "with-w-axis")]
            w: self.w.and_then(|v| Some(v.abs())),
        }
    }
}

impl<T> TVector<T>
where
    T: ArithmeticOps + RealOps + core::ops::AddAssign + core::fmt::Debug,
    TVector<T>:
        core::ops::Div<T, Output = TVector<T>> + core::ops::Div<TVector<T>, Output = TVector<T>>,
{
    pub fn pow(&self, _power: i32) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(|| None, |v| Some(v.pow(_power))),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(|| None, |v| Some(v.pow(_power))),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(|| None, |v| Some(v.pow(_power))),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(|| None, |v| Some(v.pow(_power))),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(|| None, |v| Some(v.pow(_power))),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(|| None, |v| Some(v.pow(_power))),
        }
    }

    #[allow(unused)]
    pub fn sqrt(&self) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(|| None, |v| v.sqrt()),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(|| None, |v| v.sqrt()),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(|| None, |v| v.sqrt()),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(|| None, |v| v.sqrt()),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(|| None, |v| v.sqrt()),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(|| None, |v| v.sqrt()),
        }
    }

    pub fn norm2(&self) -> Option<T>
    where
        T: RealOps + core::ops::AddAssign<T>,
    {
        #[allow(unused)]
        use core::ops::Mul;
        let n = self.mul(*self).sum();
        let r = n.sqrt();
        r
    }

    pub fn scalar_product(&self, rhs: TVector<T>) -> T
    where
        T: RealOps + core::ops::AddAssign<T>,
        TVector<T>: core::ops::Mul<TVector<T>, Output = TVector<T>>,
    {
        ((*self) * rhs).sum()
    }

    /// Computes the orthogonal projection of `other` over `this`
    /// proj(self, other) = \frac{self \cdot other}{|self|^(2)}
    pub fn orthogonal_projection(&self, other: TVector<T>) -> T
    where
        T: RealOps + core::ops::AddAssign<T> + core::ops::Div<Output = T>,
        TVector<T>: core::ops::Mul<TVector<T>, Output = TVector<T>>,
    {
        (*self).scalar_product(other) / (self.pow(2).sum().abs())
    }

    pub fn unit(&self) -> Self {
        match self.norm2() {
            None => Self::nan(),
            Some(norm) => match norm.is_zero() {
                true => Self::nan(),
                false => {
                    let t1 = *self;
                    let t2 = norm;
                    t1 / t2
                }
            },
        }
    }
    /***
    custom behavior
     */
    pub fn decompose_normal(&self) -> (Self, T)
    where
        TVector<T>: core::ops::Div<T, Output = TVector<T>>,
        T: ArithmeticOps + RealOps,
    {
        match self.norm2() {
            None => (Self::nan(), T::zero()),
            Some(norm) => match norm.is_zero() {
                true => (Self::nan(), T::zero()),
                false => ((*self) / norm, norm),
            },
        }
    }

    pub fn rdp(&self, _digits: u32) -> TVector<T> {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(|| None, |v| Some(v.rdp(_digits))),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(|| None, |v| Some(v.rdp(_digits))),
        }
    }

    pub fn floor(&self) -> TVector<T>
    where
        T: RealOps,
    {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(|| None, |v| Some(v.floor())),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(|| None, |v| Some(v.floor())),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(|| None, |v| Some(v.floor())),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(|| None, |v| Some(v.floor())),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(|| None, |v| Some(v.floor())),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(|| None, |v| Some(v.floor())),
        }
    }

    pub fn ceil(&self) -> TVector<T>
    where
        T: RealOps,
    {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(|| None, |v| Some(v.ceil())),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(|| None, |v| Some(v.ceil())),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(|| None, |v| Some(v.ceil())),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(|| None, |v| Some(v.ceil())),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(|| None, |v| Some(v.ceil())),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(|| None, |v| Some(v.ceil())),
        }
    }

    pub fn round(&self) -> TVector<T> {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(|| None, |v| Some(v.rdp(0))),
            //
            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(|| None, |v| Some(v.rdp(0))),
            //
            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(|| None, |v| Some(v.rdp(0))),
            //
            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(|| None, |v| Some(v.rdp(0))),
            //
            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(|| None, |v| Some(v.rdp(0))),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(|| None, |v| Some(v.rdp(0))),
        }
    }
}

impl<T> Default for TVector<T>
where
    T: ArithmeticOps + core::fmt::Debug,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> core::fmt::Debug for TVector<T>
where
    T: ArithmeticOps + core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut spacing = false;
        let mut err = None;
        self.foreach_values(|coord, value| {
            if err.is_none() {
                if spacing {
                    err = core::write!(f, " ").err();
                } else {
                    spacing = true;
                }
            }
            if err.is_none() {
                if f.alternate() {
                    err = core::write!(f, "{} {:?}", coord.alternative_name(), value).err();
                } else {
                    err = core::write!(f, "{} {:?}", coord.name(), value).err();
                }
            }
        });
        match err {
            Some(err) => Err(err),
            None => Ok(()),
        }
    }
}

#[cfg(feature = "with-defmt")]
impl<T> defmt::Format for TVector<T>
where
    T: ArithmeticOps + defmt::Format,
{
    // TODO: reimplement to be more efficient

    fn format(&self, fmt: defmt::Formatter) {
        #[allow(unused_mut)]
        let mut spacing = false;

        self.foreach_values(|_c, _v| {
            if spacing {
                defmt::write!(fmt, " ");
            } else {
                spacing = true;
            }
            defmt::write!(fmt, "{} {:?}", _c, _v);
        });
    }
}

impl<T> core::ops::Add for TVector<T>
where
    T: ArithmeticOps + core::ops::Add<Output = T>,
{
    type Output = Self;

    fn add(self, _rhs: Self) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.and_then(|lv| _rhs.e.and_then(|rv| Some(lv + rv))),

            #[cfg(feature = "with-x-axis")]
            x: self.x.and_then(|lv| _rhs.x.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-y-axis")]
            y: self.y.and_then(|lv| _rhs.y.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-z-axis")]
            z: self.z.and_then(|lv| _rhs.z.and_then(|rv| Some(lv + rv))),

            #[cfg(feature = "with-a-axis")]
            a: self.a.and_then(|lv| _rhs.a.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-b-axis")]
            b: self.b.and_then(|lv| _rhs.b.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-c-axis")]
            c: self.c.and_then(|lv| _rhs.c.and_then(|rv| Some(lv + rv))),

            #[cfg(feature = "with-i-axis")]
            i: self.i.and_then(|lv| _rhs.i.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-j-axis")]
            j: self.j.and_then(|lv| _rhs.j.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-k-axis")]
            k: self.k.and_then(|lv| _rhs.k.and_then(|rv| Some(lv + rv))),

            #[cfg(feature = "with-u-axis")]
            u: self.u.and_then(|lv| _rhs.u.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-v-axis")]
            v: self.v.and_then(|lv| _rhs.v.and_then(|rv| Some(lv + rv))),
            #[cfg(feature = "with-w-axis")]
            w: self.w.and_then(|lv| _rhs.w.and_then(|rv| Some(lv + rv))),
        }
    }
}

impl<T> core::ops::AddAssign for TVector<T>
where
    T: ArithmeticOps + core::ops::AddAssign,
{
    fn add_assign(&mut self, _rhs: Self) {
        #[cfg(feature = "with-e-axis")]
        self.e.as_mut().map(|lv| _rhs.e.map(|rv| *lv += rv));

        #[cfg(feature = "with-x-axis")]
        self.x.as_mut().map(|lv| _rhs.x.map(|rv| *lv += rv));
        #[cfg(feature = "with-y-axis")]
        self.y.as_mut().map(|lv| _rhs.y.map(|rv| *lv += rv));
        #[cfg(feature = "with-z-axis")]
        self.z.as_mut().map(|lv| _rhs.z.map(|rv| *lv += rv));

        #[cfg(feature = "with-a-axis")]
        self.a.as_mut().map(|lv| _rhs.a.map(|rv| *lv += rv));
        #[cfg(feature = "with-b-axis")]
        self.b.as_mut().map(|lv| _rhs.b.map(|rv| *lv += rv));
        #[cfg(feature = "with-c-axis")]
        self.c.as_mut().map(|lv| _rhs.c.map(|rv| *lv += rv));

        #[cfg(feature = "with-i-axis")]
        self.i.as_mut().map(|lv| _rhs.i.map(|rv| *lv += rv));
        #[cfg(feature = "with-j-axis")]
        self.j.as_mut().map(|lv| _rhs.j.map(|rv| *lv += rv));
        #[cfg(feature = "with-k-axis")]
        self.k.as_mut().map(|lv| _rhs.k.map(|rv| *lv += rv));

        #[cfg(feature = "with-u-axis")]
        self.u.as_mut().map(|lv| _rhs.u.map(|rv| *lv += rv));
        #[cfg(feature = "with-v-axis")]
        self.v.as_mut().map(|lv| _rhs.v.map(|rv| *lv += rv));
        #[cfg(feature = "with-w-axis")]
        self.w.as_mut().map(|lv| _rhs.w.map(|rv| *lv += rv));
    }
}

impl<T> core::ops::Neg for TVector<T>
where
    T: ArithmeticOps + core::ops::Neg<Output = T>,
{
    type Output = Self;

    fn neg(self) -> Self::Output {
        self.map_values(|_c, _v| Some(_v.neg()))
    }
}

impl<T> core::ops::Sub for TVector<T>
where
    T: ArithmeticOps + core::ops::Sub<Output = T>,
{
    type Output = Self;

    fn sub(self, _rhs: Self) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.and_then(|lv| _rhs.e.and_then(|rv| Some(lv - rv))),

            #[cfg(feature = "with-x-axis")]
            x: self.x.and_then(|lv| _rhs.x.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-y-axis")]
            y: self.y.and_then(|lv| _rhs.y.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-z-axis")]
            z: self.z.and_then(|lv| _rhs.z.and_then(|rv| Some(lv - rv))),

            #[cfg(feature = "with-a-axis")]
            a: self.a.and_then(|lv| _rhs.a.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-b-axis")]
            b: self.b.and_then(|lv| _rhs.b.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-c-axis")]
            c: self.c.and_then(|lv| _rhs.c.and_then(|rv| Some(lv - rv))),

            #[cfg(feature = "with-i-axis")]
            i: self.i.and_then(|lv| _rhs.i.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-j-axis")]
            j: self.j.and_then(|lv| _rhs.j.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-k-axis")]
            k: self.k.and_then(|lv| _rhs.k.and_then(|rv| Some(lv - rv))),

            #[cfg(feature = "with-u-axis")]
            u: self.u.and_then(|lv| _rhs.u.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-v-axis")]
            v: self.v.and_then(|lv| _rhs.v.and_then(|rv| Some(lv - rv))),
            #[cfg(feature = "with-w-axis")]
            w: self.w.and_then(|lv| _rhs.w.and_then(|rv| Some(lv - rv))),
        }
    }
}

impl<T> core::ops::SubAssign for TVector<T>
where
    T: ArithmeticOps + core::ops::SubAssign,
{
    fn sub_assign(&mut self, _rhs: Self) {
        #[cfg(feature = "with-e-axis")]
        self.e.as_mut().map(|lv| _rhs.e.map(|rv| *lv -= rv));

        #[cfg(feature = "with-x-axis")]
        self.x.as_mut().map(|lv| _rhs.x.map(|rv| *lv -= rv));
        #[cfg(feature = "with-y-axis")]
        self.y.as_mut().map(|lv| _rhs.y.map(|rv| *lv -= rv));
        #[cfg(feature = "with-z-axis")]
        self.z.as_mut().map(|lv| _rhs.z.map(|rv| *lv -= rv));

        #[cfg(feature = "with-a-axis")]
        self.a.as_mut().map(|lv| _rhs.a.map(|rv| *lv -= rv));
        #[cfg(feature = "with-b-axis")]
        self.b.as_mut().map(|lv| _rhs.b.map(|rv| *lv -= rv));
        #[cfg(feature = "with-c-axis")]
        self.c.as_mut().map(|lv| _rhs.c.map(|rv| *lv -= rv));

        #[cfg(feature = "with-i-axis")]
        self.i.as_mut().map(|lv| _rhs.i.map(|rv| *lv -= rv));
        #[cfg(feature = "with-j-axis")]
        self.j.as_mut().map(|lv| _rhs.j.map(|rv| *lv -= rv));
        #[cfg(feature = "with-k-axis")]
        self.k.as_mut().map(|lv| _rhs.k.map(|rv| *lv -= rv));

        #[cfg(feature = "with-u-axis")]
        self.u.as_mut().map(|lv| _rhs.u.map(|rv| *lv -= rv));
        #[cfg(feature = "with-v-axis")]
        self.v.as_mut().map(|lv| _rhs.v.map(|rv| *lv -= rv));
        #[cfg(feature = "with-w-axis")]
        self.w.as_mut().map(|lv| _rhs.w.map(|rv| *lv -= rv));
    }
}

impl<T> core::ops::Mul<TVector<T>> for TVector<T>
where
    T: ArithmeticOps + core::ops::Mul<T, Output = T>,
{
    type Output = Self;

    fn mul(self, _rhs: TVector<T>) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.and_then(|lv| _rhs.e.and_then(|rv| Some(lv * rv))),

            #[cfg(feature = "with-x-axis")]
            x: self.x.and_then(|lv| _rhs.x.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-y-axis")]
            y: self.y.and_then(|lv| _rhs.y.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-z-axis")]
            z: self.z.and_then(|lv| _rhs.z.and_then(|rv| Some(lv * rv))),

            #[cfg(feature = "with-a-axis")]
            a: self.a.and_then(|lv| _rhs.a.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-b-axis")]
            b: self.b.and_then(|lv| _rhs.b.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-c-axis")]
            c: self.c.and_then(|lv| _rhs.c.and_then(|rv| Some(lv * rv))),

            #[cfg(feature = "with-i-axis")]
            i: self.i.and_then(|lv| _rhs.i.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-j-axis")]
            j: self.j.and_then(|lv| _rhs.j.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-k-axis")]
            k: self.k.and_then(|lv| _rhs.k.and_then(|rv| Some(lv * rv))),

            #[cfg(feature = "with-u-axis")]
            u: self.u.and_then(|lv| _rhs.u.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-v-axis")]
            v: self.v.and_then(|lv| _rhs.v.and_then(|rv| Some(lv * rv))),
            #[cfg(feature = "with-w-axis")]
            w: self.w.and_then(|lv| _rhs.w.and_then(|rv| Some(lv * rv))),
        }
    }
}

impl<T> core::ops::Mul<T> for TVector<T>
where
    T: ArithmeticOps + core::ops::Mul<T, Output = T>,
{
    type Output = Self;

    fn mul(self, _rhs: T) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.and_then(|v| Some(v * _rhs)),

            #[cfg(feature = "with-x-axis")]
            x: self.x.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-y-axis")]
            y: self.y.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-z-axis")]
            z: self.z.and_then(|v| Some(v * _rhs)),

            #[cfg(feature = "with-a-axis")]
            a: self.a.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-b-axis")]
            b: self.b.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-c-axis")]
            c: self.c.and_then(|v| Some(v * _rhs)),

            #[cfg(feature = "with-i-axis")]
            i: self.i.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-j-axis")]
            j: self.j.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-k-axis")]
            k: self.k.and_then(|v| Some(v * _rhs)),

            #[cfg(feature = "with-u-axis")]
            u: self.u.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-v-axis")]
            v: self.v.and_then(|v| Some(v * _rhs)),
            #[cfg(feature = "with-w-axis")]
            w: self.w.and_then(|v| Some(v * _rhs)),
        }
    }
}

impl<T> core::ops::Div<TVector<T>> for TVector<T>
where
    T: ArithmeticOps + RealOps + core::ops::Div<T, Output = T>,
{
    type Output = TVector<T>;

    fn div(self, _rhs: TVector<T>) -> Self {
        Self {
            _phantom: PhantomData,
            #[cfg(feature = "with-e-axis")]
            e: self.e.map_or_else(
                || None,
                |v| {
                    _rhs.e.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),

            #[cfg(feature = "with-x-axis")]
            x: self.x.map_or_else(
                || None,
                |v| {
                    _rhs.x.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-y-axis")]
            y: self.y.map_or_else(
                || None,
                |v| {
                    _rhs.y.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-z-axis")]
            z: self.z.map_or_else(
                || None,
                |v| {
                    _rhs.z.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),

            #[cfg(feature = "with-a-axis")]
            a: self.a.map_or_else(
                || None,
                |v| {
                    _rhs.a.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-b-axis")]
            b: self.b.map_or_else(
                || None,
                |v| {
                    _rhs.b.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-c-axis")]
            c: self.c.map_or_else(
                || None,
                |v| {
                    _rhs.c.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),

            #[cfg(feature = "with-i-axis")]
            i: self.i.map_or_else(
                || None,
                |v| {
                    _rhs.i.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-j-axis")]
            j: self.j.map_or_else(
                || None,
                |v| {
                    _rhs.j.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-k-axis")]
            k: self.k.map_or_else(
                || None,
                |v| {
                    _rhs.k.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),

            #[cfg(feature = "with-u-axis")]
            u: self.u.map_or_else(
                || None,
                |v| {
                    _rhs.u.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-v-axis")]
            v: self.v.map_or_else(
                || None,
                |v| {
                    _rhs.v.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
            #[cfg(feature = "with-w-axis")]
            w: self.w.map_or_else(
                || None,
                |v| {
                    _rhs.w.map_or_else(
                        || None,
                        |divisor| {
                            if !divisor.is_zero() {
                                Some(v.div(divisor))
                            } else {
                                None
                            }
                        },
                    )
                },
            ),
        }
    }
}

impl<T> core::ops::Div<T> for TVector<T>
where
    T: ArithmeticOps + RealOps + core::ops::Div<T, Output = T>,
{
    type Output = TVector<T>;

    fn div(self, rhs: T) -> Self {
        if rhs.is_zero() {
            Self::nan()
        } else {
            Self {
                _phantom: PhantomData,
                #[cfg(feature = "with-e-axis")]
                e: self.e.and_then(|lv| Some(lv / rhs)),
                //
                #[cfg(feature = "with-x-axis")]
                x: self.x.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-y-axis")]
                y: self.y.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-z-axis")]
                z: self.z.and_then(|lv| Some(lv / rhs)),
                //
                #[cfg(feature = "with-a-axis")]
                a: self.a.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-b-axis")]
                b: self.b.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-c-axis")]
                c: self.c.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-i-axis")]
                i: self.i.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-j-axis")]
                j: self.j.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-k-axis")]
                k: self.k.and_then(|lv| Some(lv / rhs)),

                #[cfg(feature = "with-u-axis")]
                u: self.u.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-v-axis")]
                v: self.v.and_then(|lv| Some(lv / rhs)),
                #[cfg(feature = "with-w-axis")]
                w: self.w.and_then(|lv| Some(lv / rhs)),
            }
        }
    }
}

//////////////

impl ArithmeticOps for i32 {
    fn zero() -> Self {
        0
    }
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
    where
        Self: core::cmp::PartialOrd,
    {
        self.gt(&0)
    }

    fn is_negligible(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0)
    }

    fn abs(&self) -> Self {
        i32::abs(*self)
    }
}

impl ArithmeticOps for u32 {
    fn zero() -> Self {
        0
    }
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
    where
        Self: core::cmp::PartialOrd,
    {
        self.gt(&0)
    }

    fn is_negligible(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for u64 {
    fn zero() -> Self {
        0
    }
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
    where
        Self: core::cmp::PartialOrd,
    {
        self.gt(&0)
    }

    fn is_negligible(&self) -> bool
    where
        Self: core::cmp::PartialOrd,
    {
        self.eq(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for u16 {
    fn zero() -> Self {
        0
    }
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.gt(&0)
    }

    fn is_negligible(&self) -> bool
    where
        Self: core::cmp::PartialOrd,
    {
        self.eq(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for u8 {
    fn zero() -> Self {
        0
    }
    fn one() -> Self {
        1
    }

    fn is_zero(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0)
    }

    fn is_defined_positive(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.gt(&0)
    }

    fn is_negligible(&self) -> bool
    where
        Self: core::cmp::PartialOrd,
    {
        self.eq(&0)
    }

    fn abs(&self) -> Self {
        *self
    }
}

impl ArithmeticOps for f32 {
    fn zero() -> Self {
        0.0f32
    }
    fn one() -> Self {
        1.0f32
    }

    fn is_zero(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.eq(&0.0f32)
    }

    fn is_defined_positive(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.gt(&0.0f32)
    }

    fn is_negligible(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        FloatCore::abs(*self) < <f32 as FloatCore>::epsilon()
    }

    fn abs(&self) -> Self {
        <f32 as FloatCore>::abs(*self)
    }
}

impl RealOps for f32 {
    fn pow(&self, p: i32) -> Self {
        self.powi(p)
    }

    fn sqrt(&self) -> Option<Self>
    where
        Self: Sized,
    {
        if !self.is_sign_negative()  {
            cfg_if::cfg_if! {
                if #[cfg(feature = "fixed-point-128-impl")] {
                    Real::from_f32(*self).sqrt().map(|v| v.0.to_f32())?
                }
                else {
                    Real((*self).into()).sqrt().map(|v| v.0 as f32)
                }
            }
            
        } else {
            None
        }
    }

    fn rdp(&self, digits: u32) -> Self {
        let dd = 10.0f32.powi(digits as i32);
        (self * dd).round() / dd
    }
    fn floor(&self) -> Self {
        <f32 as FloatCore>::floor(*self)
    }

    fn ceil(&self) -> Self {
        <f32 as FloatCore>::ceil(*self)
    }
}

impl ArithmeticOps for Real {
    fn zero() -> Self {
        Real::zero()
    }

    fn one() -> Self {
        Real::one()
    }

    fn is_zero(&self) -> bool {
        Real::is_zero(self)
    }

    fn is_defined_positive(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        self.gt(&Real::zero())
    }

    fn is_negligible(&self) -> bool
    where
        Self: core::cmp::PartialEq,
    {
        Real::is_negligible(self)
    }

    fn abs(&self) -> Self {
        Real::abs(*self)
    }
}

impl RealOps for Real {
    fn pow(&self, p: i32) -> Real {
        self.powi(p)
    }

    fn sqrt(&self) -> Option<Self>
    where
        Self: Sized,
    {
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
