use crate::hwa::math::Real;

#[allow(unused)]
#[inline]
pub fn to_fixed(val: (i32, u8)) -> Real {
    Real::new(val.0.into(), val.1 as u32)
}
