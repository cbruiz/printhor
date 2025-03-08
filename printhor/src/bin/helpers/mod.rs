//! Miscellaneous utilery
use crate::hwa;

#[inline]
pub fn to_fixed(val: (i32, u8)) -> hwa::math::Real {
    hwa::math::Real::new(val.0.into(), val.1 as u32)
}
