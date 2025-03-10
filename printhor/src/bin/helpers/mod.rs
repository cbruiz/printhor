//! Miscellaneous utils
use crate::hwa;

/// Converts an `async-gcode` decimal representation in the format of tuple (signed_decimal: i32, number_of_decimal_digits: u8) to [hwa::math::Real]
#[inline]
pub fn to_fixed(val: (i32, u8)) -> hwa::math::Real {
    hwa::math::Real::new(val.0.into(), val.1 as u32)
}
