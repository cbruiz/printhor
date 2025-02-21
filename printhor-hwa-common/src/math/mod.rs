//! This module provides a numeric abstraction so anyone can use the proper backend for specific MCU
cfg_if::cfg_if! {
    if #[cfg(feature="float-point-f32-impl")] {
        mod real_f32;
        mod constants_f32;
        pub use real_f32::*;
        pub use constants_f32::*;
    }
    else if #[cfg(feature="float-point-f64-impl")] {
        mod real_f64;
        mod constants_f64;
        pub use real_f64::*;
        pub use constants_f64::*;
    }
    else if #[cfg(feature="fixed-point-128-impl")] {
        mod real_fixedpoint;
        mod constants_fixedpoint;
        pub use real_fixedpoint::*;
        pub use constants_fixedpoint::*;
        pub use rust_decimal;
        pub use rust_decimal_macros;
    }
    else {
        compile_error!("Real arithmetic backend not selected");
    }
}
mod geometry;
pub use geometry::*;

#[cfg(test)]
mod test {
    use crate as hwa;
    #[test]
    fn it_works() {
        let zero = hwa::math::ZERO;
        let one = hwa::math::ONE;
        let x = hwa::make_real!(0.0);
        assert_eq!(x.ceil(), zero, "ceil(0) is zero");
        let y = hwa::make_real!(0.55);
        assert_eq!(y.ceil(), one, "ceil(0.51) is one");
    }
}
