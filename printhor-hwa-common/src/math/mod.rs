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
    use hwa::CoordSel;
    use crate::math;
    use crate::math::Real;

    #[test]
    fn test_real() {
        let zero = hwa::math::ZERO;
        let one = hwa::math::ONE;
        let x = hwa::make_real!(0.0);
        assert_eq!(x.ceil(), zero, "ceil(0) is zero");
        let y = hwa::make_real!(0.55);
        assert_eq!(y.ceil(), one, "ceil(0.51) is one");
        
        assert_eq!(zero.sin(), zero, "sin(0.0) is zero");
        assert_eq!(zero.cos(), one, "cos(0.0) is zero");
        
        assert!(one.is_positive());
        assert_eq!(zero, hwa::math::Real::from_inner(0.0));
        assert_eq!(1i64, one.to_i64().unwrap());
        assert_eq!(1i64, one.int());
        assert_eq!(1f64, one.to_f64());
        assert_eq!(Real::from_f32(180.0f32), math::PI.r2d());
        assert_eq!(math::PI, Real::from_f32(180.0f32).d2r());
        assert_eq!(Real::from_f32(180.0f32).sign(), one);
        assert_eq!(Real::vmax(Some(one), Some(zero)), Some(one));
        assert_eq!(Real::vmax(None, Some(zero)), None);
        assert_eq!(Real::vmax(Some(one), None), Some(one));
        assert!(one > zero);
        assert_eq!(one.clamp(zero, zero), zero);
        assert_eq!((math::PI / math::FOUR).tan(), math::ONE);
        assert_eq!(math::HALF.atan2(math::HALF), math::PI / math::FOUR);
    }

    #[test]
    fn test_vector_i32() {
        use crate::math::TVector;
        use crate as printhor_hwa_common;
        
        let zero: TVector<i32> = hwa::make_vector!(x=0, y=0, z=0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());
        
        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(relevant_coords, zero.negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_nan_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, zero.not_nan_coords().intersection(relevant_coords));

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[test]
    fn test_vector_f32() {
        use crate::math::TVector;
        use crate as printhor_hwa_common;

        let zero: TVector<f32> = hwa::make_vector!(x=0.0, y=0.0, z=0.0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1.0));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1.0));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(relevant_coords, zero.negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_nan_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, zero.not_nan_coords().intersection(relevant_coords));

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[test]
    fn test_vector_u32() {
        use crate::math::TVector;
        use crate as printhor_hwa_common;

        let zero: TVector<u32> = hwa::make_vector!(x=0, y=0, z=0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(relevant_coords, zero.negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_nan_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, zero.not_nan_coords().intersection(relevant_coords));

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[test]
    fn test_vector_u64() {
        use crate::math::TVector;
        use crate as printhor_hwa_common;

        let zero: TVector<u64> = hwa::make_vector!(x=0, y=0, z=0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(relevant_coords, zero.negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_nan_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, zero.not_nan_coords().intersection(relevant_coords));
        
        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[test]
    fn test_vector_u16() {
        use crate::math::TVector;
        use crate as printhor_hwa_common;

        let zero: TVector<u16> = hwa::make_vector!(x=0, y=0, z=0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(relevant_coords, zero.negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_nan_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, zero.not_nan_coords().intersection(relevant_coords));
        
        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[test]
    fn test_vector_u8() {
        use crate::math::TVector;
        use crate as printhor_hwa_common;

        let zero: TVector<u8> = hwa::make_vector!(x=0, y=0, z=0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(relevant_coords, zero.negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_negligible_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, one.not_nan_coords().intersection(relevant_coords));
        assert_eq!(relevant_coords, zero.not_nan_coords().intersection(relevant_coords));
        
        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[test]
    fn test_vector_real() {
        let zero = hwa::make_vector_real!(x=0.0, y=0.0, z=0.0);
        assert_eq!(zero, zero.ceil());

        let zero = hwa::make_vector_real!(x=0.0, y=0.0, z=0.0);
        assert_eq!(zero, zero.floor());
    }
}
