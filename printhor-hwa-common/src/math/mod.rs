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
    use hwa::math;
    use math::CoordSel;
    use math::Real;
    use math::TVector;

    #[test]
    fn test_real() {
        let zero = hwa::math::ZERO;
        let one = hwa::math::ONE;
        
        assert_eq!(Real::zero(), zero);
        assert_eq!(Real::one(), one);
        
        let x = hwa::make_real!(0.0);
        assert_eq!(x.ceil(), zero, "ceil(0) is zero");
        let y = hwa::make_real!(0.55);
        assert_eq!(y.ceil(), one, "ceil(0.51) is one");
        
        assert!(one.is_positive());
        assert_eq!(zero, hwa::math::Real::from_inner(0.0));
        assert_eq!(1i64, one.to_i64().unwrap());
        assert_eq!(1i64, one.int());
        assert_eq!(1f64, one.to_f64());
        assert_eq!(Real::vmax(Some(one), Some(zero)), Some(one));
        assert_eq!(Real::vmax(None, Some(zero)), None);
        assert_eq!(Real::vmax(Some(one), None), Some(one));
        assert!(one > zero);
        assert_eq!(one.clamp(zero, zero), zero);
    }

    #[test]
    fn test_trigonometry() {

        assert_eq!(math::ZERO.sin(), math::ZERO, "sin(0.0) is zero");
        assert_eq!(math::ZERO.cos(), math::ONE, "cos(0.0) is zero");
        assert_eq!(math::ONE.acos(), math::ZERO, "acos(1.0) is zero");
        
        assert_eq!(math::ONE.ln().exp(), math::ONE, "exp(ln(1)) is ONE");
        
        assert_eq!(Real::from_f32(180.0f32), math::PI.r2d());
        assert_eq!(math::PI, Real::from_f32(180.0f32).d2r());
        assert_eq!(Real::from_f32(180.0f32).sign(), math::ONE);
        assert_eq!((math::PI / math::FOUR).tan(), math::ONE);
        assert_eq!(math::HALF.atan2(math::HALF), math::PI / math::FOUR);
        
        let _one: TVector<f32> = TVector::one();
        let _zero: TVector<f32> = TVector::zero();
        
        let _two = math::TWO.to_f64() as f32;
        let four_four = TVector::new_with_coord(CoordSel::X.union(CoordSel::Y), Some(4.0f32));
        let two_two = TVector::new_with_coord(CoordSel::X.union(CoordSel::Y), Some(2.0f32));

        assert_eq!(four_four.sqrt().rdp(6), two_two);
        
    }

    #[test]
    fn test_vector_bounds_etc() {

        let empty = TVector::new();
        let one = TVector::one();
        let zero = TVector::zero();
        let one_prima = zero.with_coord_if_set(CoordSel::all_axis(), Some(math::ONE));
        assert_eq!(one, one_prima);
        let empty_prima = empty.with_coord_if_set(CoordSel::all_axis(), Some(math::ONE));
        assert_eq!(empty, empty_prima);

        assert_eq!(one.vmax(), Some(math::ONE));
        assert_eq!(one.vmin(), Some(math::ONE));
        assert_eq!(zero.vmin(), Some(math::ZERO));
        assert_eq!(zero.vmax(), Some(math::ZERO));
        assert_eq!(empty.vmax(), None);
        assert_eq!(empty.vmin(), None);
        
        assert!(!one.bounded_by(&zero));
        assert!(zero.bounded_by(&one));
        assert!(!zero.bounded_by(&empty));
        assert!(!one.bounded_by(&empty));
        
        assert!(zero.is_nan_or_zero());
        assert!(empty.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

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
