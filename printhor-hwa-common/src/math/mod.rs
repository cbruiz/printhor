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
        let zero = math::ZERO;
        let one = math::ONE;
        let two = math::TWO;
        let four = math::TWO + math::TWO;
        let epsilon = Real::epsilon();
        let half_epsilon = epsilon / two;

        use core::ops::Neg;
        let minus_one = one.neg();
        assert_eq!(minus_one, zero - one);

        assert_eq!(Real::one(), one);
        assert_eq!(Real::new(0, 0), zero);
        assert_eq!(Real::new(1, 0), one);
        assert_eq!(Real::new(2, 0), two);
        assert!(Real::one().is_positive());
        assert!(!Real::zero().is_defined_positive());

        assert_eq!(minus_one.abs(), one);
        assert_eq!(minus_one.powi(2), one);
        assert_eq!(minus_one.powi(2), one);
        assert_eq!(two.powi(2), four);
        //assert!(<Real as RealOps>::sqrt(&zero));
        assert_eq!(two.recip(), math::HALF);
        assert_eq!(epsilon, math::EPSILON);
        assert!(epsilon >= zero);
        assert!(!epsilon.is_negligible());
        assert!(half_epsilon.is_negligible());
        assert_eq!(epsilon.round(), zero);
        assert!(!one.is_negligible());
        assert!(!minus_one.is_negligible());
        
        hwa::info!("Epsilon is {:?}", epsilon);
        
        assert_eq!(minus_one.sqrt(), None);
        assert_eq!(one.sqrt().and_then(|v| Some(v.rdp(6))), Some(one));
        assert_eq!(zero.sqrt(), Some(zero));
        
        let x = hwa::make_real!(0.0);
        assert_eq!(x.ceil(), zero, "ceil(0) is zero");
        let y = hwa::make_real!(0.55);
        assert_eq!(y.ceil(), one, "ceil(0.51) is one");

        assert_eq!(Real::vmax(None, None), None);
        assert_eq!(Real::vmax(Some(zero), Some(one)), Some(one));
        assert_eq!(Real::vmax(Some(one), None), Some(one));
        assert_eq!(Real::vmax(None, Some(one)), None);

        assert_eq!(Real::vmin(None, None), None);
        assert_eq!(Real::vmin(Some(zero), Some(one)), Some(zero));
        assert_eq!(Real::vmin(Some(zero), None), Some(zero));
        assert_eq!(Real::vmin(None, Some(zero)), None);
        
        assert!(one.is_positive());
        assert_eq!(zero, Real::from_inner(0.0));
        assert_eq!(1i64, one.to_i64().unwrap());
        assert_eq!(1i32, one.rdp(0).to_i32().unwrap());
        assert_eq!(1i64, one.int());
        assert_eq!(1f64, one.to_f64());
        assert_eq!(2i32, (math::ONE + math::ONE + math::ONE + math::ONE).sqrt().unwrap().to_i32().unwrap());
        assert_eq!(Real::vmax(Some(one), Some(zero)), Some(one));
        assert_eq!(Real::vmax(None, Some(zero)), None);
        assert_eq!(Real::vmax(Some(one), None), Some(one));
        assert!(one > zero);
        assert_eq!(one.clamp(zero, zero), zero);
        
        let mut sum = one;
        sum += one;
        assert_eq!(sum, two);
        sum -= one;
        assert_eq!(sum, one);

        let mut scale = two;
        scale *= two;
        assert_eq!(scale, four);
        scale /= two;
        assert_eq!(scale, two);
        
        assert_eq!(two.partial_cmp(&two), Some(core::cmp::Ordering::Equal));
        assert!(two.cmp(&two).is_eq());
        let ordering_result = one.cmp(&two);
        assert!(ordering_result.is_ne());
        assert!(ordering_result.is_le());
        
        assert_eq!(one.max(epsilon), one);
        assert_eq!(epsilon.max(one), one);
        assert_eq!(epsilon.max(zero), epsilon);
        assert_eq!(epsilon.clamp(zero, one), epsilon);
        assert_eq!(minus_one.clamp(zero, one), zero);
        assert_eq!(two.clamp(zero, one), one);
        
        
    }

    #[test]
    fn test_format() {
        assert_eq!(format!("{:?}", math::ZERO).as_str(), "0.0");
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
    
    #[test]
    fn test_axis() {
        use crate as printhor_hwa_common;
        let c = CoordSel::X;
        assert!(!c.is_alternate());
        assert_eq!(CoordSel::UNSET.name(), "?");
        // Format works
        let _ = format!("{:?}", c);
        // Alternate format works
        let _ = format!("{:#?}", c);
        
        let one  = TVector::one();
        let two = one.copy_with_coords(CoordSel::all_axis(), Some(2));
        assert_eq!(one + one, two);
        let empty = TVector::new_with_coord(CoordSel::empty(), Some(1));
       
        assert_eq!(empty + empty, empty);
        
        let mut t2 = empty;
        t2.set_coord(CoordSel::E, Some(1));
        assert_ne!(t2, empty);
        t2.set_coord(CoordSel::E, None);
        assert_eq!(t2, empty);

        assert!(TVector::<Real>::new().is_nan_or_zero());
        assert!(!hwa::make_vector!(e=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(x=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(y=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(z=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(a=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(b=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(c=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(i=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(j=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(k=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(u=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(v=1).is_nan_or_zero());
        assert!(!hwa::make_vector!(w=1).is_nan_or_zero());
        
        assert_eq!(TVector::new_with_const_value(1.0), TVector::one());
        assert_eq!(TVector::new().map_nan(&Real::one()), TVector::one());
        
        assert_eq!(TVector::new().map_coords(CoordSel::all(), &Some(math::ONE) ), TVector::one());
        assert_eq!(TVector::new().map_coords(CoordSel::empty(), &Some(math::ONE) ), TVector::new());
        
        assert_eq!(TVector::<Real>::one().selecting(CoordSel::all()), TVector::one());
        assert_eq!(TVector::<Real>::one().selecting(CoordSel::empty()), TVector::new());

        assert_eq!(TVector::<Real>::one().selecting_negligible(CoordSel::all()), TVector::new());
        assert_eq!(TVector::<Real>::zero().selecting_negligible(CoordSel::all()), TVector::zero());
        assert_eq!(TVector::<Real>::new().selecting_negligible(CoordSel::all()), TVector::new());

        assert_eq!(TVector::<Real>::one().excluding(CoordSel::all()), TVector::new());
        assert_eq!(TVector::<Real>::one().excluding(CoordSel::empty()), TVector::one());

        assert_eq!(TVector::<Real>::one().excluding_negligible(), TVector::one());
        assert_eq!(TVector::<Real>::zero().excluding_negligible(), TVector::new());
        assert_eq!(TVector::<Real>::new().excluding_negligible(), TVector::new());

        assert_eq!(TVector::new().map_nan_coords(CoordSel::all(), &math::ONE), TVector::one());
        assert_eq!(TVector::new().map_nan_coords(CoordSel::empty(), &math::ONE), TVector::new());

        assert_eq!(TVector::<Real>::new().map_val(&math::ONE), TVector::new());
        assert_eq!(TVector::<Real>::one().map_val(&math::ZERO), TVector::zero());

        assert_eq!(TVector::<Real>::one().sum(), Real::from_lit(CoordSel::num_axis() as i64,0));

        assert_eq!(TVector::<Real>::one().pow(1), TVector::<Real>::one());

        let num_axis = Real::from_lit(CoordSel::num_axis() as i64,0);
        assert_eq!(TVector::<Real>::new().norm2(), Some(math::ZERO));
        assert_eq!(TVector::one().with_coord_if_set(CoordSel::all(), Some(math::TWO)).unit().norm2().unwrap().rdp(6), math::ONE);
        assert_eq!(TVector::<Real>::one().norm2(), num_axis.sqrt());

        assert_eq!(TVector::<Real>::new().round(), TVector::new());
        assert_eq!(TVector::<Real>::zero().round(), TVector::zero());

        assert_eq!(TVector::<Real>::default(), TVector::new());

        let _ = std::format!("{:?}", TVector::<Real>::zero());
        let _ = std::format!("{:?}", TVector::<Real>::new());

        let _ = std::format!("{:#?}", TVector::<Real>::zero());
        let _ = std::format!("{:#?}", TVector::<Real>::new());

        let one = TVector::<Real>::one();
        let zero = TVector::<Real>::zero();
        let none = TVector::<Real>::new();
        assert_eq!(one + zero, one);
        let mut zero_prima = zero;
        zero_prima += zero;
        assert_eq!(zero_prima, zero);
        zero_prima += one;
        assert_eq!(zero_prima, one);
        zero_prima -= one;
        assert_eq!(zero_prima, zero);

        assert_eq!(zero_prima * one, zero);
        
        assert_eq!(one * none, none);
        assert_eq!(one / none, none);
        assert_eq!(one / one, one);
        assert_eq!(one / zero, none);

        assert_eq!(one * math::ONE, one);
        assert_eq!(one / math::ONE, one);
        assert_eq!(one / math::ZERO, none);

        assert_eq!(one.get_coord(CoordSel::E), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::X), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::Y), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::Z), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::A), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::B), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::C), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::I), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::J), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::K), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::U), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::V), Some(math::ONE));
        assert_eq!(one.get_coord(CoordSel::W), Some(math::ONE));
        
        use core::ops::Neg;
        let minus_one = one.neg();
        let mut acc = zero;
        acc.increment(CoordSel::all(), math::ONE);
        assert_eq!(acc, one);
        acc.decrement_if_positive(CoordSel::all());
        assert_eq!(acc, zero);
        acc.increment(CoordSel::empty(), math::ONE);
        assert_eq!(acc, zero);
        let mut acc2 = minus_one;
        acc2.decrement_if_positive(CoordSel::all());
        assert_eq!(acc2, minus_one);
        acc2.decrement_if_positive(CoordSel::empty());
        assert_eq!(acc2, minus_one);
        
        acc2.assign(CoordSel::empty(), &one);
        assert_eq!(acc2, minus_one);
        acc2.assign(CoordSel::all(), &one);
        assert_eq!(acc2, one);
        
        acc2 = acc2.with_coord_if_set(CoordSel::empty(), Some(math::TWO));
        assert_eq!(acc2, one);
        acc2 = acc2.with_coord_if_set(CoordSel::all(), Some(math::ZERO - math::ONE));
        assert_eq!(acc2, minus_one);

        acc2 = acc2.with_coords_if_set(CoordSel::empty(), &one);
        assert_eq!(acc2, minus_one);
        acc2 = acc2.with_coords_if_set(CoordSel::all(), &one);
        assert_eq!(acc2, one);
        
        assert_eq!(minus_one.clamp_lower_than(zero), minus_one);
        assert_eq!(minus_one.clamp_lower_than(none), minus_one);
        
        assert_eq!(minus_one.clamp_higher_than(zero), zero);
        assert_eq!(one.clamp_higher_than(zero), one);
        assert_eq!(one.clamp_higher_than(none), one);
        let two = one + one;

        assert_eq!(two.copy_with_coords(CoordSel::E, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::E, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::X, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::X, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::Y, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::Y, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::Z, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::Z, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::A, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::A, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::B, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::B, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::C, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::C, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::I, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::I, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::J, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::J, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::K, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::K, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::U, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::U, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::V, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::V, Some(math::ONE)).vmin(), Some(math::ONE));
        assert_eq!(two.copy_with_coords(CoordSel::W, Some(math::ONE)).vmax(), Some(math::TWO));
        assert_eq!(two.copy_with_coords(CoordSel::W, Some(math::ONE)).vmin(), Some(math::ONE));
        
        let (_, _) = one.decompose_normal();
        

        
        
        // vmax, vmin
        
        // unit
        
    }
}
