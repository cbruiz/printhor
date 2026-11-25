//! This module provides a numeric abstraction so anyone can use the proper backend for specific MCU
cfg_if::cfg_if! {
    if #[cfg(feature="fixed-point-128-impl")] {
        mod real_fixedpoint;
        mod constants_fixedpoint;
        pub use real_fixedpoint::*;
        pub use constants_fixedpoint::*;
        pub use rust_decimal;
        pub use rust_decimal_macros;
    }
    else if #[cfg(feature="float-point-f64-impl")] {
        mod real_f64;
        mod constants_f64;
        pub use real_f64::*;
        pub use constants_f64::*;
    }
    else if #[cfg(feature="float-point-f32-impl")] {
        mod real_f32;
        mod constants_f32;
        pub use real_f32::*;
        pub use constants_f32::*;
    }
    else {
        compile_error!(
            "printhor-hwa-common needs a math backend: enable one of the features \
            `float-point-f32-impl`, `float-point-f64-impl` or `fixed-point-128-impl`"
        );
    }
}
mod geometry;

pub use geometry::*;

/// Returns the smallest strictly positive real root of `a·x² + b·x + c = 0`, if any.
///
/// Numerically stable formulation (no catastrophic cancellation): with
/// `q = -(b + sign(b)·√Δ)/2` the roots are `q/a` and `c/q`.
/// Degenerates gracefully to the linear case when `a == 0`.
///
/// Never panics: a negative discriminant (no real roots) returns `None`.
pub fn solve_quatratic(a: Real, b: Real, c: Real) -> Option<Real> {
    if a.is_zero() {
        if b.is_zero() {
            return None;
        }
        let r = -c / b;
        return if r.is_defined_positive() { Some(r) } else { None };
    }
    let discriminant = b * b - FOUR * a * c;
    let sqrt_d = discriminant.sqrt()?;
    let q = if b >= ZERO {
        -(b + sqrt_d) * HALF
    } else {
        (sqrt_d - b) * HALF
    };
    let r1 = q / a;
    let r2 = if q.is_zero() { r1 } else { c / q };
    smallest_positive(&[Some(r1), Some(r2)])
}

/// Alias with the correct spelling.
#[inline]
pub fn solve_quadratic(a: Real, b: Real, c: Real) -> Option<Real> {
    solve_quatratic(a, b, c)
}

#[inline]
fn smallest_positive(roots: &[Option<Real>]) -> Option<Real> {
    let mut best: Option<Real> = None;
    for r in roots.iter().flatten() {
        if r.is_defined_positive() {
            best = match best {
                Some(b) if b <= *r => Some(b),
                _ => Some(*r),
            };
        }
    }
    best
}

/// Returns the smallest strictly positive real root of `a·x³ + b·x² + c·x + d = 0`, if any.
///
/// Closed form (Cardano / trigonometric) gives the initial estimates, which are then polished
/// with two Newton iterations on the *original* polynomial. The polishing step recovers the
/// accuracy lost by cancellation in Cardano's formula and by the reduced-precision
/// transcendental functions (`cbrt`, `acos`, `cos`) of some numeric backends.
///
/// Never panics.
///
/// <https://en.wikipedia.org/wiki/Cubic_equation>
pub fn solve_cubic(a: Real, b: Real, c: Real, d: Real) -> Option<Real> {
    if a.is_negligible() {
        return solve_quatratic(b, c, d);
    }

    let a1 = b / a;
    let a2 = c / a;
    let a3 = d / a;
    let shift = a1 * THIRD;

    // Depressed cubic t³ + p·t + q = 0 with x = t - a1/3
    let p = a2 - a1 * a1 * THIRD;
    let q = TWO * a1 * a1 * a1 / TWENTY_SEVEN - a1 * a2 * THIRD + a3;
    let discriminant = q * q / FOUR + p * p * p / TWENTY_SEVEN;

    let mut roots: [Option<Real>; 3] = [None, None, None];
    if discriminant > ZERO {
        // One real root. Choose the cube-root argument that avoids cancellation and
        // derive the conjugate term through u·v = -p/3.
        let sqrt_d = discriminant.sqrt()?;
        let w = if q >= ZERO {
            -q * HALF - sqrt_d
        } else {
            -q * HALF + sqrt_d
        };
        let u = w.cbrt();
        let v = if u.is_zero() { ZERO } else { -p / (THREE * u) };
        roots[0] = Some(u + v - shift);
    } else if p >= ZERO {
        // Triple root (p = q = 0). With rounding, p may come out tiny positive while the
        // discriminant underflows to zero: mathematically p > 0 implies discriminant > 0
        roots[0] = Some(-shift);
    } else {
        // Three real roots (p < 0): t_k = m·cos(φ - 2πk/3)
        let m = TWO * (-p * THIRD).sqrt()?;
        // cos(3φ) = (3q / 2p)·√(-3/p) = 3q / (p·m)
        let arg = (THREE * q / (p * m)).clamp(-ONE, ONE);
        let phi = arg.acos() * THIRD;
        let two_pi_thirds = TWO * PI * THIRD;
        for (k, slot) in roots.iter_mut().enumerate() {
            let angle = phi - two_pi_thirds * Real::from_lit(k as i64, 0);
            *slot = Some(m * angle.cos() - shift);
        }
    }

    // Newton polishing on the original monic polynomial, until convergence (at most 4 steps).
    // The closed-form estimate can be off by ~1e-2 when `cbrt`/`acos`/`cos` are low-precision
    // approximations (micromath, also used by the f64 backend); quadratic convergence then needs
    // up to 3-4 steps to reach full precision (1e-2 → 1e-4 → 1e-8 → 1e-16).
    for slot in roots.iter_mut() {
        if let Some(x0) = *slot {
            let mut x = x0;
            for _ in 0..4 {
                let f = ((x + a1) * x + a2) * x + a3;
                let df = (THREE * x + TWO * a1) * x + a2;
                if df.is_zero() {
                    break;
                }
                let dx = f / df;
                x = x - dx;
                if !(dx.abs() > x.abs() * EPSILON * FOUR) {
                    break;
                }
            }
            *slot = Some(x);
        }
    }
    smallest_positive(&roots)
}

#[cfg(test)]
mod solver_test {
    use super::*;

    fn close(a: Real, b: f64, rel: f64) -> bool {
        let x = a.to_f64();
        (x - b).abs() <= rel * b.abs().max(1.0)
    }

    /// The power-of-ten fast path of `from_lit` is bit-identical to the former `powi` division.
    #[test]
    #[cfg(all(
        any(feature = "float-point-f32-impl", feature = "float-point-f64-impl"),
        not(feature = "fixed-point-128-impl")
    ))]
    fn from_lit_matches_powi() {
        use num_traits::float::FloatCore;
        type F = RealImpl;
        for scale in 0..=24u32 {
            for num in [0i64, 1, -1, 7, 12345, -987654, 31_415_926, i32::MAX as i64] {
                let expected = (num as F) / FloatCore::powi(10.0 as F, scale as i32);
                assert_eq!(
                    Real::from_lit(num, scale).inner().to_bits(),
                    expected.to_bits(),
                    "from_lit({}, {})",
                    num,
                    scale
                );
            }
        }
    }

    #[test]
    fn sqrt_is_accurate() {
        // Backend-dependent precision target
        #[cfg(feature = "float-point-f64-impl")]
        let tol = 4.0e-16;
        #[cfg(feature = "fixed-point-128-impl")]
        let tol = 1.0e-12;
        #[cfg(not(any(feature = "float-point-f64-impl", feature = "fixed-point-128-impl")))]
        let tol = 2.4e-7;
        for &v in &[1.0e-6f64, 2.0e-3, 0.5, 1.0, 2.0, 3.0, 10.0, 123.456, 9.0e3, 1.0e6, 7.3e8] {
            let r = Real::from_f32(v as f32).sqrt().unwrap();
            let expected = (Real::from_f32(v as f32).to_f64()).sqrt();
            assert!(close(r, expected, tol), "sqrt({}) = {} expected {}", v, r.to_f64(), expected);
        }
    }

    #[test]
    fn quadratic_never_panics_and_is_stable() {
        assert_eq!(solve_quatratic(ONE, ZERO, ONE), None); // no real roots
        // x² - 3x + 2 = 0 → 1, 2
        assert!(close(solve_quatratic(ONE, -THREE, TWO).unwrap(), 1.0, 1e-6));
        // Linear: 2x - 4 = 0
        assert!(close(solve_quatratic(ZERO, TWO, -FOUR).unwrap(), 2.0, 1e-6));
        // Cancellation-prone: x² + 1e4·x - 1 = 0 → x ≈ 1e-4
        let r = solve_quatratic(ONE, Real::from_f32(1.0e4), -ONE).unwrap();
        assert!(close(r, 9.999999990e-5, 1e-5), "got {}", r.to_f64());
    }

    #[test]
    fn cubic_three_real_roots() {
        // (x-1)(x-2)(x+3) = x³ - 7x + 6 → smallest positive = 1
        let r = solve_cubic(ONE, ZERO, -Real::from_f32(7.0), SIX).unwrap();
        assert!(close(r, 1.0, 1e-5), "got {}", r.to_f64());
        // (x-0.5)(x-4)(x-6) = x³ - 10.5x² + 29x - 12 → 0.5
        let r = solve_cubic(ONE, -Real::from_f32(10.5), Real::from_f32(29.0), -TWELVE).unwrap();
        assert!(close(r, 0.5, 1e-5), "got {}", r.to_f64());
    }

    #[test]
    fn cubic_motion_case() {
        // j/6·t³ + a/2·t² + v·t - s = 0, j=30000, a=0, v=50, s=0.01 (tiny displacement, cancellation-prone)
        let j = Real::from_f32(30000.0);
        let r = solve_cubic(j * SIXTH, ZERO, Real::from_f32(50.0), -Real::from_f32(0.01)).unwrap();
        let t = r.to_f64();
        let resid = 5000.0 * t * t * t + 50.0 * t - 0.01;
        assert!(t > 0.0 && resid.abs() < 1e-6, "t={} resid={}", t, resid);
    }
}

#[cfg(test)]
mod test {
    use crate as hwa;
    use hwa::math;
    use math::CoordSel;
    use math::Real;
    use math::TVector;

    #[test]
    fn test_weird_cases() {
        let real: Option<Real> = hwa::make_optional_real!();
        assert_eq!(real, None);
    }

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
        assert_eq!(zero, math::ZERO);
        assert_eq!(1i64, one.to_i64().unwrap());
        assert_eq!(1i32, one.rdp(0).to_i32().unwrap());
        assert_eq!(1i64, one.int());
        assert_eq!(1f64, one.to_f64());
        assert_eq!(
            2i32,
            (math::ONE + math::ONE + math::ONE + math::ONE)
                .sqrt()
                .unwrap()
                .to_i32()
                .unwrap()
        );
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
        // micromath (f64 backend) returns ~7.6e-20, not an exact zero
        assert!(math::ONE.acos().abs() < Real::from_f32(1.0e-6), "acos(1.0) is zero");

        assert_eq!(math::ONE.ln().exp(), math::ONE, "exp(ln(1)) is ONE");

        assert_eq!(Real::from_f32(180.0f32), math::PI.r2d());
        assert_eq!(math::PI, Real::from_f32(180.0f32).d2r());
        assert_eq!(Real::from_f32(180.0f32).sign(), math::ONE);
        assert_eq!((math::PI / math::FOUR).tan().rdp(0), math::ONE);
        assert!(
            (math::HALF.atan2(math::HALF) - (math::PI / math::FOUR))
                .rdp(4)
                .is_zero()
        );

        let _one: TVector<f32> = TVector::one();
        let _zero: TVector<f32> = TVector::zero();

        let _two = math::TWO.to_f64() as f32;

        cfg_if::cfg_if! {
            if #[cfg(feature="with-x-axis")] {
                let four_four = TVector::new_with_coord(CoordSel::X.union(CoordSel::Y), Some(4.0f32));
                let two_two = TVector::new_with_coord(CoordSel::X.union(CoordSel::Y), Some(2.0f32));
                assert_eq!(four_four.sqrt().rdp(6), two_two);
            }
        }
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
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
        assert!(zero.bounded_by(&empty));
        assert!(one.bounded_by(&empty));

        assert!(zero.is_nan_or_zero());
        assert!(empty.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    #[test]
    fn test_vector_i32() {
        use crate as printhor_hwa_common;
        use crate::math::TVector;

        let zero: TVector<i32> = hwa::make_vector!(x = 0, y = 0, z = 0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(
            relevant_coords,
            zero.negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_nan_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            zero.not_nan_coords().intersection(relevant_coords)
        );

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    #[test]
    fn test_vector_f32() {
        use crate as printhor_hwa_common;
        use crate::math::TVector;

        let zero: TVector<f32> = hwa::make_vector!(x = 0.0, y = 0.0, z = 0.0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1.0));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1.0));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(
            relevant_coords,
            zero.negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_nan_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            zero.not_nan_coords().intersection(relevant_coords)
        );

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    #[test]
    fn test_vector_u32() {
        use crate as printhor_hwa_common;
        use crate::math::TVector;

        let zero: TVector<u32> = hwa::make_vector!(x = 0, y = 0, z = 0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(
            relevant_coords,
            zero.negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_nan_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            zero.not_nan_coords().intersection(relevant_coords)
        );

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    #[test]
    fn test_vector_u64() {
        use crate as printhor_hwa_common;
        use crate::math::TVector;

        let zero: TVector<u64> = hwa::make_vector!(x = 0, y = 0, z = 0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(
            relevant_coords,
            zero.negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_nan_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            zero.not_nan_coords().intersection(relevant_coords)
        );

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    #[test]
    fn test_vector_u16() {
        use crate as printhor_hwa_common;
        use crate::math::TVector;

        let zero: TVector<u16> = hwa::make_vector!(x = 0, y = 0, z = 0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(
            relevant_coords,
            zero.negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_nan_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            zero.not_nan_coords().intersection(relevant_coords)
        );

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    #[test]
    fn test_vector_u8() {
        use crate as printhor_hwa_common;
        use crate::math::TVector;

        let zero: TVector<u8> = hwa::make_vector!(x = 0, y = 0, z = 0);

        let mut one = zero;
        one.apply_values(|_c, _x| Some(1));

        let other_one = one.with_coord_if_set(CoordSel::all_axis(), Some(1));
        assert_eq!(one, other_one);
        assert!(zero.is_nan_or_zero());
        assert!(!one.is_nan_or_zero());

        let relevant_coords = CoordSel::X.union(CoordSel::Y).union(CoordSel::Z);

        assert_eq!(
            relevant_coords,
            zero.negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_negligible_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            one.not_nan_coords().intersection(relevant_coords)
        );
        assert_eq!(
            relevant_coords,
            zero.not_nan_coords().intersection(relevant_coords)
        );

        assert_eq!(zero.abs(), zero);
        assert_eq!(one.abs(), one);
    }

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    #[test]
    fn test_vector_real() {
        let zero = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 0.0);
        assert_eq!(zero, zero.ceil());

        let zero = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 0.0);
        assert_eq!(zero, zero.floor());
    }

    #[cfg(feature = "with-x-axis")]
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

        let one = TVector::one();
        let two = one.copy_with_coords(CoordSel::all_axis(), Some(math::TWO));
        assert_eq!(one + one, two);
        let empty = TVector::new_with_coord(CoordSel::empty(), Some(math::ONE));

        assert_eq!(one + empty, empty);

        #[allow(unused_mut)]
        let mut t2 = empty;
        #[cfg(feature = "with-e-axis")]
        t2.set_coord(CoordSel::E, Some(math::ONE));
        #[cfg(feature = "with-e-axis")]
        assert_ne!(t2, empty);
        #[cfg(feature = "with-e-axis")]
        t2.set_coord(CoordSel::E, None);
        assert_eq!(t2, empty);

        assert!(TVector::<Real>::new().is_nan_or_zero());
        #[cfg(feature = "with-e-axis")]
        assert!(!hwa::make_vector!(e = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-x-axis")]
        assert!(!hwa::make_vector!(x = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-y-axis")]
        assert!(!hwa::make_vector!(y = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-z-axis")]
        assert!(!hwa::make_vector!(z = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-a-axis")]
        assert!(!hwa::make_vector!(a = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-b-axis")]
        assert!(!hwa::make_vector!(b = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-c-axis")]
        assert!(!hwa::make_vector!(c = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-i-axis")]
        assert!(!hwa::make_vector!(i = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-j-axis")]
        assert!(!hwa::make_vector!(j = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-k-axis")]
        assert!(!hwa::make_vector!(k = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-u-axis")]
        assert!(!hwa::make_vector!(u = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-v-axis")]
        assert!(!hwa::make_vector!(v = 1.0).is_nan_or_zero());
        #[cfg(feature = "with-w-axis")]
        assert!(!hwa::make_vector!(w = 1.0).is_nan_or_zero());

        assert_eq!(TVector::new_with_const_value(1.0), TVector::one());
        assert_eq!(TVector::new().map_nan(&Real::one()), TVector::one());

        assert_eq!(
            TVector::new().map_coords(CoordSel::all(), &Some(math::ONE)),
            TVector::one()
        );
        assert_eq!(
            TVector::new().map_coords(CoordSel::empty(), &Some(math::ONE)),
            TVector::new()
        );

        assert_eq!(
            TVector::<Real>::one().selecting(CoordSel::all()),
            TVector::one()
        );
        assert_eq!(
            TVector::<Real>::one().selecting(CoordSel::empty()),
            TVector::new()
        );

        assert_eq!(
            TVector::<Real>::one().selecting_negligible(CoordSel::all()),
            TVector::new()
        );
        assert_eq!(
            TVector::<Real>::zero().selecting_negligible(CoordSel::all()),
            TVector::zero()
        );
        assert_eq!(
            TVector::<Real>::new().selecting_negligible(CoordSel::all()),
            TVector::new()
        );

        assert_eq!(
            TVector::<Real>::one().excluding(CoordSel::all()),
            TVector::new()
        );
        assert_eq!(
            TVector::<Real>::one().excluding(CoordSel::empty()),
            TVector::one()
        );

        assert_eq!(
            TVector::<Real>::one().excluding_negligible(),
            TVector::one()
        );
        assert_eq!(
            TVector::<Real>::zero().excluding_negligible(),
            TVector::new()
        );
        assert_eq!(
            TVector::<Real>::new().excluding_negligible(),
            TVector::new()
        );

        assert_eq!(
            TVector::new().map_nan_coords(CoordSel::all(), &math::ONE),
            TVector::one()
        );
        assert_eq!(
            TVector::new().map_nan_coords(CoordSel::empty(), &math::ONE),
            TVector::new()
        );

        assert_eq!(TVector::<Real>::new().map_val(&math::ONE), TVector::new());
        assert_eq!(TVector::<Real>::one().map_val(&math::ZERO), TVector::zero());

        assert_eq!(
            TVector::<Real>::one().sum(),
            Real::from_lit(CoordSel::num_axis() as i64, 0)
        );

        assert_eq!(TVector::<Real>::one().pow(1), TVector::<Real>::one());

        let num_axis = Real::from_lit(CoordSel::num_axis() as i64, 0);

        // Produce a vector (2, 2, 2, ...). Compute the unit vector. Then ensure its magnitude is practically 1
        assert_eq!(TVector::<Real>::new().norm2(), Some(math::ZERO));
        let vec_two = TVector::one().with_coord_if_set(CoordSel::all(), Some(math::TWO));
        let vec_two_unit = vec_two.unit();
        let vec_two_unit_magnitude = vec_two_unit.norm2().unwrap();
        // Relaxed sqrt precision because rust decimal uses Newton-Raphson iterative method
        assert_eq!(vec_two_unit_magnitude.rdp(6), math::ONE);
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

        #[cfg(feature = "with-e-axis")]
        assert_eq!(one.get_coord(CoordSel::E), Some(math::ONE));
        #[cfg(feature = "with-x-axis")]
        assert_eq!(one.get_coord(CoordSel::X), Some(math::ONE));
        #[cfg(feature = "with-y-axis")]
        assert_eq!(one.get_coord(CoordSel::Y), Some(math::ONE));
        #[cfg(feature = "with-z-axis")]
        assert_eq!(one.get_coord(CoordSel::Z), Some(math::ONE));
        #[cfg(feature = "with-a-axis")]
        assert_eq!(one.get_coord(CoordSel::A), Some(math::ONE));
        #[cfg(feature = "with-b-axis")]
        assert_eq!(one.get_coord(CoordSel::B), Some(math::ONE));
        #[cfg(feature = "with-c-axis")]
        assert_eq!(one.get_coord(CoordSel::C), Some(math::ONE));
        #[cfg(feature = "with-i-axis")]
        assert_eq!(one.get_coord(CoordSel::I), Some(math::ONE));
        #[cfg(feature = "with-j-axis")]
        assert_eq!(one.get_coord(CoordSel::J), Some(math::ONE));
        #[cfg(feature = "with-k-axis")]
        assert_eq!(one.get_coord(CoordSel::K), Some(math::ONE));
        #[cfg(feature = "with-u-axis")]
        assert_eq!(one.get_coord(CoordSel::U), Some(math::ONE));
        #[cfg(feature = "with-v-axis")]
        assert_eq!(one.get_coord(CoordSel::V), Some(math::ONE));
        #[cfg(feature = "with-w-axis")]
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
        #[cfg(feature = "with-e-axis")]
        let two: TVector<Real> = one + one;

        #[cfg(feature = "with-e-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::E, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-e-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::E, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-x-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::X, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-x-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::X, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-y-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::Y, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-y-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::Y, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-z-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::Z, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-z-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::Z, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-a-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::A, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-a-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::A, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-b-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::B, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-b-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::B, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-c-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::C, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-c-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::C, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-i-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::I, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-i-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::I, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-j-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::J, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-j-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::J, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-k-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::K, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-k-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::K, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-u-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::U, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-u-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::U, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-v-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::V, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-v-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::V, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );
        #[cfg(feature = "with-w-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::W, Some(math::ONE)).vmax(),
            Some(math::TWO)
        );
        #[cfg(feature = "with-w-axis")]
        assert_eq!(
            two.copy_with_coords(CoordSel::W, Some(math::ONE)).vmin(),
            Some(math::ONE)
        );

        let (_, _) = one.decompose_normal();
    }
}

#[cfg(test)]
mod coverage_tests;
