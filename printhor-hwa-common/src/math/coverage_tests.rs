//! Boundary-case tests for the numeric backends (`Real`) and the polynomial solvers.
//!
//! Every test compiles and runs with the three backends (`float-point-f32-impl`,
//! `float-point-f64-impl` and `fixed-point-128-impl`). Backend-specific expectations are gated
//! with `cfg` (mirroring the backend precedence of `math/mod.rs`) and tolerances are chosen per
//! backend:
//! - f32: exact `std` transcendental functions on the host, ~1e-7 relative precision.
//! - f64: f64 arithmetic, but `sin`/`cos`/`atan2`/`acos`/`ln`/`exp` are micromath (f32)
//!   approximations (atan2 ≤ 2e-3 rad, acos ≤ 3e-2 rad).
//! - fixed point: 28 decimal digits; `atan2`/`acos` are native Decimal implementations.
use super::*;
use core::cmp::Ordering;

/// Builds a `Real` from an `f64` literal at the best precision the backend offers.
fn r(v: f64) -> Real {
    cfg_if::cfg_if! {
        if #[cfg(feature = "fixed-point-128-impl")] {
            use rust_decimal::prelude::FromPrimitive;
            Real::from_fixed(rust_decimal::Decimal::from_f64(v).unwrap())
        } else if #[cfg(feature = "float-point-f64-impl")] {
            Real::from_f64(v)
        } else {
            Real::from_f32(v as f32)
        }
    }
}

/// Picks the tolerance of the active backend.
#[allow(unused_variables)]
const fn pick(f32_tol: f64, f64_tol: f64, fixed_tol: f64) -> f64 {
    cfg_if::cfg_if! {
        if #[cfg(feature = "fixed-point-128-impl")] {
            fixed_tol
        } else if #[cfg(feature = "float-point-f64-impl")] {
            f64_tol
        } else {
            f32_tol
        }
    }
}

#[track_caller]
fn assert_abs(label: &str, got: Real, expected: f64, tol: f64) {
    let g = got.to_f64();
    assert!(
        (g - expected).abs() <= tol,
        "{}: got {} expected {} (abs tol {})",
        label,
        g,
        expected,
        tol
    );
}

#[track_caller]
fn assert_rel(label: &str, got: Real, expected: f64, tol: f64) {
    let g = got.to_f64();
    assert!(
        (g - expected).abs() <= tol * expected.abs(),
        "{}: got {} expected {} (rel tol {})",
        label,
        g,
        expected,
        tol
    );
}

//#region "Real: ordering, selection and accessors"

#[test]
fn ord_min_max_clamp_pick_the_right_operand() {
    let lo = r(-2.5);
    let hi = r(1.25);
    let hi_again = r(1.25);

    assert_eq!(lo.min(hi), lo);
    assert_eq!(hi.min(lo), lo);
    assert_eq!(lo.max(hi), hi);
    assert_eq!(hi.max(lo), hi);
    assert_eq!(hi.min(hi_again), hi);
    assert_eq!(core::cmp::min(hi, lo), lo);
    assert_eq!(core::cmp::max(lo, hi), hi);

    // Clamp bounds are inclusive and values inside are untouched
    assert_eq!(lo.clamp(lo, hi), lo);
    assert_eq!(hi.clamp(lo, hi), hi);
    assert_eq!(r(-10.0).clamp(lo, hi), lo);
    assert_eq!(r(10.0).clamp(lo, hi), hi);
    assert_eq!(ZERO.clamp(lo, hi), ZERO);

    // `Ord` and `PartialOrd` agree on ordinary values
    assert_eq!(lo.cmp(&hi), Ordering::Less);
    assert_eq!(hi.cmp(&lo), Ordering::Greater);
    assert_eq!(hi.cmp(&hi_again), Ordering::Equal);
    assert_eq!(lo.partial_cmp(&hi), Some(Ordering::Less));
    assert!(lo < hi && lo <= hi && hi > lo && hi >= lo && hi <= hi_again && hi >= hi_again);
}

#[test]
fn optional_vmin_vmax_pick_the_extreme_defined_value() {
    let lo = r(-1.0);
    let hi = r(3.0);
    assert_eq!(Real::vmin(Some(hi), Some(lo)), Some(lo));
    assert_eq!(Real::vmin(Some(lo), Some(hi)), Some(lo));
    assert_eq!(Real::vmin(Some(hi), Some(hi)), Some(hi));
    assert_eq!(Real::vmax(Some(lo), Some(hi)), Some(hi));
    assert_eq!(Real::vmax(Some(hi), Some(lo)), Some(hi));
    assert_eq!(Real::vmax(Some(lo), Some(lo)), Some(lo));
    assert_eq!(Real::vmin(Some(lo), None), Some(lo));
    assert_eq!(Real::vmax(None, None), None);
}

#[test]
fn literal_constructors_and_inner_value() {
    assert_abs("from_lit(-12345, 3)", Real::from_lit(-12345, 3), -12.345, 1e-6);
    assert_eq!(Real::new(5, 1), HALF);
    assert_eq!(Real::from_lit(0, 7), ZERO);
    // Scales beyond the exact power-of-ten tables of the float backends (f32: 10, f64: 22)
    assert_rel("from_lit(3, 24)", Real::from_lit(3, 24), 3.0e-24, 1e-6);
    // `inner()` exposes the backend representation unchanged
    let v = Real::from_lit(15, 1);
    assert!(v.inner() == v.0);
    assert_eq!(Real::from_lit(-15, 1).abs(), v);
}

//#endregion

//#region "Real: predicates, signs and integer conversions"

#[test]
fn sign_and_positivity_predicates_at_the_epsilon_boundary() {
    let eps = Real::epsilon();
    let half_eps = eps / TWO;

    assert_eq!(r(42.0).sign(), ONE);
    assert_eq!(r(-0.001).sign(), -ONE);
    // Zero is mapped to +1 by every backend
    assert_eq!(ZERO.sign(), ONE);

    assert!(eps.is_defined_positive());
    assert!(!ZERO.is_defined_positive());
    assert!(!(-eps).is_defined_positive());

    assert!(ZERO.is_positive());
    assert!(eps.is_positive());
    assert!(!(-eps).is_positive());

    // Negligible means strictly below epsilon in magnitude, on both sides of zero
    assert!(ZERO.is_negligible());
    assert!(half_eps.is_negligible());
    assert!((-half_eps).is_negligible());
    assert!(!eps.is_negligible());
    assert!(!(-eps).is_negligible());

    assert!(ZERO.is_zero());
    assert!(!eps.is_zero());
}

#[test]
fn integer_conversions_of_integral_and_out_of_range_values() {
    assert_eq!(r(-7.0).int(), -7);
    assert_eq!(r(-7.0).to_i32(), Some(-7));
    assert_eq!(r(123456.0).to_i64(), Some(123456));
    assert_eq!(ZERO.to_i32(), Some(0));
    // i32 overflow is reported as None (no wrapping, no saturation)
    assert_eq!(r(3.0e9).to_i32(), None);
    assert_eq!(r(-3.0e9).to_i32(), None);
    assert_eq!(r(3.0e9).to_i64(), Some(3_000_000_000));
    // `int()` truncates toward zero in every backend
    assert_eq!(r(2.75).int(), 2);
    assert_eq!(r(-2.75).int(), -2);
}

#[test]
#[cfg(not(feature = "fixed-point-128-impl"))]
fn float_integer_conversions_reject_nan_and_infinity() {
    type F = RealImpl;
    assert_eq!(Real(F::NAN).to_i32(), None);
    assert_eq!(Real(F::INFINITY).to_i64(), None);
    assert_eq!(Real(F::NEG_INFINITY).to_i32(), None);
}

/// `to_i32`/`to_i64` feed step counts and timer widths. In the float backends they truncate
/// toward zero (`num_traits` semantics, same as `int()`), but the fixed point backend rounds
/// first, so the same value yields a different integer depending on the backend.
#[test]
fn integer_conversions_truncate_toward_zero_in_every_backend() {
    for &(v, expected) in &[(2.75, 2i64), (-2.75, -2), (2.5, 2), (0.999, 0), (-0.999, 0)] {
        assert_eq!(r(v).int(), expected, "int({})", v);
        assert_eq!(r(v).to_i64(), Some(expected), "to_i64({})", v);
        assert_eq!(r(v).to_i32(), Some(expected as i32), "to_i32({})", v);
    }
}

/// `round()`/`round_dp()` (and `TVector::round()`, used to compute step counts) should give the
/// same result in every backend. Float backends round half away from zero; the fixed point
/// backend inherits `rust_decimal`'s banker's rounding (half to even).
#[test]
fn rounding_of_ties_is_consistent_across_backends() {
    assert_eq!(r(2.5).round(), r(3.0));
    assert_eq!(r(-2.5).round(), r(-3.0));
    assert_eq!(r(0.5).round(), ONE);
    assert_eq!(r(0.25).round_dp(1), r(0.3));
    assert_eq!(r(2.5).rdp(0), r(3.0));
}

#[test]
fn rounding_floor_and_ceil_of_non_ties() {
    assert_eq!(r(2.4).round(), TWO);
    assert_eq!(r(-2.6).round(), -THREE);
    assert_eq!(r(1.5).floor(), ONE);
    assert_eq!(r(-1.5).floor(), -TWO);
    assert_eq!(r(1.5).ceil(), TWO);
    assert_eq!(r(-1.5).ceil(), -ONE);
    assert_abs("round_dp(1.23456, 2)", r(1.23456).round_dp(2), 1.23, 1e-6);
    assert_abs("rdp(-1.23456, 3)", r(-1.23456).rdp(3), -1.235, 1e-6);
}

//#endregion

//#region "Real: powers and roots"

#[test]
fn powi_and_recip_exact_values() {
    assert_eq!(TWO.powi(-2), QUARTER);
    assert_eq!((-THREE).powi(3), -TWENTY_SEVEN);
    assert_eq!(r(123.0).powi(0), ONE);
    assert_eq!(FOUR.recip(), QUARTER);
    assert_eq!((-HALF).recip(), -TWO);
}

#[test]
fn sqrt_domain_and_tiny_values() {
    assert_eq!(r(-1.0).sqrt(), None);
    assert_eq!((-Real::epsilon()).sqrt(), None);
    assert_eq!(ZERO.sqrt(), Some(ZERO));
    assert_eq!(FOUR.sqrt(), Some(TWO));
    // Below epsilon: still accurate
    assert_rel("sqrt(1e-20)", r(1.0e-20).sqrt().unwrap(), 1.0e-10, pick(1e-6, 1e-14, 1e-12));
    assert_rel("sqrt(eps)", Real::epsilon().sqrt().unwrap(), Real::epsilon().to_f64().sqrt(), pick(1e-6, 1e-14, 1e-12));
}

#[test]
fn sqrt_relative_accuracy_over_many_decades() {
    let (lo, hi, tol) = {
        cfg_if::cfg_if! {
            if #[cfg(feature = "fixed-point-128-impl")] {
                (-14, 14, 1e-12)
            } else if #[cfg(feature = "float-point-f64-impl")] {
                (-300, 300, 4e-16)
            } else {
                (-37, 37, 2.4e-7)
            }
        }
    };
    for e in (lo..=hi).step_by(1) {
        for m in [1.0f64, 2.0, 7.5] {
            let x = r(m * 10f64.powi(e));
            let expected = x.to_f64().sqrt();
            assert_rel("sqrt", x.sqrt().unwrap(), expected, tol);
        }
    }
}

/// The largest finite value must not overflow inside the Heron correction step.
#[test]
#[cfg(not(feature = "fixed-point-128-impl"))]
fn sqrt_of_the_largest_finite_value() {
    type F = RealImpl;
    let s = Real(F::MAX).sqrt().unwrap();
    assert!(s.inner().is_finite(), "sqrt(MAX) = {:?}", s);
    assert_rel("sqrt(MAX)", s, (F::MAX as f64).sqrt(), pick(2.4e-7, 4e-16, 0.0));
}

/// Subnormal inputs defeat the magic-constant seed of the fast inverse square root (it assumes
/// a normalised exponent) and the fixed number of Newton steps does not fully recover: the
/// result is ~1e-3 (f32) / ~3e-6 (f64) off, instead of ≤ 1 ulp as for normal inputs.
#[test]
#[cfg(not(feature = "fixed-point-128-impl"))]
fn sqrt_of_subnormal_values() {
    type F = RealImpl;
    for x in [F::MIN_POSITIVE / 8.0, F::MIN_POSITIVE / 1024.0] {
        let s = Real(x).sqrt().unwrap();
        assert_rel("sqrt(subnormal)", s, (x as f64).sqrt(), 1e-6);
    }
}

#[test]
fn cbrt_of_exact_cubes_and_signs() {
    let tol = pick(2e-6, 1e-14, 1e-12);
    for &(x, expected) in &[
        (8.0, 2.0),
        (-27.0, -3.0),
        (1.0, 1.0),
        (-1.0, -1.0),
        (3.375, 1.5),
        (0.001, 0.1),
        (1.0e-6, 1.0e-2),
        (1.0e6, 1.0e2),
        (-1.0e9, -1.0e3),
    ] {
        assert_rel("cbrt", r(x).cbrt(), expected, tol);
    }
    assert_eq!(ZERO.cbrt(), ZERO);
}

#[test]
#[cfg(not(feature = "fixed-point-128-impl"))]
fn libm_cbrt_special_values() {
    type F = RealImpl;
    assert!(Real::libm_cbrt(F::NAN).is_nan());
    assert_eq!(Real::libm_cbrt(F::INFINITY), F::INFINITY);
    assert_eq!(Real::libm_cbrt(F::NEG_INFINITY), F::NEG_INFINITY);

    // cbrt(±0) is ±0 (sign preserved)
    let pz = Real::libm_cbrt(0.0);
    assert!(pz == 0.0 && pz.is_sign_positive());
    let nz = Real::libm_cbrt(-0.0);
    assert!(nz == 0.0 && nz.is_sign_negative());
}

/// `cbrt` must be accurate for every finite magnitude, subnormals included. The f32 port of
/// libm's `cbrtf` performs its two Newton steps in f32 (libm uses f64 there): `t·(2x + t³)`
/// underflows to 0 for |x| < ~1e-35 and overflows to inf (→ NaN) for |x| > ~1e28.
#[test]
#[cfg(not(feature = "fixed-point-128-impl"))]
fn libm_cbrt_is_accurate_over_the_whole_finite_range() {
    type F = RealImpl;
    let (lo, hi, tol) = {
        cfg_if::cfg_if! {
            if #[cfg(feature = "float-point-f64-impl")] {
                (-320, 307, 1e-14)
            } else {
                (-44, 38, 2e-6)
            }
        }
    };
    for e in lo..=hi {
        for m in [1.0f64, 3.3] {
            for sign in [1.0f64, -1.0] {
                let x = (sign * m * 10f64.powi(e)) as F;
                if x == 0.0 || !x.is_finite() {
                    continue;
                }
                let got = Real::libm_cbrt(x) as f64;
                let expected = (x as f64).cbrt();
                assert!(
                    (got - expected).abs() <= tol * expected.abs(),
                    "cbrt({:e}) = {:e} expected {:e}",
                    x,
                    got,
                    expected
                );
            }
        }
    }
}

/// Negative zero (e.g. `-ZERO`, or the negation of a zero coordinate) is not a negative number.
#[test]
fn sqrt_of_negative_zero_is_zero() {
    assert_eq!((-ZERO).sqrt(), Some(ZERO));
    assert_eq!((-TVector::<Real>::zero()).norm2(), Some(ZERO));
}

//#endregion

//#region "Real: transcendental functions"

#[test]
fn inverse_trigonometry_quadrants_and_domain_edges() {
    let pi = core::f64::consts::PI;
    let acos_tol = pick(1e-6, 3e-2, 1e-9);
    assert_abs("acos(1)", ONE.acos(), 0.0, acos_tol);
    assert_abs("acos(-1)", (-ONE).acos(), pi, acos_tol);
    assert_abs("acos(0)", ZERO.acos(), pi / 2.0, acos_tol);
    assert_abs("acos(0.5)", HALF.acos(), pi / 3.0, acos_tol);
    assert_abs("acos(-0.5)", (-HALF).acos(), 2.0 * pi / 3.0, acos_tol);

    // micromath's atan2 (f64 backend) is documented at 2e-3 rad but reaches ~2.7e-3 (atan2(2, 1))
    let atan_tol = pick(1e-6, 3e-3, 1e-9);
    for &(y, x, expected) in &[
        (1.0, 1.0, pi / 4.0),
        (1.0, -1.0, 3.0 * pi / 4.0),
        (-1.0, -1.0, -3.0 * pi / 4.0),
        (-1.0, 1.0, -pi / 4.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, pi / 2.0),
        (-1.0, 0.0, -pi / 2.0),
        (2.0, 1.0, 2.0f64.atan()),
    ] {
        assert_abs("atan2", r(y).atan2(r(x)), expected, atan_tol);
    }
    // The negative real axis is the branch cut: ±π are both valid
    let a = ZERO.atan2(-ONE).to_f64().abs();
    assert!((a - pi).abs() <= atan_tol, "|atan2(0, -1)| = {}", a);
}

/// `atan2(0, 0)` is 0 by convention (std, f32 and fixed point backends). The f64 backend
/// delegates to micromath, whose approximation divides 0/0 there and returns NaN.
#[test]
fn atan2_at_the_origin_is_zero() {
    assert_eq!(ZERO.atan2(ZERO), ZERO);
    assert_eq!((-ZERO).atan2(ZERO), ZERO);
}

/// Out of [-1, 1] acos is documented to return NaN: true for the float backends.
#[test]
#[cfg(not(feature = "fixed-point-128-impl"))]
fn acos_outside_the_unit_interval_is_nan() {
    assert!(r(1.5).acos().inner().is_nan());
    assert!(r(-1.0001).acos().inner().is_nan());
}

/// The fixed point backend cannot represent NaN: arguments outside [-1, 1] (e.g. a dot product of
/// unit vectors rounded up) are clamped to the unit interval instead.
#[test]
#[cfg(feature = "fixed-point-128-impl")]
fn acos_outside_the_unit_interval_is_clamped() {
    let slightly_above_one = ONE + Real::epsilon();
    let outcome = std::panic::catch_unwind(|| slightly_above_one.acos());
    assert!(outcome.is_ok(), "acos(1 + eps) panicked");
    assert!(outcome.unwrap().to_f64().abs() < 1e-12);
    assert!(((-r(1.5)).acos().to_f64() - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn direct_trigonometry_and_angle_conversions() {
    let pi = core::f64::consts::PI;
    let tol = pick(1e-6, 2e-3, 1e-9);
    assert_abs("sin(pi/2)", (PI / TWO).sin(), 1.0, tol);
    assert_abs("sin(-pi/2)", (-PI / TWO).sin(), -1.0, tol);
    assert_abs("cos(pi)", PI.cos(), -1.0, tol);
    assert_abs("cos(pi/3)", (PI / THREE).cos(), 0.5, tol);
    // rust_decimal's tan is less precise than its sin/cos (~4e-9 at -pi/4)
    assert_abs("tan(-pi/4)", (-PI / FOUR).tan(), -1.0, pick(1e-6, 2e-3, 1e-8));

    assert_abs("d2r(90)", r(90.0).d2r(), pi / 2.0, 1e-6);
    assert_abs("d2r(-45)", r(-45.0).d2r(), -pi / 4.0, 1e-6);
    assert_abs("r2d(-pi)", (-PI).r2d(), -180.0, 1e-4);
    assert_abs("r2d(d2r(12.345))", r(12.345).d2r().r2d(), 12.345, 1e-4);
}

#[test]
fn exponential_and_logarithm() {
    let e = core::f64::consts::E;
    let tol = pick(1e-6, 5e-3, 1e-9);
    assert_abs("exp(0)", ZERO.exp(), 1.0, tol);
    assert_abs("ln(1)", ONE.ln(), 0.0, tol);
    assert_rel("exp(1)", ONE.exp(), e, tol);
    assert_rel("exp(-1)", (-ONE).exp(), 1.0 / e, tol);
    assert_rel("ln(10)", TEN.ln(), 10f64.ln(), tol);
    assert_rel("ln(0.5)", HALF.ln(), 0.5f64.ln(), tol);
    assert_rel("exp(ln(7))", SEVEN.ln().exp(), 7.0, tol);
}

//#endregion

//#region "Polynomial solvers"

#[test]
fn quadratic_degenerate_cases_and_alias() {
    // 0·x² + 0·x + c: no isolated root, whatever c is
    assert_eq!(solve_quatratic(ZERO, ZERO, ONE), None);
    assert_eq!(solve_quatratic(ZERO, ZERO, ZERO), None);
    // Linear with a negative root, and with the root exactly at 0 (not strictly positive)
    assert_eq!(solve_quatratic(ZERO, TWO, FOUR), None);
    assert_eq!(solve_quatratic(ZERO, TWO, ZERO), None);
    // Both roots negative: (x + 1)(x + 2)
    assert_eq!(solve_quadratic(ONE, THREE, TWO), None);
    // Double root: (x - 1)²
    assert_abs("double root", solve_quadratic(ONE, -TWO, ONE).unwrap(), 1.0, 1e-6);
    // Roots of opposite sign: (x - 1)(x + 2) = x² + x - 2
    assert_abs("mixed signs", solve_quadratic(ONE, ONE, -TWO).unwrap(), 1.0, 1e-6);
    // Negative leading coefficient: -(x - 2)(x - 3) = -x² + 5x - 6
    assert_abs("a < 0", solve_quadratic(-ONE, FIVE, -SIX).unwrap(), 2.0, 1e-6);

    // The correctly spelled alias is the same function
    for &(a, b, c) in &[(1.0, -3.0, 2.0), (0.0, 2.0, -4.0), (1.0, 0.0, 1.0), (2.0, 7.0, -4.0)] {
        assert_eq!(
            solve_quadratic(r(a), r(b), r(c)),
            solve_quatratic(r(a), r(b), r(c))
        );
    }
}

#[test]
fn cubic_with_negligible_leading_coefficient_is_solved_as_quadratic() {
    // 0·x³ + x² - 3x + 2 = (x - 1)(x - 2)
    assert_abs("a = 0", solve_cubic(ZERO, ONE, -THREE, TWO).unwrap(), 1.0, 1e-6);
    assert_eq!(
        solve_cubic(ZERO, ZERO, TWO, -FOUR),
        solve_quatratic(ZERO, TWO, -FOUR)
    );
    #[cfg(not(feature = "fixed-point-128-impl"))]
    {
        // Leading coefficient below epsilon (floats): treated as zero
        let tiny = Real::epsilon() / FOUR;
        assert_abs("a < eps", solve_cubic(tiny, ONE, -THREE, TWO).unwrap(), 1.0, 1e-5);
    }
}

#[test]
fn cubic_with_a_single_real_root() {
    let tol = pick(1e-5, 1e-12, 1e-12);
    // (x - 1)(x² - 6x + 10) = x³ - 7x² + 16x - 10: depressed q = 52/27 ≥ 0
    assert_abs("q >= 0", solve_cubic(ONE, -SEVEN, r(16.0), -TEN).unwrap(), 1.0, tol);
    // (x - 1)(x² + 4x + 5) = x³ + 3x² + x - 5: depressed q < 0
    assert_abs("q < 0", solve_cubic(ONE, THREE, ONE, -FIVE).unwrap(), 1.0, tol);
    // x³ + x + 2 = (x + 1)(x² - x + 2): the only real root is negative
    assert_eq!(solve_cubic(ONE, ZERO, ONE, TWO), None);
    // Scaled polynomial (leading coefficient ≠ 1): 2·(x - 0.5)(x² + 1) = 2x³ - x² + 2x - 1
    assert_abs("scaled", solve_cubic(TWO, -ONE, TWO, -ONE).unwrap(), 0.5, tol);
}

/// A triple root makes the depressed cubic `t³ = 0` (p = q = 0). With rounding, `p` may come
/// out as a tiny *positive* number while the discriminant underflows to zero: the solver then
/// takes the three-real-roots branch, `sqrt(-p/3)` fails and the whole call returns `None`.
/// This is what happens with the fixed point backend (1/3 is not exact in base 10).
#[test]
fn cubic_with_a_triple_root() {
    let tol = pick(1e-5, 1e-12, 1e-12);
    // (x - 1)³ = x³ - 3x² + 3x - 1
    assert_abs("(x-1)^3", solve_cubic(ONE, -THREE, THREE, -ONE).unwrap(), 1.0, tol);
    // (x - 2)³ = x³ - 6x² + 12x - 8
    assert_abs("(x-2)^3", solve_cubic(ONE, -SIX, TWELVE, -EIGHT).unwrap(), 2.0, tol);
}

#[test]
fn cubic_with_three_real_roots_picks_the_smallest_positive() {
    let tol = pick(1e-5, 1e-12, 1e-12);
    // (x + 1)(x - 2)(x - 5) = x³ - 6x² + 3x + 10 → 2
    assert_abs("one negative root", solve_cubic(ONE, -SIX, THREE, TEN).unwrap(), 2.0, tol);
    // (x + 1)(x + 2)(x + 3): no positive root
    assert_eq!(solve_cubic(ONE, SIX, r(11.0), SIX), None);
    // x(x - 1)(x - 2) = x³ - 3x² + 2x: the root at 0 is not strictly positive → 1
    assert_abs("root at zero", solve_cubic(ONE, -THREE, TWO, ZERO).unwrap(), 1.0, tol);
}

//#endregion

