//! Boundary-case tests for `CoordSel` and `TVector`.
//!
//! The tests iterate over `CoordSel::all_axis()` so they exercise every axis the crate is
//! compiled with (all thirteen with `with-all-axis`, fewer with the firmware feature sets) and
//! every numeric backend.
use super::*;
use crate as hwa;
use hwa::math;
use std::string::String;
use std::vec::Vec;

fn axes() -> Vec<CoordSel> {
    CoordSel::all_axis().iter().collect()
}

fn int(i: i64) -> Real {
    Real::from_lit(i, 0)
}

/// Builds a vector assigning `f(position, axis)` to every axis.
fn vector_from(f: impl Fn(usize, CoordSel) -> Option<Real>) -> TVector<Real> {
    let mut t = TVector::new();
    for (i, c) in axes().into_iter().enumerate() {
        t.set_coord(c, f(i, c));
    }
    t
}

/// A `fmt::Write` sink that fails once more than `budget` bytes have been written.
struct LimitedWriter {
    out: String,
    budget: usize,
}

impl core::fmt::Write for LimitedWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if s.len() > self.budget {
            self.budget = 0;
            Err(core::fmt::Error)
        } else {
            self.budget -= s.len();
            self.out.push_str(s);
            Ok(())
        }
    }
}

fn write_limited(budget: usize, args: core::fmt::Arguments) -> (core::fmt::Result, String) {
    let mut w = LimitedWriter {
        out: String::new(),
        budget,
    };
    let r = core::fmt::write(&mut w, args);
    (r, w.out)
}

//#region "CoordSel"

#[test]
fn coord_sel_index_is_dense_and_follows_declaration_order() {
    let all = axes();
    assert_eq!(all.len(), CoordSel::num_axis());
    for (i, c) in all.iter().enumerate() {
        assert_eq!(c.index(), i, "index of {}", c.name());
    }
    // A multi-axis selection resolves to its first axis
    assert_eq!(CoordSel::all_axis().index(), 0);
    if let Some(last) = all.last() {
        assert_eq!(last.union(CoordSel::UNSET).index(), all.len() - 1);
    }
    // Empty and flag-only selections fall through to `num_axis()` (one past the last index)
    assert_eq!(CoordSel::empty().index(), CoordSel::num_axis());
    assert_eq!(CoordSel::UNSET.index(), CoordSel::num_axis());
    assert_eq!(CoordSel::ALTERNATE_NAME.index(), CoordSel::num_axis());
}

#[test]
fn all_axis_holds_axes_only() {
    let all = CoordSel::all_axis();
    assert!(!all.contains(CoordSel::UNSET));
    assert!(!all.contains(CoordSel::ALTERNATE_NAME));
    assert!(!all.is_alternate());
    assert!(CoordSel::ALTERNATE_NAME.is_alternate());
    assert_eq!(all.iter().count(), CoordSel::num_axis());
    for c in all.iter() {
        assert_eq!(c.name().len(), 1, "axis names are single letters");
    }
}

#[test]
fn motion_relevant_axis_selects_every_axis_but_the_extruder() {
    let m = CoordSel::motion_relevant_axis();
    assert!(!m.contains(CoordSel::UNSET));
    for c in axes() {
        #[cfg(feature = "with-e-axis")]
        if c == CoordSel::E {
            assert!(!m.contains(c));
            continue;
        }
        assert!(m.contains(c), "{} is motion relevant", c.name());
    }
}

/// `motion_relevant_axis()` is documented as "all motion relevant axis excluding E", but unlike
/// `all_axis()` it does not strip the `ALTERNATE_NAME` flag, so the selection reports
/// `is_alternate() == true` (defmt then prints it with the alternative axis names).
#[test]
fn motion_relevant_axis_carries_no_flags() {
    let m = CoordSel::motion_relevant_axis();
    assert!(!m.is_alternate());
    assert!(!m.contains(CoordSel::ALTERNATE_NAME));
}

#[test]
#[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))]
fn coord_sel_debug_lists_selected_axes_and_propagates_writer_errors() {
    let xy = CoordSel::X | CoordSel::Y;
    assert_eq!(std::format!("{:?}", xy), "X | Y");
    assert_eq!(std::format!("{:?}", CoordSel::X), "X");
    // Flags are not axes: nothing is printed
    assert_eq!(std::format!("{:?}", CoordSel::empty()), "");
    assert_eq!(std::format!("{:?}", CoordSel::UNSET | CoordSel::ALTERNATE_NAME), "");
    // `{:#?}` switches to the alternative (kinematics-specific) names
    assert_eq!(
        std::format!("{:#?}", xy),
        std::format!(
            "{} | {}",
            CoordSel::X.alternative_name(),
            CoordSel::Y.alternative_name()
        )
    );

    // Writer failures are reported, both on the first axis and on the separator
    assert!(write_limited(0, format_args!("{:?}", xy)).0.is_err());
    let (r, partial) = write_limited(1, format_args!("{:?}", xy));
    assert!(r.is_err());
    assert_eq!(partial, "X");
    let (r, full) = write_limited(64, format_args!("{:?}", xy));
    assert!(r.is_ok());
    assert_eq!(full, "X | Y");
}

//#endregion

//#region "TVector: per-axis accessors"

#[test]
fn set_and_get_every_axis_individually() {
    let t = vector_from(|i, _| Some(int(i as i64 + 1)));
    for (i, c) in axes().into_iter().enumerate() {
        assert_eq!(t.get_coord(c), Some(int(i as i64 + 1)), "{}", c.name());
        // Overwriting a single axis leaves the others untouched
        let mut u = t;
        u.set_coord(c, None);
        assert_eq!(u.get_coord(c), None);
        assert_eq!(u.nan_coords(), c);
    }
    // A selection that is not exactly one axis does not name a coordinate
    assert_eq!(t.get_coord(CoordSel::empty()), None);
    assert_eq!(t.get_coord(CoordSel::UNSET), None);
    if CoordSel::num_axis() > 1 {
        assert_eq!(t.get_coord(CoordSel::all_axis()), None);
    }
    // A multi-axis selection writes all of them at once
    let mut u = TVector::<Real>::new();
    u.set_coord(CoordSel::all_axis(), Some(math::ONE));
    assert_eq!(u, TVector::one());
    u.set_coord(CoordSel::all_axis(), None);
    assert_eq!(u, TVector::new());
}

#[test]
fn assign_and_assign_if_set_copy_only_selected_coords() {
    let all = axes();
    // Source defines only the even positions
    let src = vector_from(|i, _| if i % 2 == 0 { Some(int(10 + i as i64)) } else { None });
    // Selection: the first half of the axes
    let selection = all
        .iter()
        .take((all.len() + 1) / 2)
        .fold(CoordSel::empty(), |acc, c| acc.union(*c));

    let mut dst = TVector::new_with_const_value(-math::ONE);
    dst.assign_if_set(selection, &src);
    for (i, c) in all.iter().enumerate() {
        let expected = if selection.contains(*c) && i % 2 == 0 {
            Some(int(10 + i as i64))
        } else {
            Some(-math::ONE)
        };
        assert_eq!(dst.get_coord(*c), expected, "assign_if_set {}", c.name());
    }

    // `assign` also copies the undefined (None) values of the selection
    let mut dst2 = TVector::<Real>::one();
    dst2.assign(selection, &src);
    for (i, c) in all.iter().enumerate() {
        let expected = if selection.contains(*c) {
            src.get_coord(*c)
        } else {
            Some(math::ONE)
        };
        assert_eq!(dst2.get_coord(*c), expected, "assign {} ({})", c.name(), i);
    }

    // Empty selection: no-op
    let mut dst3 = TVector::<Real>::zero();
    dst3.assign_if_set(CoordSel::empty(), &TVector::one());
    assert_eq!(dst3, TVector::zero());
    // Every axis selected and defined: full copy
    dst3.assign_if_set(CoordSel::all_axis(), &TVector::one());
    assert_eq!(dst3, TVector::one());
    // Every axis selected, none defined: nothing changes
    dst3.assign_if_set(CoordSel::all_axis(), &TVector::new());
    assert_eq!(dst3, TVector::one());
}

#[test]
fn decrement_if_positive_only_touches_strictly_positive_defined_coords() {
    for c in axes() {
        let mut t = TVector::<Real>::new();
        t.set_coord(c, Some(math::TWO));
        assert!(t.decrement_if_positive(c), "{} = 2", c.name());
        assert_eq!(t.get_coord(c), Some(math::ONE));
        // Not selected: untouched
        assert!(!t.decrement_if_positive(CoordSel::empty()));
        assert_eq!(t.get_coord(c), Some(math::ONE));

        for v in [Some(math::ZERO), Some(-math::ONE), Some(-Real::epsilon()), None] {
            t.set_coord(c, v);
            assert!(!t.decrement_if_positive(c), "{} = {:?}", c.name(), v);
            assert_eq!(t.get_coord(c), v);
        }

        // A value below one is still decremented (it may become negative)
        t.set_coord(c, Some(math::HALF));
        assert!(t.decrement_if_positive(CoordSel::all_axis()));
        assert_eq!(t.get_coord(c), Some(-math::HALF));
    }
}

#[test]
fn increment_skips_undefined_coords() {
    let mut t = vector_from(|i, _| if i % 2 == 0 { Some(int(i as i64)) } else { None });
    t.increment(CoordSel::all_axis(), math::ONE);
    for (i, c) in axes().into_iter().enumerate() {
        let expected = if i % 2 == 0 { Some(int(i as i64 + 1)) } else { None };
        assert_eq!(t.get_coord(c), expected);
    }
}

#[test]
fn vmax_vmin_scan_every_axis_in_both_orders() {
    let n = CoordSel::num_axis() as i64;
    let ascending = vector_from(|i, _| Some(int(i as i64)));
    assert_eq!(ascending.vmax(), Some(int(n - 1)));
    assert_eq!(ascending.vmin(), Some(int(0)));

    let descending = vector_from(|i, _| Some(int(n - i as i64)));
    assert_eq!(descending.vmax(), Some(int(n)));
    assert_eq!(descending.vmin(), Some(int(1)));

    // Undefined coords are ignored; a single defined coord is both the max and the min
    for (i, c) in axes().into_iter().enumerate() {
        let single = TVector::new_with_coord(c, Some(int(-(i as i64))));
        assert_eq!(single.vmax(), Some(int(-(i as i64))), "{}", c.name());
        assert_eq!(single.vmin(), Some(int(-(i as i64))), "{}", c.name());
    }
}

#[test]
fn bounded_by_checks_every_defined_axis() {
    let bound = TVector::new_with_const_value(math::TWO);
    for c in axes() {
        let mut lhs = TVector::<Real>::one();
        lhs.set_coord(c, Some(math::FIVE));
        assert!(!lhs.bounded_by(&bound), "{} exceeds", c.name());

        // No bound on that axis
        let mut open = bound;
        open.set_coord(c, None);
        assert!(lhs.bounded_by(&open), "{} unbounded", c.name());

        // The bound is inclusive and negative values are always below it
        lhs.set_coord(c, Some(math::TWO));
        assert!(lhs.bounded_by(&bound));
        lhs.set_coord(c, Some(-math::FIVE));
        assert!(lhs.bounded_by(&bound));

        // Undefined left-hand coords are not checked
        lhs.set_coord(c, None);
        assert!(lhs.bounded_by(&TVector::zero().with_coord(CoordSel::all_axis(), Some(math::ONE))));
    }
}

//#endregion

//#region "TVector: mapping and arithmetic"

#[test]
fn sign_maps_positive_to_one_and_the_rest_to_minus_one() {
    let t = vector_from(|i, _| match i % 4 {
        0 => Some(Real::from_lit(35, 1)),
        1 => Some(-math::TWO),
        2 => Some(math::ZERO),
        _ => None,
    });
    let s = t.sign();
    for (i, c) in axes().into_iter().enumerate() {
        let expected = match i % 4 {
            0 => Some(math::ONE),
            // Zero is not "defined positive": it maps to -1 (unlike Real::sign, which gives +1)
            1 | 2 => Some(-math::ONE),
            _ => None,
        };
        assert_eq!(s.get_coord(c), expected, "sign of {}", c.name());
    }
}

#[test]
fn with_coord_and_map_values_respect_undefined_coords() {
    let all = axes();
    let first = all[0];
    let rest = CoordSel::all_axis().difference(first);
    let t = TVector::<Real>::one().with_coord(first, None);

    let doubled = t.map_values(|_, v| Some(v * math::TWO));
    assert_eq!(doubled.get_coord(first), None);
    assert_eq!(doubled, TVector::new_with_coord(rest, Some(math::TWO)));

    // `with_coord` replaces the selected coords, including with None
    let replaced = t.with_coord(CoordSel::all_axis(), Some(math::THREE));
    assert_eq!(replaced, TVector::new_with_const_value(math::THREE));
    let cleared = t.with_coord(rest, None);
    assert_eq!(cleared, TVector::new());
    assert_eq!(t.with_coord(CoordSel::empty(), Some(math::TEN)), t);

    // `map_values` may also drop values
    let filtered = TVector::<Real>::one().map_values(|c, v| if c == first { None } else { Some(v) });
    assert_eq!(filtered, t);
}

#[test]
fn subtraction_propagates_undefined_coords() {
    let all = axes();
    let a = vector_from(|i, _| if i == 0 { None } else { Some(int(3 * i as i64)) });
    let b = vector_from(|i, _| if i == 1 { None } else { Some(int(i as i64)) });
    let d = a - b;
    for (i, c) in all.iter().enumerate() {
        let expected = if i < 2 { None } else { Some(int(2 * i as i64)) };
        assert_eq!(d.get_coord(*c), expected, "{}", c.name());
    }
    assert_eq!(TVector::<Real>::one() - TVector::one(), TVector::zero());
    assert_eq!(TVector::<Real>::one() - TVector::new(), TVector::new());
}

#[test]
#[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))]
fn scalar_product_projection_and_normalisation() {
    let tol = Real::from_f32(1.0e-5);
    let close = |a: Real, b: Real| (a - b).abs() < tol;
    // a = (3, 4) and b ⟂ a = (-4, 3), every other axis zero
    let base = TVector::<Real>::zero();
    let a = base
        .with_coord(CoordSel::X, Some(math::THREE))
        .with_coord(CoordSel::Y, Some(math::FOUR));
    let b = base
        .with_coord(CoordSel::X, Some(-math::FOUR))
        .with_coord(CoordSel::Y, Some(math::THREE));

    assert_eq!(a.scalar_product(b), math::ZERO, "orthogonal");
    assert_eq!(a.scalar_product(a * math::TWO), int(50), "parallel");
    assert_eq!(a.scalar_product(-a), int(-25), "anti-parallel");
    assert_eq!(a.scalar_projection(a * math::TWO), math::TWO);
    assert_eq!(a.scalar_projection(b), math::ZERO);
    // Undefined coords do not contribute to the product
    assert_eq!(a.scalar_product(TVector::new()), math::ZERO);

    let u = a.unit();
    assert!(close(u.get_coord(CoordSel::X).unwrap(), Real::from_lit(6, 1)));
    assert!(close(u.get_coord(CoordSel::Y).unwrap(), Real::from_lit(8, 1)));
    assert!(close(u.norm2().unwrap(), math::ONE));
    let (dir, norm) = a.decompose_normal();
    assert!(close(norm, math::FIVE));
    assert!(close(dir.scalar_product(u), math::ONE));

    // Zero-length vectors have no direction
    assert_eq!(TVector::<Real>::zero().unit(), TVector::nan());
    assert_eq!(TVector::<Real>::new().unit(), TVector::nan());
    assert_eq!(TVector::<Real>::zero().decompose_normal(), (TVector::nan(), math::ZERO));
    assert_eq!(TVector::<Real>::new().decompose_normal(), (TVector::nan(), math::ZERO));
}

#[test]
#[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))]
fn vector_debug_lists_defined_coords_and_propagates_writer_errors() {
    let xv = Real::from_lit(15, 1);
    let yv = -math::TWO;
    let t = TVector::<Real>::new()
        .with_coord(CoordSel::X, Some(xv))
        .with_coord(CoordSel::Y, Some(yv));
    let expected = std::format!("X {:?} Y {:?}", xv, yv);
    assert_eq!(std::format!("{:?}", t), expected);
    assert_eq!(
        std::format!("{:#?}", t),
        std::format!(
            "{} {:?} {} {:?}",
            CoordSel::X.alternative_name(),
            xv,
            CoordSel::Y.alternative_name(),
            yv
        )
    );
    assert_eq!(std::format!("{:?}", TVector::<Real>::new()), "");

    // Failing on the first coordinate, then on the separator before the second one
    assert!(write_limited(0, format_args!("{:?}", t)).0.is_err());
    let first = std::format!("X {:?}", xv);
    let (r, partial) = write_limited(first.len(), format_args!("{:?}", t));
    assert!(r.is_err());
    assert_eq!(partial, first);
    let (r, full) = write_limited(expected.len(), format_args!("{:?}", t));
    assert!(r.is_ok());
    assert_eq!(full, expected);
}

//#endregion

//#region "Scalar trait implementations"

#[test]
fn arithmetic_ops_of_integer_coordinates() {
    assert_eq!(<i32 as ArithmeticOps>::zero(), 0);
    assert_eq!(<i32 as ArithmeticOps>::one(), 1);
    assert!(ArithmeticOps::is_zero(&0i32));
    assert!(!ArithmeticOps::is_zero(&-1i32));
    assert!(ArithmeticOps::is_defined_positive(&1i32));
    assert!(!ArithmeticOps::is_defined_positive(&0i32));
    assert!(!ArithmeticOps::is_defined_positive(&-1i32));
    assert!(ArithmeticOps::is_negligible(&0i32));
    assert!(!ArithmeticOps::is_negligible(&1i32));
    assert_eq!(ArithmeticOps::abs(&-5i32), 5);

    macro_rules! check_unsigned {
        ($t:ty) => {{
            assert_eq!(<$t as ArithmeticOps>::zero(), 0);
            assert_eq!(<$t as ArithmeticOps>::one(), 1);
            assert!(ArithmeticOps::is_zero(&(0 as $t)));
            assert!(!ArithmeticOps::is_zero(&<$t>::MAX));
            assert!(ArithmeticOps::is_defined_positive(&(1 as $t)));
            assert!(!ArithmeticOps::is_defined_positive(&(0 as $t)));
            assert!(ArithmeticOps::is_negligible(&(0 as $t)));
            assert!(!ArithmeticOps::is_negligible(&(1 as $t)));
            assert_eq!(ArithmeticOps::abs(&<$t>::MAX), <$t>::MAX);
        }};
    }
    check_unsigned!(u8);
    check_unsigned!(u16);
    check_unsigned!(u32);
    check_unsigned!(u64);

    // Integer vectors: steps per axis
    let steps: TVector<u32> = TVector::new_with_const_value(7u32);
    assert_eq!(steps.sum(), 7 * CoordSel::num_axis() as u32);
    assert_eq!(steps.vmax(), Some(7));
    assert!(TVector::<u8>::zero().is_nan_or_zero());
    assert_eq!(TVector::<u16>::one().negligible_coords(), CoordSel::empty());
    assert_eq!(TVector::<u64>::zero().not_negligible_coords(), CoordSel::empty());
}

#[test]
fn arithmetic_and_real_ops_of_f32_coordinates() {
    assert_eq!(<f32 as ArithmeticOps>::zero(), 0.0);
    assert_eq!(<f32 as ArithmeticOps>::one(), 1.0);
    assert!(ArithmeticOps::is_zero(&0.0f32));
    assert!(ArithmeticOps::is_zero(&-0.0f32));
    assert!(ArithmeticOps::is_defined_positive(&f32::EPSILON));
    assert!(!ArithmeticOps::is_defined_positive(&0.0f32));
    assert!(ArithmeticOps::is_negligible(&(f32::EPSILON / 2.0)));
    assert!(ArithmeticOps::is_negligible(&(-f32::EPSILON / 2.0)));
    assert!(!ArithmeticOps::is_negligible(&f32::EPSILON));
    assert_eq!(ArithmeticOps::abs(&-2.5f32), 2.5);

    assert_eq!(RealOps::pow(&2.0f32, 3), 8.0);
    assert_eq!(RealOps::pow(&2.0f32, -1), 0.5);
    assert_eq!(RealOps::sqrt(&-1.0f32), None);
    assert_eq!(RealOps::sqrt(&0.0f32), Some(0.0));
    let s = RealOps::sqrt(&16.0f32).unwrap();
    assert!((s - 4.0).abs() < 1e-6);
    assert!((RealOps::rdp(&1.26f32, 1) - 1.3).abs() < 1e-6);
    assert!((RealOps::rdp(&-1.26f32, 1) + 1.3).abs() < 1e-6);
    assert_eq!(RealOps::floor(&-1.5f32), -2.0);
    assert_eq!(RealOps::ceil(&-1.5f32), -1.0);

    // Vector-level helpers built on the scalar implementations
    let v: TVector<f32> = TVector::new_with_const_value(4.0f32);
    assert_eq!(v.sqrt(), TVector::new_with_const_value(2.0f32));
    assert_eq!(TVector::new_with_const_value(-4.0f32).sqrt(), TVector::new());
}

/// `-0.0` compares equal to zero, but `<f32 as RealOps>::sqrt` checks the sign bit and rejects
/// it, so e.g. `(-TVector::<f32>::zero()).sqrt()` loses every coordinate.
/// `Real::sqrt(-0.0)` returns `Some(0)` in the float backends.
#[test]
fn f32_real_ops_sqrt_of_negative_zero_is_zero() {
    assert_eq!(RealOps::sqrt(&-0.0f32), Some(0.0));
}

#[test]
fn arithmetic_and_real_ops_of_real_coordinates() {
    assert_eq!(<Real as ArithmeticOps>::zero(), math::ZERO);
    assert_eq!(<Real as ArithmeticOps>::one(), math::ONE);
    assert!(ArithmeticOps::is_zero(&math::ZERO));
    assert!(ArithmeticOps::is_defined_positive(&Real::epsilon()));
    assert!(!ArithmeticOps::is_defined_positive(&math::ZERO));
    assert!(ArithmeticOps::is_negligible(&(Real::epsilon() / math::TWO)));
    assert_eq!(ArithmeticOps::abs(&-math::THREE), math::THREE);

    assert_eq!(RealOps::pow(&math::TWO, 3), math::EIGHT);
    assert_eq!(RealOps::sqrt(&math::NINE), Some(math::THREE));
    assert_eq!(RealOps::sqrt(&-math::NINE), None);
    assert_eq!(RealOps::rdp(&Real::from_lit(1234, 3), 1), Real::from_lit(12, 1));
    assert_eq!(RealOps::floor(&-math::HALF), -math::ONE);
    assert_eq!(RealOps::ceil(&-math::HALF), math::ZERO);

    let v = TVector::new_with_const_value(-Real::from_lit(25, 1));
    assert_eq!(v.abs(), TVector::new_with_const_value(Real::from_lit(25, 1)));
    assert_eq!(v.floor(), TVector::new_with_const_value(-math::THREE));
    assert_eq!(v.ceil(), TVector::new_with_const_value(-math::TWO));
    assert_eq!(v.pow(2), TVector::new_with_const_value(Real::from_lit(625, 2)));
}

//#endregion
