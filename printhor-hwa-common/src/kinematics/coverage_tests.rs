//! Tests for the default (identity) kinematics and the Core-XY transformer.
use super::*;
use crate as hwa;
use hwa::math;
use hwa::math::CoordSel;

/// A position with every axis defined except the first one (exercises partial vectors).
fn partial_position() -> TVector<Real> {
    let first = CoordSel::all_axis().iter().next().unwrap();
    let mut p = TVector::new();
    for (i, c) in CoordSel::all_axis().iter().enumerate() {
        p.set_coord(c, Some(Real::from_lit(10 * i as i64 - 25, 1)));
    }
    p.set_coord(first, None);
    p
}

/// The default trait methods are the identity: the cartesian transformer never fails and never
/// clamps, whatever the magnitude or definition of the coordinates.
#[test]
fn cartessian_identity_projects_and_clamps_without_changes() {
    let identity = cartessian::Identity;
    for p in [
        partial_position(),
        TVector::new(),
        TVector::zero(),
        TVector::new_with_const_value(math::ONE_MILLION),
        TVector::new_with_const_value(-math::ONE_MILLION),
    ] {
        assert_eq!(identity.project_to_space(&p), Ok(p));
        assert_eq!(identity.project_to_world(&p), Ok(p));
        assert_eq!(identity.clamp_to_workspace(&p), p);
    }
}

#[cfg(all(
    feature = "with-motion-core-xy-kinematics",
    not(feature = "with-motion-anthropomorphic-kinematics")
))]
mod core_xy_tests {
    use super::*;
    use crate::kinematics::core_xy::CoreXYTransformer;

    fn xyz(x: i64, y: i64, z: Option<i64>) -> TVector<Real> {
        let mut p = TVector::new();
        p.set_coord(CoordSel::X, Some(Real::from_lit(x, 0)));
        p.set_coord(CoordSel::Y, Some(Real::from_lit(y, 0)));
        #[cfg(feature = "with-z-axis")]
        p.set_coord(CoordSel::Z, z.map(|v| Real::from_lit(v, 0)));
        #[cfg(not(feature = "with-z-axis"))]
        let _ = z;
        p
    }

    #[test]
    fn core_xy_maps_x_y_to_belt_sums_and_back() {
        let t = CoreXYTransformer;
        // A = X + Y, B = X - Y; other axes are untouched
        let world = xyz(10, 3, Some(5));
        let space = t.project_to_space(&world).unwrap();
        assert_eq!(space, xyz(13, 7, Some(5)));
        assert_eq!(t.project_to_world(&space), Ok(world));

        // Negative coordinates and pure-Y motion (both belts move in opposite directions)
        let world = xyz(0, -4, None);
        let space = t.project_to_space(&world).unwrap();
        assert_eq!(space, xyz(-4, 4, None));
        assert_eq!(t.project_to_world(&space), Ok(world));

        // Default clamp: identity
        assert_eq!(t.clamp_to_workspace(&world), world);
    }

    #[test]
    fn core_xy_requires_both_x_and_y() {
        let t = CoreXYTransformer;
        let full = xyz(1, 2, Some(3));
        for missing in [CoordSel::X, CoordSel::Y, CoordSel::X | CoordSel::Y] {
            let p = full.with_coord(missing, None);
            assert_eq!(t.project_to_space(&p), Err(()), "missing {:?}", missing);
            assert_eq!(t.project_to_world(&p), Err(()), "missing {:?}", missing);
        }
    }
}
