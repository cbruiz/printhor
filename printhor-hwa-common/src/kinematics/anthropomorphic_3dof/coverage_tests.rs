//! Singularity, workspace and partial-input tests for the anthropomorphic 3DOF kinematics.
//!
//! Tolerances depend on the backend: the f64 backend evaluates sin/cos/atan2 with micromath
//! (f32 approximations, ~1e-3 rad), so forward/inverse round trips deviate up to ~1 mm there.
use super::*;
use crate as hwa;
use hwa::math::{Real, TVector};

fn lit(v: i64) -> Real {
    Real::from_lit(v, 0)
}

/// Position tolerance (world units) of the active backend.
const fn pos_tol() -> f64 {
    cfg_if::cfg_if! {
        if #[cfg(feature = "fixed-point-128-impl")] {
            1e-6
        } else if #[cfg(feature = "float-point-f64-impl")] {
            1.5
        } else {
            1e-2
        }
    }
}

/// Leg with h = 70, l1 = 24, l2 = 55, l3 = 70 (reachable shell around the shoulder: [15, 125])
/// whose home azimuth is the world +X direction of its own coordinate triplet.
fn leg(x: CoordSel, y: CoordSel, z: CoordSel) -> SingleActuator {
    SingleActuator::new(
        Real::from_lit(70, 0),
        Real::from_lit(24, 0),
        Real::from_lit(55, 0),
        Real::from_lit(70, 0),
        Real::from_lit(0, 0),
        Real::from_lit(0, 0),
        Real::from_lit(90, 0),
        x,
        y,
        z,
        x,
        y,
        z,
    )
}

fn point(a: &SingleActuator, x: Real, y: Real, z: Real) -> TVector<Real> {
    TVector::new()
        .with_coord(a.rel_x_coord, Some(x))
        .with_coord(a.rel_y_coord, Some(y))
        .with_coord(a.rel_z_coord, Some(z))
}

#[track_caller]
fn assert_point(label: &str, a: &SingleActuator, p: &TVector<Real>, expected: (f64, f64, f64), tol: f64) {
    let got = (
        p.get_coord(a.rel_x_coord).unwrap().to_f64(),
        p.get_coord(a.rel_y_coord).unwrap().to_f64(),
        p.get_coord(a.rel_z_coord).unwrap().to_f64(),
    );
    let err = ((got.0 - expected.0).powi(2) + (got.1 - expected.1).powi(2) + (got.2 - expected.2).powi(2)).sqrt();
    assert!(err <= tol, "{}: got {:?} expected {:?} (tol {})", label, got, expected, tol);
}

#[test]
fn reach_limits_and_coordinate_masks() {
    let a = leg(CoordSel::X, CoordSel::Y, CoordSel::Z);
    assert_eq!(a.reach_limits(), (lit(15), lit(125)));
    assert_eq!(a.coords(), CoordSel::X | CoordSel::Y | CoordSel::Z);
    assert_eq!(a.world_coords(), CoordSel::X | CoordSel::Y | CoordSel::Z);
    // |l2 - l3| is symmetric
    let mut b = leg(CoordSel::X, CoordSel::Y, CoordSel::Z);
    core::mem::swap(&mut b.l2, &mut b.l3);
    assert_eq!(b.reach_limits(), (lit(15), lit(125)));
}

/// On the vertical axis through the base the azimuth θ1 is undefined: the home direction is
/// used, and FK(IK(P)) still lands on P.
#[test]
fn target_on_the_vertical_axis_keeps_the_home_azimuth() {
    let a = leg(CoordSel::X, CoordSel::Y, CoordSel::Z);
    let target = point(&a, lit(0), lit(0), lit(0));
    let (clamped, was_clamped) = a.clamp_world(&target);
    assert!(!was_clamped, "74 units from the shoulder: reachable");
    assert_eq!(clamped, target);

    let q = inverse_kinematics(&target, &a).unwrap();
    assert_eq!(q.get_coord(a.theta1_coord), Some(lit(0)), "home azimuth");
    let back = direct_kinematics(&q, &a).unwrap();
    assert_point("FK(IK(axis point))", &a, &back, (0.0, 0.0, 0.0), pos_tol());
}

/// A target exactly at the shoulder has no direction: it is projected straight down onto the
/// inner sphere (fully folded elbow), and the effector actually reaches that projection.
#[test]
fn target_at_the_shoulder_folds_the_arm_downwards() {
    let a = leg(CoordSel::X, CoordSel::Y, CoordSel::Z);
    let shoulder = point(&a, lit(24), lit(0), lit(70));
    let (clamped, was_clamped) = a.clamp_world(&shoulder);
    assert!(was_clamped);
    assert_point("projection", &a, &clamped, (24.0, 0.0, 55.0), 1e-6);

    let q = inverse_kinematics(&shoulder, &a).unwrap();
    let back = direct_kinematics(&q, &a).unwrap();
    assert_point("FK(IK(shoulder))", &a, &back, (24.0, 0.0, 55.0), pos_tol());
}

#[test]
fn targets_on_the_shell_boundaries_are_not_clamped() {
    let a = leg(CoordSel::X, CoordSel::Y, CoordSel::Z);
    // Outer sphere: straight along the home direction at the shoulder height
    let outer = point(&a, lit(24 + 125), lit(0), lit(70));
    assert_eq!(a.clamp_world(&outer), (outer, false));
    // Inner sphere: straight below the shoulder
    let inner = point(&a, lit(24), lit(0), lit(70 - 15));
    assert_eq!(a.clamp_world(&inner), (inner, false));
    // Just outside the outer sphere
    let beyond = point(&a, lit(24 + 126), lit(0), lit(70));
    let (c, was_clamped) = a.clamp_world(&beyond);
    assert!(was_clamped);
    assert_point("outer projection", &a, &c, (149.0, 0.0, 70.0), 1e-6);
}

/// An actuator needs its three world coordinates: anything less cannot be transformed and is
/// passed through the workspace clamp untouched.
#[test]
fn partial_targets_are_rejected_or_passed_through() {
    let a = leg(CoordSel::X, CoordSel::Y, CoordSel::Z);
    let full = point(&a, lit(60), lit(10), lit(0));
    for missing in [a.rel_x_coord, a.rel_y_coord, a.rel_z_coord] {
        let partial = full.with_coord(missing, None);
        assert_eq!(a.clamp_world(&partial), (partial, false));
        assert_eq!(a.clamp_to_workspace(&partial), partial);
        assert!(inverse_kinematics(&partial, &a).is_err());
        assert!(a.project_to_space(&partial).is_err());
    }
    // An empty space position is read as all-zero joint offsets (the home pose)
    let home = a.project_to_world(&TVector::new()).unwrap();
    let home_explicit = a
        .project_to_world(&TVector::new_with_coord(a.coords(), Some(lit(0))))
        .unwrap();
    assert_eq!(home, home_explicit);
}

#[test]
fn single_actuator_clamp_to_workspace_matches_clamp_world() {
    let a = leg(CoordSel::X, CoordSel::Y, CoordSel::Z);
    for (x, y, z) in [(1000, 0, 0), (24, 0, 70), (30, 30, -500), (60, 10, 0)] {
        let p = point(&a, lit(x), lit(y), lit(z));
        assert_eq!(a.clamp_to_workspace(&p), a.clamp_world(&p).0);
    }
}

#[cfg(all(
    feature = "with-x-axis",
    feature = "with-y-axis",
    feature = "with-z-axis",
    feature = "with-a-axis",
    feature = "with-b-axis",
    feature = "with-c-axis",
    feature = "with-i-axis",
    feature = "with-j-axis",
    feature = "with-k-axis",
    feature = "with-u-axis",
    feature = "with-v-axis",
    feature = "with-w-axis"
))]
mod quadruped {
    use super::*;

    fn legs() -> [SingleActuator; 4] {
        [
            leg(CoordSel::X, CoordSel::Y, CoordSel::Z),
            leg(CoordSel::A, CoordSel::B, CoordSel::C),
            leg(CoordSel::I, CoordSel::J, CoordSel::K),
            leg(CoordSel::U, CoordSel::V, CoordSel::W),
        ]
    }

    fn robot() -> Quadruped {
        let [fl, fr, bl, br] = legs();
        Quadruped::new(fl, fr, bl, br)
    }

    /// Each leg clamps its own coordinates onto its own shell, independently of the others.
    #[test]
    fn quadruped_clamps_every_leg_independently() {
        let legs = legs();
        let mut target = TVector::new();
        // FL far away, FR at its shoulder, BL reachable, BR far below
        let goals = [(1000, 0, 70), (24, 0, 70), (60, 10, 0), (24, 0, -1000)];
        for (l, (x, y, z)) in legs.iter().zip(goals) {
            target.assign_if_set(l.world_coords(), &point(l, lit(x), lit(y), lit(z)));
        }
        #[cfg(feature = "with-e-axis")]
        target.set_coord(CoordSel::E, Some(lit(7)));

        let out = robot().clamp_to_workspace(&target);
        let expected = [(149.0, 0.0, 70.0), (24.0, 0.0, 55.0), (60.0, 10.0, 0.0), (24.0, 0.0, -55.0)];
        for (l, e) in legs.iter().zip(expected) {
            assert_point("leg clamp", l, &out, e, 1e-6);
        }
        #[cfg(feature = "with-e-axis")]
        assert_eq!(out.get_coord(CoordSel::E), Some(lit(7)), "non-leg coords are untouched");
    }

    /// One leg without a complete target makes the whole quadruped projection fail.
    #[test]
    fn quadruped_projection_fails_if_any_leg_is_incomplete() {
        let legs = legs();
        let mut target = TVector::new();
        for l in legs.iter() {
            target.assign_if_set(l.world_coords(), &point(l, lit(60), lit(10), lit(0)));
        }
        let space = robot().project_to_space(&target).unwrap();
        let world = robot().project_to_world(&space).unwrap();
        for l in legs.iter() {
            assert_point("quadruped round trip", l, &world, (60.0, 10.0, 0.0), pos_tol());
        }

        let broken = target.with_coord(CoordSel::K, None);
        assert!(robot().project_to_space(&broken).is_err());
    }
}
