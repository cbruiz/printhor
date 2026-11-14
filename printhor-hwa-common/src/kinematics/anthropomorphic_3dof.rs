//! Spherical Anthropomorphic actuator kinematics: 4x 3DOF
//! STILL FRESH!! Work in Progress

use crate as hwa;
use hwa::kinematics::WorldToSpaceTransformer;
use hwa::math;
use math::{CoordSel, Real, TVector};

//#region "SingleActuator"

/// Single 3DOF actuator/effector Denavit–Hartenberg parameters
pub struct SingleActuator {
    /// Body height
    pub h: Real,
    /// first join length
    pub l1: Real,
    /// second join length
    pub l2: Real,
    /// third join (actuator) length
    pub l3: Real,
    /// Initial theta1
    pub initial_theta1: Real,
    /// Initial theta2
    pub initial_theta2: Real,
    /// Initial theta3
    pub initial_theta3: Real,
    /// Coordinate corresponding to relative X (world units)
    pub rel_x_coord: CoordSel,
    /// Coordinate corresponding to relative Y (world units)
    pub rel_y_coord: CoordSel,
    /// Coordinate corresponding to relative Z (world units)
    pub rel_z_coord: CoordSel,
    /// Coordinate corresponding to 1st join angle (space units)
    pub theta1_coord: CoordSel,
    /// Coordinate corresponding to 2nd join angle (space units)
    pub theta2_coord: CoordSel,
    /// Coordinate corresponding to 3rd join angle (space units)
    pub theta3_coord: CoordSel,
}
impl SingleActuator {
    pub const fn new(
        h: Real,
        l1: Real,
        l2: Real,
        l3: Real,
        initial_theta1: Real,
        initial_theta2: Real,
        initial_theta3: Real,
        rel_x_coord: CoordSel,
        rel_y_coord: CoordSel,
        rel_z_coord: CoordSel,
        theta1_coord: CoordSel,
        theta2_coord: CoordSel,
        theta3_coord: CoordSel,
    ) -> SingleActuator {
        Self {
            h,
            l1,
            l2,
            l3,
            initial_theta1,
            initial_theta2,
            initial_theta3,
            rel_x_coord,
            rel_y_coord,
            rel_z_coord,
            theta1_coord,
            theta2_coord,
            theta3_coord,
        }
    }

    /// Returns the CoordSel mask for the three joint-angle (space) coordinates.
    pub const fn coords(&self) -> CoordSel {
        self.theta1_coord
            .union(self.theta2_coord)
            .union(self.theta3_coord)
    }

    /// Returns the CoordSel mask for the three world-position coordinates.
    pub const fn world_coords(&self) -> CoordSel {
        self.rel_x_coord
            .union(self.rel_y_coord)
            .union(self.rel_z_coord)
    }

    /// Radii of the reachable spherical shell around the shoulder: `[|L2 − L3|, L2 + L3]`.
    #[inline]
    pub fn reach_limits(&self) -> (Real, Real) {
        ((self.l2 - self.l3).abs(), self.l2 + self.l3)
    }

    /// Arm-plane coordinates of an actuator-frame target `(x, y, z)`:
    /// `(θ1 [rad], ρ, p_z)` where `ρ` is the **signed** horizontal distance from the shoulder along
    /// the leg direction (negative when the target is between the vertical axis and the shoulder)
    /// and `p_z = h − z` (positive below the base).
    ///
    /// θ1 is undefined on the vertical axis; the home direction is used there.
    fn arm_plane(&self, x: Real, y: Real, z: Real) -> (Real, Real, Real) {
        let r_xy = (x * x + y * y).sqrt().unwrap_or(math::ZERO);
        let theta1 = if r_xy.is_negligible() {
            self.initial_theta1.d2r()
        } else {
            y.atan2(x)
        };
        (theta1, r_xy - self.l1, self.h - z)
    }

    /// Radial projection of an arm-plane point `(ρ, p_z)` onto the reachable spherical shell
    /// centred at the shoulder. Returns the (possibly) projected point and whether it was outside.
    ///
    /// Note: when `|L2 − L3| ≤ L1` (true for any practical leg) the projected point always stays
    /// on the same side of the vertical axis, so θ1 is preserved.
    fn clamp_plane(&self, rho: Real, pz: Real) -> (Real, Real, bool) {
        let (r_min, r_max) = self.reach_limits();
        let r = (rho * rho + pz * pz).sqrt().unwrap_or(math::ZERO);
        if r > r_max {
            let k = r_max / r;
            (rho * k, pz * k, true)
        } else if r < r_min {
            if r.is_negligible() {
                // Direction undefined: fold the arm pointing down
                (math::ZERO, r_min, true)
            } else {
                let k = r_min / r;
                (rho * k, pz * k, true)
            }
        } else {
            (rho, pz, false)
        }
    }

    /// Closest reachable position (actuator world coordinates) to `world_pos`: the target is
    /// projected radially onto the spherical shell `|L2 − L3| ≤ |P − S| ≤ L2 + L3` around the
    /// shoulder `S`. Coordinates not handled by this actuator are returned untouched.
    pub fn clamp_world(&self, world_pos: &TVector<Real>) -> (TVector<Real>, bool) {
        let (x, y, z) = match (
            world_pos.get_coord(self.rel_x_coord),
            world_pos.get_coord(self.rel_y_coord),
            world_pos.get_coord(self.rel_z_coord),
        ) {
            (Some(x), Some(y), Some(z)) => (x, y, z),
            _ => return (*world_pos, false),
        };
        let (theta1, rho, pz) = self.arm_plane(x, y, z);
        let (rho, pz, clamped) = self.clamp_plane(rho, pz);
        if !clamped {
            return (*world_pos, false);
        }
        let r_xy = rho + self.l1;
        let mut out = *world_pos;
        out.set_coord(self.rel_x_coord, Some(r_xy * theta1.cos()));
        out.set_coord(self.rel_y_coord, Some(r_xy * theta1.sin()));
        out.set_coord(self.rel_z_coord, Some(self.h - pz));
        (out, true)
    }
}

impl WorldToSpaceTransformer for SingleActuator {
    fn project_to_space(&self, world_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        let mut space_pos = TVector::new();

        let transformation_result = inverse_kinematics(&world_pos, self);
        match transformation_result {
            Err(_) => {
                hwa::error!("Error computing inverse kinematics for leg");
                return Err(());
            }
            Ok(transformed_pos) => {
                // inverse_kinematics stores angles at theta1/2/3_coord slots.
                space_pos.assign_if_set(self.coords(), &transformed_pos);
            }
        }
        Ok(space_pos)
    }

    fn project_to_world(&self, space_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        let mut world_pos = TVector::new();

        let transformation_result = direct_kinematics(&space_pos, self);
        match transformation_result {
            Err(()) => {
                hwa::error!("Error computing direct kinematics for leg");
                return Err(());
            }
            Ok(transformed_pos) => {
                // direct_kinematics stores world positions at rel_x/y/z_coord slots.
                world_pos.assign_if_set(self.world_coords(), &transformed_pos);
            }
        }
        Ok(world_pos)
    }

    fn clamp_to_workspace(&self, world_pos: &TVector<Real>) -> TVector<Real> {
        self.clamp_world(world_pos).0
    }
}

//#endregion

//#region "Quadruped"

/// A Quadruped spider. Just for fun nor profit
pub struct Quadruped {
    /// Leg Front-Left
    /// ```text
    ///
    ///  -X-/^\---
    ///  ---\_/---
    /// ```
    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis"
    ))]
    leg_fl: SingleActuator,
    /// Leg Front-Right
    /// ```text
    ///
    ///  ---/^\-X-
    ///  ---\_/---
    /// ```
    #[cfg(all(
        feature = "with-a-axis",
        feature = "with-b-axis",
        feature = "with-c-axis"
    ))]
    leg_fr: SingleActuator,
    /// Leg Bottom-Left
    /// ```text
    ///
    ///  ---/^\---
    ///  -X-\_/---
    /// ```
    #[cfg(all(
        feature = "with-i-axis",
        feature = "with-j-axis",
        feature = "with-k-axis"
    ))]
    leg_bl: SingleActuator,
    /// Leg Bottom-Right
    /// ```text
    ///
    ///  ---/^\---
    ///  ---\_/-X-
    /// ```
    #[cfg(all(
        feature = "with-u-axis",
        feature = "with-v-axis",
        feature = "with-w-axis"
    ))]
    leg_br: SingleActuator,
}
impl Quadruped {
    pub const fn new(
        #[cfg(all(
            feature = "with-x-axis",
            feature = "with-y-axis",
            feature = "with-z-axis"
        ))]
        leg_fl: SingleActuator,
        #[cfg(all(
            feature = "with-a-axis",
            feature = "with-b-axis",
            feature = "with-c-axis"
        ))]
        leg_fr: SingleActuator,
        #[cfg(all(
            feature = "with-i-axis",
            feature = "with-j-axis",
            feature = "with-k-axis"
        ))]
        leg_bl: SingleActuator,
        #[cfg(all(
            feature = "with-u-axis",
            feature = "with-v-axis",
            feature = "with-w-axis"
        ))]
        leg_br: SingleActuator,
    ) -> Self {
        Self {
            #[cfg(all(
                feature = "with-x-axis",
                feature = "with-y-axis",
                feature = "with-z-axis"
            ))]
            leg_fl,
            #[cfg(all(
                feature = "with-a-axis",
                feature = "with-b-axis",
                feature = "with-c-axis"
            ))]
            leg_fr,
            #[cfg(all(
                feature = "with-i-axis",
                feature = "with-j-axis",
                feature = "with-k-axis"
            ))]
            leg_bl,
            #[cfg(all(
                feature = "with-u-axis",
                feature = "with-v-axis",
                feature = "with-w-axis"
            ))]
            leg_br,
        }
    }
}

impl WorldToSpaceTransformer for Quadruped {
    fn project_to_space(&self, world_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        let mut space_pos = TVector::new();

        for actuator in [
            #[cfg(all(
                feature = "with-x-axis",
                feature = "with-y-axis",
                feature = "with-z-axis"
            ))]
            &self.leg_fl,
            #[cfg(all(
                feature = "with-a-axis",
                feature = "with-b-axis",
                feature = "with-c-axis"
            ))]
            &self.leg_fr,
            #[cfg(all(
                feature = "with-i-axis",
                feature = "with-j-axis",
                feature = "with-k-axis"
            ))]
            &self.leg_bl,
            #[cfg(all(
                feature = "with-u-axis",
                feature = "with-v-axis",
                feature = "with-w-axis"
            ))]
            &self.leg_br,
        ] {
            let transformed_pos = actuator.project_to_space(world_pos)?;
            space_pos.assign_if_set(actuator.coords(), &transformed_pos);
        }
        Ok(space_pos)
    }

    fn project_to_world(&self, space_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        let mut world_pos = TVector::new();

        for actuator in [
            #[cfg(all(
                feature = "with-x-axis",
                feature = "with-y-axis",
                feature = "with-z-axis"
            ))]
            &self.leg_fl,
            #[cfg(all(
                feature = "with-a-axis",
                feature = "with-b-axis",
                feature = "with-c-axis"
            ))]
            &self.leg_fr,
            #[cfg(all(
                feature = "with-i-axis",
                feature = "with-j-axis",
                feature = "with-k-axis"
            ))]
            &self.leg_bl,
            #[cfg(all(
                feature = "with-u-axis",
                feature = "with-v-axis",
                feature = "with-w-axis"
            ))]
            &self.leg_br,
        ] {
            let transformed_pos = actuator.project_to_world(space_pos)?;
            // direct_kinematics stores results at rel_x/y/z_coord slots; use world_coords() to read them.
            world_pos.assign_if_set(actuator.world_coords(), &transformed_pos);
        }
        Ok(world_pos)
    }

    /// Each leg clamps its own world coordinates to its reachable shell.
    fn clamp_to_workspace(&self, world_pos: &TVector<Real>) -> TVector<Real> {
        #[allow(unused_mut)]
        let mut out = *world_pos;
        for actuator in [
            #[cfg(all(
                feature = "with-x-axis",
                feature = "with-y-axis",
                feature = "with-z-axis"
            ))]
            &self.leg_fl,
            #[cfg(all(
                feature = "with-a-axis",
                feature = "with-b-axis",
                feature = "with-c-axis"
            ))]
            &self.leg_fr,
            #[cfg(all(
                feature = "with-i-axis",
                feature = "with-j-axis",
                feature = "with-k-axis"
            ))]
            &self.leg_bl,
            #[cfg(all(
                feature = "with-u-axis",
                feature = "with-v-axis",
                feature = "with-w-axis"
            ))]
            &self.leg_br,
        ] {
            out = actuator.clamp_world(&out).0;
        }
        out
    }
}

//#endregion

/// Analytically computes the inverse kinematics for a 3-DOF anthropomorphic manipulator (R-R-R in 3D)
///
/// https://www.geogebra.org/3d/agurreym
fn inverse_kinematics(
    world_pos: &TVector<Real>,
    _actuator: &SingleActuator,
) -> Result<TVector<Real>, ()> {
    hwa::debug!(
        "[i_kine] | h = {:?}, l1 = {:?}, l2 = {:?}, l3 = {:?}",
        _actuator.h,
        _actuator.l1,
        _actuator.l2,
        _actuator.l3
    );

    hwa::debug!(
        "[i_kine] | Actuator target pos: [X {:?} Y {:?} Z {:?}",
        world_pos.get_coord(_actuator.rel_x_coord),
        world_pos.get_coord(_actuator.rel_y_coord),
        world_pos.get_coord(_actuator.rel_z_coord),
    );

    let (x, y, z) = match (
        world_pos.get_coord(_actuator.rel_x_coord),
        world_pos.get_coord(_actuator.rel_y_coord),
        world_pos.get_coord(_actuator.rel_z_coord),
    ) {
        (Some(x), Some(y), Some(z)) => (x, y, z),
        _ => return Err(()),
    };

    // Guard against target coinciding with the base (singularity).
    let dz = z - _actuator.h;
    if (x * x + y * y + dz * dz).is_negligible() {
        hwa::warn!("[i_kine] target is at the base position — singularity, returning Err");
        return Err(());
    }

    // theta1: rotation about the vertical axis. (rho, pz): planar problem for theta2/theta3 in
    // the leg plane, with rho *signed* (the former |rho| mirrored every target lying between the
    // vertical axis and the shoulder, producing a wrong joint solution).
    let (theta1_r, rho, pz) = _actuator.arm_plane(x, y, z);

    // Workspace: the effector can only reach the spherical shell |L2−L3| ≤ r ≤ L2+L3 centred at
    // the shoulder. Unreachable targets are projected radially onto that shell (the former
    // axis-aligned box clamp left the box corners outside of the sphere and could alter θ1).
    let (rho, pz, _clamped) = _actuator.clamp_plane(rho, pz);
    #[cfg(feature = "with-log")]
    if _clamped {
        hwa::debug!("[i_kine] target out of reach: projected onto the workspace boundary");
    }

    // Law of cosines for the planar R-R arm (links l2, l3).
    let cos_theta3 = (rho * rho + pz * pz - _actuator.l2 * _actuator.l2 - _actuator.l3 * _actuator.l3)
        / (math::TWO * _actuator.l2 * _actuator.l3);
    // Rounding at the shell boundary may leave |cos| marginally above 1.
    let cos_theta3 = cos_theta3.clamp(-math::ONE, math::ONE);

    // Elbow-up solution (positive sin_theta3 branch).
    let sin_theta3 = (math::ONE - cos_theta3 * cos_theta3)
        .max(math::ZERO)
        .sqrt()
        .unwrap_or(math::ZERO);
    let theta3_r = sin_theta3.atan2(cos_theta3);

    let theta2_r = pz.atan2(rho)
        - (_actuator.l3 * sin_theta3).atan2(_actuator.l2 + _actuator.l3 * cos_theta3);

    // Convert to degrees and subtract initial offsets to get relative joint angles.
    let theta1 = theta1_r.r2d() - _actuator.initial_theta1;
    let theta2 = theta2_r.r2d() - _actuator.initial_theta2;
    let theta3 = theta3_r.r2d() - _actuator.initial_theta3;

    // Store the joint angles in a fresh vector using the actuator's SPACE (theta) coordinate slots.
    // The caller (project_to_space) reads from self.coords() which maps to theta1/2/3_coord.
    let mut p = TVector::new();
    p.set_coord(_actuator.theta1_coord, Some(theta1));
    p.set_coord(_actuator.theta2_coord, Some(theta2));
    p.set_coord(_actuator.theta3_coord, Some(theta3));

    Ok(p)
}

/// Analytically computes the direct kinematics for a 3-DOF anthropomorphic manipulator (R-R-R in 3D)
///
/// https://www.geogebra.org/3d/hujnva2f
///
/// ```text
/// tep1 = Rz(theta_1) * Trans(L_1, 0, H) =
///  | cos(theta_1)   -sin(theta_1)  0             L_1 * cos(theta_1)   |
///  | sin(theta_1)   cos(theta_1)   0             L_1 * sin(theta_1)   |
///  | 0              0              1             H                    |
///  | 0              0              0             1                    |
///
/// tep2 = Ry(theta_2) * Trans(L_2, 0, 0) =
///  | cos(theta_2)   0              sin(theta_2)   L_2 * cos(theta_2)  |
///  | 0              1              0              0                   |
///  | -sin(theta_2)  0              cos(theta_2)   -L_2 * sin(theta_2) |
///  | 0              0              0              1                   |
///
/// tep3 = Ry(theta_3) * Trans(L_3, 0, 0) =
///  | cos(theta_3)   0            sin(theta_3)     L_3 * cos(theta_3)  |
///  | 0              1            0                0                   |
///  | -sin(theta_3)  0            cos(theta_3)     -L_3 * sin(theta_3) |
///  | 0              0            0                1                   |
///
/// direct_kine_efector = tep1 (*) tep2 (*) tep3 (*) (0;0;0;1)
/// ```
///
fn direct_kinematics(
    space_pos: &TVector<Real>,
    _actuator: &SingleActuator,
) -> Result<TVector<Real>, ()> {
    hwa::debug!(
        "[kine] | h = {:?}, l1 = {:?}, l2 = {:?}, l3 = {:?}",
        _actuator.h,
        _actuator.l1,
        _actuator.l2,
        _actuator.l3
    );

    // Add the initial offsets to get absolute DH angles (in degrees), then convert to radians.
    let theta1_d = space_pos
        .get_coord(_actuator.theta1_coord)
        .unwrap_or(math::ZERO)
        + _actuator.initial_theta1;
    let theta2_d = space_pos
        .get_coord(_actuator.theta2_coord)
        .unwrap_or(math::ZERO)
        + _actuator.initial_theta2;
    let theta3_d = space_pos
        .get_coord(_actuator.theta3_coord)
        .unwrap_or(math::ZERO)
        + _actuator.initial_theta3;

    let theta_1 = theta1_d.d2r();
    let theta_2 = theta2_d.d2r();
    let theta_3 = theta3_d.d2r();

    // Not yet optimized for readability.
    // rel_x = L1·cos(t1) + [L2·cos(t2) + L3·cos(t2+t3)]·cos(t1)
    let rel_x = {
        (_actuator.l1 * theta_1.cos()) + (_actuator.l2 * theta_1.cos() * theta_2.cos())
            - (_actuator.l3 * theta_1.cos() * theta_2.sin() * theta_3.sin())
            + (_actuator.l3 * theta_1.cos() * theta_2.cos() * theta_3.cos())
    };

    // rel_y = L1·sin(t1) + [L2·cos(t2) + L3·cos(t2+t3)]·sin(t1)
    let rel_y = {
        (_actuator.l1 * theta_1.sin()) + (_actuator.l2 * theta_1.sin() * theta_2.cos())
            - (_actuator.l3 * theta_1.sin() * theta_2.sin() * theta_3.sin())
            + (_actuator.l3 * theta_1.sin() * theta_2.cos() * theta_3.cos())
    };

    // rel_z = H − L2·sin(t2) − L3·sin(t2+t3)
    let rel_z = {
        (_actuator.h)
            - (_actuator.l2 * theta_2.sin())
            - (_actuator.l3 * theta_2.sin() * theta_3.cos())
            - (_actuator.l3 * theta_3.sin() * theta_2.cos())
    };

    // Store world position in a *fresh* vector at the actuator's WORLD (rel) coordinate slots.
    // The caller (project_to_world) reads from self.world_coords() which maps to rel_x/y/z_coord.
    let mut p = TVector::new();
    p.set_coord(_actuator.rel_x_coord, Some(rel_x));
    p.set_coord(_actuator.rel_y_coord, Some(rel_y));
    p.set_coord(_actuator.rel_z_coord, Some(rel_z));

    Ok(p)
}

// The f64 backend computes sin/cos/atan2 with micromath (f32 approximations, ~1e-3 rad):
// FK/IK round trips then deviate up to ~0.7 mm, beyond these tests' tolerances. With `std`
// (host tests) the f32 backend uses the exact std functions instead, so they run in f32 only.
#[cfg(all(test, not(feature = "float-point-f64-impl")))]
mod tests {
    use crate as hwa;
    use crate::kinematics::anthropomorphic_3dof::{
        SingleActuator, direct_kinematics, inverse_kinematics,
    };
    use hwa::math;

    /// Tolerance for approximate equality checks (in the same units as the link lengths).
    const TOL: f32 = 1e-2;

    /// Actuator whose world coords (X,Y,Z) and space/theta coords (X,Y,Z) are the same.
    /// This is the simplest case and useful for verifying the kinematics math.
    const fn actuator_same_coords() -> SingleActuator {
        SingleActuator::new(
            hwa::make_real!(70.0),   // h
            hwa::make_real!(24.0),   // l1
            hwa::make_real!(55.0),   // l2
            hwa::make_real!(70.0),   // l3
            hwa::make_real!(45.0),   // initial_theta1 (degrees)
            hwa::make_real!(0.0),    // initial_theta2
            hwa::make_real!(90.0),   // initial_theta3
            hwa::CoordSel::X,        // rel_x_coord (world)
            hwa::CoordSel::Y,        // rel_y_coord (world)
            hwa::CoordSel::Z,        // rel_z_coord (world)
            hwa::CoordSel::X,        // theta1_coord (space)
            hwa::CoordSel::Y,        // theta2_coord (space)
            hwa::CoordSel::Z,        // theta3_coord (space)
        )
    }

    /// Actuator whose world coords are X,Y,Z but space/theta coords are A,B,C.
    /// This exercises the coord-mapping path that was previously broken.
    const fn actuator_different_coords() -> SingleActuator {
        SingleActuator::new(
            hwa::make_real!(70.0),
            hwa::make_real!(24.0),
            hwa::make_real!(55.0),
            hwa::make_real!(70.0),
            hwa::make_real!(45.0),
            hwa::make_real!(0.0),
            hwa::make_real!(90.0),
            hwa::CoordSel::X,        // world: X
            hwa::CoordSel::Y,        // world: Y
            hwa::CoordSel::Z,        // world: Z
            hwa::CoordSel::A,        // space: A  (different from rel coords!)
            hwa::CoordSel::B,        // space: B
            hwa::CoordSel::C,        // space: C
        )
    }

    // Legacy helpers kept for backward compatibility with existing tests.
    const fn top_right_actuator() -> SingleActuator { actuator_same_coords() }

    const fn top_left_actuator() -> SingleActuator {
        SingleActuator::new(
            hwa::make_real!(70.0),
            hwa::make_real!(24.0),
            hwa::make_real!(55.0),
            hwa::make_real!(70.0),
            hwa::make_real!(45.0),
            hwa::make_real!(0.0),
            hwa::make_real!(90.0),
            hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z,
            hwa::CoordSel::A, hwa::CoordSel::B, hwa::CoordSel::C,
        )
    }

    const fn bottom_right_actuator() -> SingleActuator {
        SingleActuator::new(
            hwa::make_real!(70.0),
            hwa::make_real!(24.0),
            hwa::make_real!(55.0),
            hwa::make_real!(70.0),
            hwa::make_real!(45.0),
            hwa::make_real!(0.0),
            hwa::make_real!(90.0),
            hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z,
            hwa::CoordSel::I, hwa::CoordSel::J, hwa::CoordSel::K,
        )
    }

    const fn bottom_left_actuator() -> SingleActuator {
        SingleActuator::new(
            hwa::make_real!(70.0),
            hwa::make_real!(24.0),
            hwa::make_real!(55.0),
            hwa::make_real!(70.0),
            hwa::make_real!(45.0),
            hwa::make_real!(0.0),
            hwa::make_real!(90.0),
            hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z,
            hwa::CoordSel::U, hwa::CoordSel::V, hwa::CoordSel::W,
        )
    }

    fn approx_eq(label: &str, got: math::Real, expected: f32) {
        // Backend-agnostic (f32 / f64)
        let v = got.to_f64() as f32;
        assert!(
            (v - expected).abs() < TOL,
            "{label}: got {v} but expected {expected} (tol={TOL})"
        );
    }

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    #[test]
    fn test_construction() {
        let _quadruped = crate::kinematics::anthropomorphic_3dof::Quadruped::new(
            #[cfg(all(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis"))]
            top_right_actuator(),
            #[cfg(all(feature = "with-a-axis", feature = "with-b-axis", feature = "with-c-axis"))]
            top_left_actuator(),
            #[cfg(all(feature = "with-i-axis", feature = "with-j-axis", feature = "with-k-axis"))]
            bottom_right_actuator(),
            #[cfg(all(feature = "with-u-axis", feature = "with-v-axis", feature = "with-w-axis"))]
            bottom_left_actuator(),
        );
    }

    // -----------------------------------------------------------------------
    // Direct kinematics (space → world)
    // -----------------------------------------------------------------------

    /// Space home (all offsets = 0) must map to the known forward-kinematics world centre.
    ///
    /// With initial angles t1=45°, t2=0°, t3=90°, h=70, l1=24, l2=55, l3=70:
    ///   x = (l1+l2)·cos(45°) = 79/√2 ≈ 55.86
    ///   y = (l1+l2)·sin(45°) ≈ 55.86
    ///   z = h − l3·sin(90°)   = 0
    #[test]
    fn test_direct_kinematics_home() {
        let actuator = actuator_same_coords();
        let space = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 0.0);
        let world = direct_kinematics(&space, &actuator).unwrap();

        approx_eq("world.x", world.get_coord(hwa::CoordSel::X).unwrap(), 55.8614);
        approx_eq("world.y", world.get_coord(hwa::CoordSel::Y).unwrap(), 55.8614);
        approx_eq("world.z", world.get_coord(hwa::CoordSel::Z).unwrap(), 0.0);
    }

    /// Same test but the space coords are A/B/C instead of X/Y/Z.
    /// Previously broken because direct_kinematics started from *space_pos
    /// and the caller read from theta (A/B/C) instead of world (X/Y/Z) slots.
    #[test]
    fn test_direct_kinematics_different_coords() {
        let actuator = actuator_different_coords();
        // Space home uses A/B/C.
        let space = hwa::make_vector_real!(a = 0.0, b = 0.0, c = 0.0);
        let world = direct_kinematics(&space, &actuator).unwrap();

        // direct_kinematics stores at rel_x/y/z (= X/Y/Z for this actuator).
        approx_eq("world.x", world.get_coord(hwa::CoordSel::X).unwrap(), 55.8614);
        approx_eq("world.y", world.get_coord(hwa::CoordSel::Y).unwrap(), 55.8614);
        approx_eq("world.z", world.get_coord(hwa::CoordSel::Z).unwrap(), 0.0);
    }

    // -----------------------------------------------------------------------
    // Inverse kinematics (world → space)
    // -----------------------------------------------------------------------

    /// Inverse of the world centre must give space home (0, 0, 0).
    #[test]
    fn test_inverse_kinematics_home() {
        let actuator = actuator_same_coords();
        let world_center = hwa::make_vector_real!(x = 55.8614311, y = 55.8614311, z = 0.0);
        let space = inverse_kinematics(&world_center, &actuator).unwrap();

        approx_eq("theta1", space.get_coord(hwa::CoordSel::X).unwrap(), 0.0);
        approx_eq("theta2", space.get_coord(hwa::CoordSel::Y).unwrap(), 0.0);
        approx_eq("theta3", space.get_coord(hwa::CoordSel::Z).unwrap(), 0.0);
    }

    /// Same with different coord mapping: inverse should give A/B/C = 0/0/0.
    /// Previously broken because inverse_kinematics stored at rel_x/y/z (X/Y/Z)
    /// instead of theta1/2/3 (A/B/C).
    #[test]
    fn test_inverse_kinematics_different_coords() {
        let actuator = actuator_different_coords();
        let world_center = hwa::make_vector_real!(x = 55.8614311, y = 55.8614311, z = 0.0);
        let space = inverse_kinematics(&world_center, &actuator).unwrap();

        approx_eq("theta1 (A)", space.get_coord(hwa::CoordSel::A).unwrap(), 0.0);
        approx_eq("theta2 (B)", space.get_coord(hwa::CoordSel::B).unwrap(), 0.0);
        approx_eq("theta3 (C)", space.get_coord(hwa::CoordSel::C).unwrap(), 0.0);
        // World coord slots must NOT be set by inverse_kinematics.
        assert!(space.get_coord(hwa::CoordSel::X).is_none(), "X should not be set in space output");
    }

    // -----------------------------------------------------------------------
    // Round-trip tests
    // -----------------------------------------------------------------------

    /// direct then inverse: space(0,0,0) → world_center → space(0,0,0).
    #[test]
    fn test_round_trip_direct_then_inverse() {
        let actuator = actuator_same_coords();
        let space_home = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 0.0);

        let world = direct_kinematics(&space_home, &actuator).unwrap();
        let space_back = inverse_kinematics(&world, &actuator).unwrap();

        approx_eq("theta1", space_back.get_coord(hwa::CoordSel::X).unwrap(), 0.0);
        approx_eq("theta2", space_back.get_coord(hwa::CoordSel::Y).unwrap(), 0.0);
        approx_eq("theta3", space_back.get_coord(hwa::CoordSel::Z).unwrap(), 0.0);
    }

    /// inverse then direct: world_center → space(0,0,0) → world_center.
    #[test]
    fn test_round_trip_inverse_then_direct() {
        let actuator = actuator_same_coords();
        let world_center = hwa::make_vector_real!(x = 55.8614311, y = 55.8614311, z = 0.0);

        let space = inverse_kinematics(&world_center, &actuator).unwrap();
        let world_back = direct_kinematics(&space, &actuator).unwrap();

        approx_eq("x", world_back.get_coord(hwa::CoordSel::X).unwrap(), 55.8614);
        approx_eq("y", world_back.get_coord(hwa::CoordSel::Y).unwrap(), 55.8614);
        approx_eq("z", world_back.get_coord(hwa::CoordSel::Z).unwrap(), 0.0);
    }

    /// Round-trip with different coord mapping (A/B/C space, X/Y/Z world).
    #[test]
    fn test_round_trip_different_coords() {
        let actuator = actuator_different_coords();
        let space_home = hwa::make_vector_real!(a = 0.0, b = 0.0, c = 0.0);

        let world = direct_kinematics(&space_home, &actuator).unwrap();
        let space_back = inverse_kinematics(&world, &actuator).unwrap();

        approx_eq("theta1 (A)", space_back.get_coord(hwa::CoordSel::A).unwrap(), 0.0);
        approx_eq("theta2 (B)", space_back.get_coord(hwa::CoordSel::B).unwrap(), 0.0);
        approx_eq("theta3 (C)", space_back.get_coord(hwa::CoordSel::C).unwrap(), 0.0);
    }

    // -----------------------------------------------------------------------
    // Non-home positions
    // -----------------------------------------------------------------------

    /// Arm pointing straight up (theta2 = -90° offset → absolute 0° from horizontal, then tilted).
    /// Just verifies that various non-zero joint angles survive a direct→inverse round-trip.
    #[test]
    fn test_round_trip_various_angles() {
        let actuator = actuator_same_coords();
        // Explore a small grid of joint-angle offsets (relative to home).
        for (dt1, dt2, dt3) in [
            (0.0_f32, 10.0, -10.0),
            (10.0, 0.0, 5.0),
            (-10.0, -5.0, 10.0),
            (15.0, 10.0, -15.0),
        ] {
            let space_in = math::TVector::new()
                .with_coord(hwa::CoordSel::X, Some(math::Real::from_f32(dt1)))
                .with_coord(hwa::CoordSel::Y, Some(math::Real::from_f32(dt2)))
                .with_coord(hwa::CoordSel::Z, Some(math::Real::from_f32(dt3)));
            let world = direct_kinematics(&space_in, &actuator).unwrap();
            let space_back = inverse_kinematics(&world, &actuator).unwrap();

            approx_eq(
                &format!("theta1 (dt1={dt1})"),
                space_back.get_coord(hwa::CoordSel::X).unwrap(),
                dt1,
            );
            approx_eq(
                &format!("theta2 (dt2={dt2})"),
                space_back.get_coord(hwa::CoordSel::Y).unwrap(),
                dt2,
            );
            approx_eq(
                &format!("theta3 (dt3={dt3})"),
                space_back.get_coord(hwa::CoordSel::Z).unwrap(),
                dt3,
            );
        }
    }

    // -----------------------------------------------------------------------
    // Boundary / robustness tests
    // -----------------------------------------------------------------------

    /// Target at the base position is a singularity and must return Err, not panic.
    #[test]
    fn test_boundary_target_at_base() {
        let actuator = actuator_same_coords();
        // b = (0, 0, h=70)
        let at_base = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 70.0);
        let result = inverse_kinematics(&at_base, &actuator);
        assert!(result.is_err(), "target at base should return Err (singularity)");
    }

    /// An out-of-reach target must be clamped and still return Ok (not panic).
    #[test]
    fn test_boundary_out_of_reach_clamped() {
        let actuator = actuator_same_coords();
        // Far outside the workspace (l2+l3 = 125 mm from the shoulder).
        let far = hwa::make_vector_real!(x = 1000.0, y = 1000.0, z = 0.0);
        let result = inverse_kinematics(&far, &actuator);
        assert!(result.is_ok(), "out-of-reach target should succeed after clamping, got {:?}", result.err());
    }

    /// A target that would yield |cos_theta3| > 1 without clamping must not panic.
    /// We create such a case by placing the target at the boundary of reachability
    /// where floating-point arithmetic can push cos_theta3 slightly outside [-1, 1].
    #[test]
    fn test_boundary_cos_theta3_no_panic() {
        let actuator = actuator_same_coords();
        // Arm at full extension in XY plane: distance from shoulder = l2+l3 = 125,
        // in the direction of initial_theta1=45°.
        // shoulder is at (l1·cos45°, l1·sin45°, h) = (16.97, 16.97, 70)
        // end-effector = shoulder + (125·cos45°, 125·sin45°, 0)
        let reach = 24.0_f32 + 125.0; // l1 + l2 + l3 from origin in XY
        let angle = core::f32::consts::FRAC_PI_4;
        let target = math::TVector::new()
            .with_coord(hwa::CoordSel::X, Some(math::Real::from_f32(reach * angle.cos())))
            .with_coord(hwa::CoordSel::Y, Some(math::Real::from_f32(reach * angle.sin())))
            .with_coord(hwa::CoordSel::Z, Some(math::Real::from_f32(70.0)));
        let result = inverse_kinematics(&target, &actuator);
        assert!(result.is_ok(), "full-extension target must not panic, got {:?}", result.err());
    }

    /// X-axis boundary clamping (was broken when radius.X was overwritten to 0).
    /// With the fix, the clamped target along pure X must be accepted.
    #[test]
    fn test_boundary_radius_x_clamping() {
        let actuator = actuator_same_coords();
        // A point far along the +X axis (theta1 ≈ 0°, not 45°).
        let target_x = hwa::make_vector_real!(x = 500.0, y = 1.0, z = 0.0);
        let result = inverse_kinematics(&target_x, &actuator);
        assert!(result.is_ok(), "X-axis boundary clamping must succeed, got {:?}", result.err());
    }

    // -----------------------------------------------------------------------
    // Displacement tests (up/down)
    // -----------------------------------------------------------------------

    /// Moving 10mm up from world_center must give a computable space position.
    #[test]
    fn test_move_10mm_up() {
        let actuator = top_right_actuator();
        let world_center = hwa::make_vector_real!(x = 55.8614311, y = 55.8614311, z = 0.0);
        let p_up = world_center + hwa::make_vector_real!(x = 0.0, y = 0.0, z = 10.0);
        let result = inverse_kinematics(&p_up, &actuator);
        assert!(result.is_ok(), "10mm up should succeed, got {:?}", result.err());
    }

    /// Moving 10mm down from world_center must give a computable space position.
    #[test]
    fn test_move_10mm_down() {
        let actuator = top_right_actuator();
        let world_center = hwa::make_vector_real!(x = 55.8614311, y = 55.8614311, z = 0.0);
        let p_down = world_center + hwa::make_vector_real!(x = 0.0, y = 0.0, z = -10.0);
        let result = inverse_kinematics(&p_down, &actuator);
        assert!(result.is_ok(), "10mm down should succeed, got {:?}", result.err());
    }

    // -----------------------------------------------------------------------
    // project_to_space / project_to_world (trait-level tests)
    // -----------------------------------------------------------------------

    /// project_to_world → project_to_space round-trip via the trait interface.
    #[test]
    fn test_trait_round_trip_same_coords() {
        use crate::kinematics::WorldToSpaceTransformer;
        let actuator = actuator_same_coords();
        let space_home = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 0.0);

        let world = actuator.project_to_world(&space_home).unwrap();
        let space_back = actuator.project_to_space(&world).unwrap();

        approx_eq("theta1", space_back.get_coord(hwa::CoordSel::X).unwrap(), 0.0);
        approx_eq("theta2", space_back.get_coord(hwa::CoordSel::Y).unwrap(), 0.0);
        approx_eq("theta3", space_back.get_coord(hwa::CoordSel::Z).unwrap(), 0.0);
    }

    /// Same round-trip with different coord mapping via the trait interface.
    /// This specifically tests the bug fix in project_to_world (world_coords vs coords).
    #[test]
    fn test_trait_round_trip_different_coords() {
        use crate::kinematics::WorldToSpaceTransformer;
        let actuator = actuator_different_coords();
        let space_home = hwa::make_vector_real!(a = 0.0, b = 0.0, c = 0.0);

        let world = actuator.project_to_world(&space_home).unwrap();

        // The world output must live at X/Y/Z (the rel_x/y/z_coord slots).
        approx_eq("world.x", world.get_coord(hwa::CoordSel::X).unwrap(), 55.8614);
        approx_eq("world.y", world.get_coord(hwa::CoordSel::Y).unwrap(), 55.8614);

        let space_back = actuator.project_to_space(&world).unwrap();

        // Space output must live at A/B/C (the theta1/2/3_coord slots).
        approx_eq("theta1 (A)", space_back.get_coord(hwa::CoordSel::A).unwrap(), 0.0);
        approx_eq("theta2 (B)", space_back.get_coord(hwa::CoordSel::B).unwrap(), 0.0);
        approx_eq("theta3 (C)", space_back.get_coord(hwa::CoordSel::C).unwrap(), 0.0);
    }

    // -----------------------------------------------------------------------
    // Workspace (reachable spherical shell) and IK correctness
    // -----------------------------------------------------------------------

    fn xyz(v: &math::TVector<math::Real>) -> (f64, f64, f64) {
        (
            v.get_coord(hwa::CoordSel::X).unwrap().to_f64(),
            v.get_coord(hwa::CoordSel::Y).unwrap().to_f64(),
            v.get_coord(hwa::CoordSel::Z).unwrap().to_f64(),
        )
    }

    fn world(x: f64, y: f64, z: f64) -> math::TVector<math::Real> {
        math::TVector::new()
            .with_coord(hwa::CoordSel::X, Some(math::Real::from_f32(x as f32)))
            .with_coord(hwa::CoordSel::Y, Some(math::Real::from_f32(y as f32)))
            .with_coord(hwa::CoordSel::Z, Some(math::Real::from_f32(z as f32)))
    }

    /// Distance from the shoulder (h=70, l1=24) to a world point
    fn shoulder_distance(p: (f64, f64, f64)) -> f64 {
        let r_xy = (p.0 * p.0 + p.1 * p.1).sqrt();
        let rho = r_xy - 24.0;
        let pz = 70.0 - p.2;
        (rho * rho + pz * pz).sqrt()
    }

    /// Targets between the vertical axis and the shoulder used to be mirrored by the IK
    /// (|ρ| instead of signed ρ): FK(IK(P)) must reproduce P.
    #[test]
    fn test_ik_target_between_axis_and_shoulder() {
        let actuator = actuator_same_coords();
        let a = 30.0f64.to_radians();
        let p = world(10.0 * a.cos(), 10.0 * a.sin(), 10.0); // r_xy = 10 < l1 = 24
        let q = inverse_kinematics(&p, &actuator).unwrap();
        let back = xyz(&direct_kinematics(&q, &actuator).unwrap());
        let want = xyz(&p);
        for (g, w) in [(back.0, want.0), (back.1, want.1), (back.2, want.2)] {
            assert!((g - w).abs() < 1e-2, "FK(IK(P)) = {:?} expected {:?}", back, want);
        }
    }

    /// Any reachable point must round-trip FK → IK → FK.
    #[test]
    fn test_ik_fk_round_trip_sweep() {
        let actuator = actuator_same_coords();
        let mut seed: u64 = 0x9E3779B97F4A7C15;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..500 {
            // Joint offsets relative to home, elbow-up branch (absolute θ3 in (5°, 175°))
            let q = math::TVector::new()
                .with_coord(hwa::CoordSel::X, Some(math::Real::from_f32((rnd() * 160.0 - 80.0) as f32)))
                .with_coord(hwa::CoordSel::Y, Some(math::Real::from_f32((rnd() * 140.0 - 70.0) as f32)))
                .with_coord(hwa::CoordSel::Z, Some(math::Real::from_f32((rnd() * 170.0 - 85.0) as f32)));
            let p = direct_kinematics(&q, &actuator).unwrap();
            let (px, py, _) = xyz(&p);
            if (px * px + py * py).sqrt() < 1.0 {
                continue; // on the vertical axis θ1 is not observable
            }
            let p2 = direct_kinematics(&inverse_kinematics(&p, &actuator).unwrap(), &actuator).unwrap();
            let (a, b) = (xyz(&p), xyz(&p2));
            let err = ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt();
            assert!(err < 2e-2, "round trip error {} for q={:?} p={:?} p2={:?}", err, q, a, b);
        }
    }

    /// Out-of-reach targets are projected exactly onto the outer sphere (|P−S| = L2+L3),
    /// keeping their direction from the shoulder; FK(IK(target)) lands on that projection.
    #[test]
    fn test_workspace_outer_sphere_projection() {
        let actuator = actuator_same_coords();
        for &(x, y, z) in &[(1000.0, 1000.0, 0.0), (500.0, 1.0, 0.0), (-300.0, 80.0, -200.0), (40.0, 40.0, 300.0)] {
            let target = world(x, y, z);
            let (clamped, was_clamped) = actuator.clamp_world(&target);
            assert!(was_clamped);
            let c = xyz(&clamped);
            assert!((shoulder_distance(c) - 125.0).abs() < 1e-2, "on the outer sphere: {:?}", c);
            // Direction preserved: same azimuth
            assert!(((c.1).atan2(c.0) - y.atan2(x)).abs() < 1e-4);
            let fk = xyz(&direct_kinematics(&inverse_kinematics(&target, &actuator).unwrap(), &actuator).unwrap());
            let err = ((fk.0 - c.0).powi(2) + (fk.1 - c.1).powi(2) + (fk.2 - c.2).powi(2)).sqrt();
            assert!(err < 2e-2, "effector must reach the projected point: {:?} vs {:?}", fk, c);
        }
    }

    /// Targets inside the inner unreachable ball (|P−S| < |L2−L3| = 15) are pushed to it.
    #[test]
    fn test_workspace_inner_sphere_projection() {
        let actuator = actuator_same_coords();
        let a = 45.0f64.to_radians();
        // 5 units from the shoulder, outwards
        let target = world((24.0 + 5.0) * a.cos(), (24.0 + 5.0) * a.sin(), 70.0);
        let (clamped, was_clamped) = actuator.clamp_world(&target);
        assert!(was_clamped);
        assert!((shoulder_distance(xyz(&clamped)) - 15.0).abs() < 1e-2);
        // Reachable targets are left untouched
        let inside = world(55.8614311, 55.8614311, 0.0);
        assert_eq!(actuator.clamp_world(&inside), (inside, false));
    }

    /// Joint angles must vary continuously along a straight world path inside the workspace
    /// (no branch flips), so that smooth world motion maps to smooth joint motion.
    #[test]
    fn test_ik_continuity_along_path() {
        let actuator = actuator_same_coords();
        let (p0, p1) = ((70.0, 20.0, -10.0), (5.0, 60.0, 20.0)); // crosses the r_xy < l1 region
        let n = 400;
        let mut prev: Option<(f64, f64, f64)> = None;
        for k in 0..=n {
            let t = k as f64 / n as f64;
            let p = world(p0.0 + (p1.0 - p0.0) * t, p0.1 + (p1.1 - p0.1) * t, p0.2 + (p1.2 - p0.2) * t);
            let q = xyz(&inverse_kinematics(&p, &actuator).unwrap());
            if let Some(pq) = prev {
                let d = (q.0 - pq.0).abs().max((q.1 - pq.1).abs()).max((q.2 - pq.2).abs());
                assert!(d < 5.0, "joint jump of {}° at t={} ({:?} -> {:?})", d, t, pq, q);
            }
            prev = Some(q);
        }
    }
}

#[cfg(test)]
mod coverage_tests;
