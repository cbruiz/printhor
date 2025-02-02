//! STILL FRESH!! Work in Progress

use crate as hwa;
use hwa::kinematics::WorldToSpaceTransformer;
use hwa::math;
use math::{CoordSel, Real, TVector};
use printhor_hwa_common_macros::make_real;
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

    pub const fn coords(&self) -> CoordSel {
        self.theta1_coord
            .union(self.theta2_coord)
            .union(self.theta3_coord)
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
                world_pos.assign_if_set(self.coords(), &transformed_pos);
            }
        }
        Ok(world_pos)
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
    leg_fl: SingleActuator,
    /// Leg Front-Right
    /// ```text
    ///
    ///  ---/^\-X-
    ///  ---\_/---
    /// ```
    leg_fr: SingleActuator,
    /// Leg Bottom-Left
    /// ```text
    ///
    ///  ---/^\---
    ///  -X-\_/---
    /// ```
    leg_bl: SingleActuator,
    /// Leg Bottom-Right
    /// ```text
    ///
    ///  ---/^\---
    ///  ---\_/-X-
    /// ```
    leg_br: SingleActuator,
}
impl Quadruped {
    pub const fn new(
        leg_fl: SingleActuator,
        leg_fr: SingleActuator,
        leg_bl: SingleActuator,
        leg_br: SingleActuator,
    ) -> Self {
        Self {
            leg_fl,
            leg_fr,
            leg_bl,
            leg_br,
        }
    }
}

impl WorldToSpaceTransformer for Quadruped {
    fn project_to_space(&self, world_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        let mut space_pos = TVector::new();

        for actuator in [&self.leg_fl, &self.leg_fr, &self.leg_bl, &self.leg_br] {
            let transformed_pos = actuator.project_to_space(world_pos)?;
            space_pos.assign_if_set(actuator.coords(), &transformed_pos);
        }
        Ok(space_pos)
    }

    fn project_to_world(&self, space_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        let mut world_pos = TVector::new();

        for actuator in [&self.leg_fl, &self.leg_fr, &self.leg_bl, &self.leg_br] {
            let transformed_pos = actuator.project_to_world(space_pos)?;
            world_pos.assign_if_set(actuator.coords(), &transformed_pos);
        }
        Ok(world_pos)
    }
}

//#endregion

/// Computes the inverse kinematics for a 3-DOF anthropomorphic manipulator (R-R-R in 3D)
fn inverse_kinematics(
    world_pos: &TVector<Real>,
    _actuator: &SingleActuator,
) -> Result<TVector<Real>, ()> {
    let mut p = *world_pos;

    hwa::debug!(
        "[i_kine] | h = {:?}, l1 = {:?}, l2 = {:?}, l3 = {:?}",
        _actuator.h,
        _actuator.l1,
        _actuator.l2,
        _actuator.l3
    );

    let x = world_pos
        .get_coord(_actuator.rel_x_coord)
        .unwrap_or(math::ZERO);
    let y = world_pos
        .get_coord(_actuator.rel_y_coord)
        .unwrap_or(math::ZERO);
    let z = world_pos
        .get_coord(_actuator.rel_z_coord)
        .unwrap_or(math::ZERO);

    let theta1_r = y.atan2(x);

    let px = x - _actuator.l1 * theta1_r.cos();
    let py = y - _actuator.l1 * theta1_r.sin();
    let pz = z - _actuator.h;

    let cos_theta3 =
        (px.powi(2) + py.powi(2) + pz.powi(2) - _actuator.l2.powi(2) - _actuator.l3.powi(2))
            / (math::TWO * _actuator.l2 * _actuator.l3);

    let theta3_r = -((math::ONE - cos_theta3.powi(2)).atan2(cos_theta3));

    let theta2_r = pz.atan2(((px * px) + (py * py)).sqrt().unwrap())
        - (_actuator.l3 * theta3_r.sin()).atan2(_actuator.l2 + _actuator.l3 * theta3_r.cos());

    let theta1 = theta1_r.r2d() - _actuator.initial_theta1;
    let theta2 = theta2_r.r2d() - _actuator.initial_theta2;
    let theta3 = theta3_r.r2d() - _actuator.initial_theta3;

    p.set_coord(_actuator.rel_x_coord, Some(theta1));
    p.set_coord(_actuator.rel_y_coord, Some(theta2));
    if theta3 > make_real!(90.0) {
        p.set_coord(_actuator.rel_z_coord, Some(theta3 - make_real!(180.0)));
    }
    else if theta3 < make_real!(-90.0) {
        p.set_coord(_actuator.rel_z_coord, Some(theta3 + make_real!(180.0)));
    }

    Ok(p)
}

/// Computes the direct kinematics for a 3-DOF anthropomorphic manipulator (R-R-R in 3D)
///
/// https://www.geogebra.org/3d/hujnva2f
///
/// ```text
/// tep1 = Rz(theta_1) * Trans(L_1, 0, H) =
///  | cos(theta_1)   -sin(theta_1)  0             L_1 * cos(theta_1)   |
///  | sin(theta_1)   cos(theta_1)   0             L_1 * sin(theta_1)   |
///  | 0              0              0             H                    |
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

    let mut p = *space_pos;

    let theta1_d = space_pos.get_coord(_actuator.theta1_coord).unwrap_or(math::ZERO)
        + _actuator.initial_theta1;
    let theta2_d = space_pos.get_coord(_actuator.theta2_coord).unwrap_or(math::ZERO)
        + _actuator.initial_theta2;
    let theta3_d = space_pos.get_coord(_actuator.theta3_coord).unwrap_or(math::ZERO)
        + _actuator.initial_theta3;

    let theta_1 = theta1_d.d2r();
    let theta_2 = theta2_d.d2r();
    let theta_3 = theta3_d.d2r();

    // Not yet optimized for readability

    let rel_x = {
        (_actuator.l1 * theta_1.cos())
            + (_actuator.l2 * theta_1.cos() * theta_2.cos())
            - (_actuator.l3 * theta_1.cos() * theta_2.sin() * theta_3.sin())
            + (_actuator.l3 * theta_1.cos() * theta_2.cos() * theta_3.cos())
    };

    let rel_y = {
        (_actuator.l1 * theta_1.sin())
            + (_actuator.l2 * theta_1.sin() * theta_2.cos())
            - (_actuator.l3 * theta_1.sin() * theta_2.sin() * theta_3.sin())
            + (_actuator.l3 * theta_1.sin() * theta_2.cos() * theta_3.cos())
    };

    let rel_z = {
        (_actuator.h)
            - (_actuator.l2 * theta_2.sin())
            - (_actuator.l3 * theta_2.sin() * theta_3.cos())
            - (_actuator.l3 * theta_3.sin() * theta_2.cos())
    };

    p.set_coord(_actuator.rel_x_coord, Some(rel_x));
    p.set_coord(_actuator.rel_y_coord, Some(rel_y));
    p.set_coord(_actuator.rel_z_coord, Some(rel_z));

    Ok(p)
}

#[cfg(test)]
mod tests {
    use printhor_hwa_common_macros::make_vector_real;
    use crate as hwa;
    use crate::kinematics::anthropomorphic_3dof::{direct_kinematics, inverse_kinematics, SingleActuator};
    use crate::math;
    use crate::math::TVector;

    #[test]
    fn test_kinematics() {
        use crate::kinematics::anthropomorphic_3dof::SingleActuator;

        let actuator = SingleActuator::new(
            // Height
            hwa::make_real!(70.0),
            // L1 length
            hwa::make_real!(24.0),
            // L2 length
            hwa::make_real!(55.0),
            // L3 length
            hwa::make_real!(70.0),
            // theta1 initial angle
            hwa::make_real!(45.0),
            // theta2 initial angle
            hwa::make_real!(0.0),
            // theta3 initial angle
            hwa::make_real!(90.0),
            // Which world coordinates labels to get from input
            hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z,
            // Which space coordinates by label to use for output 
            hwa::CoordSel::X, hwa::CoordSel::Y, hwa::CoordSel::Z,
        );

        let space_pos = hwa::make_vector_real!(x=0.0, y=0.0, z=0.0);
        // Project (0,0,0) in space units to world to get the final effector position
        let world_absolute_pos = direct_kinematics(&space_pos, &actuator).unwrap();
        // Coordinate translation: Assuming the initial position of final effector is 0,0,0
        let world_center = make_vector_real!(x=55.8614311, y=55.8614311, z=0.0);
        // Compute relative world position.
        let world_pos = world_absolute_pos - world_center;

        // Deviation from expected world_pos
        let distance_1 = world_pos.norm2().unwrap();
        assert!(distance_1.is_negligible(), "World Projection is negligible compared to expected");

        let space_absolute_pos = inverse_kinematics(&world_absolute_pos, &actuator).unwrap();
        let distance_2 = space_absolute_pos.norm2().unwrap();
        assert!(distance_1.is_negligible(), "Space Projection is negligible compared to expected");
    }
}
