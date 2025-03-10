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
            world_pos.assign_if_set(actuator.coords(), &transformed_pos);
        }
        Ok(world_pos)
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
    let mut p = *world_pos;

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

    let e = TVector::zero()
        .with_coord(CoordSel::X, world_pos.get_coord(_actuator.rel_x_coord))
        .with_coord(CoordSel::Y, world_pos.get_coord(_actuator.rel_y_coord))
        .with_coord(CoordSel::Z, world_pos.get_coord(_actuator.rel_z_coord));
    // base position
    let b = TVector::zero().with_coord(CoordSel::Z, Some(_actuator.h));
    let e_delta = e - b;
    let e_max = e_delta.abs() / e_delta.norm2().unwrap() * (_actuator.l2 + _actuator.l3);

    // E_{capped}=(
    // Minimo(x(B) + L_{1} + x(E_{max}), Máximo(x(E), x(B) - L_{1} - x(E_{max}))),
    // Mínimo(y(B) + L_{1} + y(E_{max}), Máximo(y(E), y(B) - L_{1} - y(E_{max}))),
    // Mínimo(z(B) + z(E_{max}), Máximo(z(E), z(B) - z(E_{max}))))

    let radius = TVector::zero()
        .with_coord(CoordSel::X, Some(_actuator.l1))
        .with_coord(CoordSel::Y, Some(_actuator.l1))
        .with_coord(CoordSel::X, Some(math::ZERO));

    let e_capped = e
        .clamp_higher_than(b - radius - e_max)
        .clamp_lower_than(b + radius + e_max);

    let x = e_capped.get_coord(CoordSel::X).unwrap();
    let y = e_capped.get_coord(CoordSel::Y).unwrap();
    let z = e_capped.get_coord(CoordSel::Z).unwrap();

    let theta1_r = y.atan2(x);

    let px = x - _actuator.l1 * theta1_r.cos();
    let py = y - _actuator.l1 * theta1_r.sin();
    let pz = _actuator.h - z;

    let cos_theta3 =
        (px.powi(2) + py.powi(2) + pz.powi(2) - _actuator.l2.powi(2) - _actuator.l3.powi(2))
            / (math::TWO * _actuator.l2 * _actuator.l3);

    // \text{theta}_{3_r} = tan^{-1}\Big(\frac{\sqrt{1-cos^2_{\text{theta}_3}}}{cos_{\text{theta}_3}}\Big)
    let theta3_r = (math::ONE - cos_theta3.powi(2))
        .sqrt()
        .unwrap()
        .atan2(cos_theta3);

    let theta2_r = pz.atan2((px.powi(2) + py.powi(2)).sqrt().unwrap())
        - (_actuator.l3 * theta3_r.sin()).atan2(_actuator.l2 + _actuator.l3 * theta3_r.cos());

    let theta1 = theta1_r.r2d() - _actuator.initial_theta1;
    let theta2 = theta2_r.r2d() - _actuator.initial_theta2;
    let theta3 = theta3_r.r2d() - _actuator.initial_theta3;

    p.set_coord(_actuator.rel_x_coord, Some(theta1));
    p.set_coord(_actuator.rel_y_coord, Some(theta2));
    p.set_coord(_actuator.rel_z_coord, Some(theta3));

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

    // Not yet optimized for readability

    let rel_x = {
        (_actuator.l1 * theta_1.cos()) + (_actuator.l2 * theta_1.cos() * theta_2.cos())
            - (_actuator.l3 * theta_1.cos() * theta_2.sin() * theta_3.sin())
            + (_actuator.l3 * theta_1.cos() * theta_2.cos() * theta_3.cos())
    };

    let rel_y = {
        (_actuator.l1 * theta_1.sin()) + (_actuator.l2 * theta_1.sin() * theta_2.cos())
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
    use crate as hwa;
    use crate::kinematics::anthropomorphic_3dof::{
        SingleActuator, direct_kinematics, inverse_kinematics,
    };

    const fn top_right_actuator() -> SingleActuator {
        SingleActuator::new(
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
            hwa::CoordSel::X,
            hwa::CoordSel::Y,
            hwa::CoordSel::Z,
            // Which space coordinates by label to use for output
            hwa::CoordSel::X,
            hwa::CoordSel::Y,
            hwa::CoordSel::Z,
        )
    }

    const fn top_left_actuator() -> SingleActuator {
        SingleActuator::new(
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
            hwa::CoordSel::X,
            hwa::CoordSel::Y,
            hwa::CoordSel::Z,
            // Which space coordinates by label to use for output
            hwa::CoordSel::A,
            hwa::CoordSel::B,
            hwa::CoordSel::C,
        )
    }

    const fn bottom_right_actuator() -> SingleActuator {
        SingleActuator::new(
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
            hwa::CoordSel::X,
            hwa::CoordSel::Y,
            hwa::CoordSel::Z,
            // Which space coordinates by label to use for output
            hwa::CoordSel::I,
            hwa::CoordSel::J,
            hwa::CoordSel::K,
        )
    }

    const fn bottom_left_actuator() -> SingleActuator {
        SingleActuator::new(
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
            hwa::CoordSel::X,
            hwa::CoordSel::Y,
            hwa::CoordSel::Z,
            // Which space coordinates by label to use for output
            hwa::CoordSel::U,
            hwa::CoordSel::V,
            hwa::CoordSel::W,
        )
    }

    #[test]
    fn test_construction() {
        let _quadruped = crate::kinematics::anthropomorphic_3dof::Quadruped::new(
            #[cfg(all(
                feature = "with-x-axis",
                feature = "with-y-axis",
                feature = "with-z-axis"
            ))]
            top_right_actuator(),
            #[cfg(all(
                feature = "with-a-axis",
                feature = "with-b-axis",
                feature = "with-c-axis"
            ))]
            top_left_actuator(),
            #[cfg(all(
                feature = "with-i-axis",
                feature = "with-j-axis",
                feature = "with-k-axis"
            ))]
            bottom_right_actuator(),
            #[cfg(all(
                feature = "with-u-axis",
                feature = "with-v-axis",
                feature = "with-w-axis"
            ))]
            bottom_left_actuator(),
        );
    }

    #[test]
    fn test_kinematics() {
        let actuator = top_right_actuator();

        // 1.1 Test space center projects to world center
        let space_center_pos = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 0.0);
        // Coordinate translation: Assuming the initial position of final effector is 0,0,0
        let world_center = hwa::make_vector_real!(x = 55.8614311, y = 55.8614311, z = 0.0);
        // The normalized world center
        let normalized_world_center = hwa::make_vector_real!(x = 0.0, y = 0.0, z = 0.0);

        // Project (0,0,0) in space units to world to get the final effector position
        let world_pos = direct_kinematics(&space_center_pos, &actuator).unwrap();
        // Normalize world position. (traslate final effector relative position to 0,0,0)
        let world_normalized_pos = world_pos - world_center;

        // Deviation from expected world_pos (position relative to base)
        let distance_1 = (world_normalized_pos - normalized_world_center)
            .norm2()
            .unwrap();
        assert!(
            distance_1.is_negligible(),
            "World Projection is negligible compared to expected"
        );

        // 1.2 Test world center projects to space center

        let space_absolute_pos =
            inverse_kinematics(&(world_normalized_pos + world_center), &actuator).unwrap();
        let _distance_2 = space_absolute_pos.norm2().unwrap();
        /* TODO: Check
        assert!(
            distance_2.is_negligible(),
            "Space Projection is negligible compared to expected"
        );
        */

        // 1.3 Test other projections
        // 1.3.1 10mm up
        let p1 = world_center + hwa::make_vector_real!(x = 0.0, y = 0.0, z = 10.0);
        let space_absolute_pos = inverse_kinematics(&p1, &actuator).unwrap();
        let _distance_3 = (space_absolute_pos
            - hwa::make_vector_real!(x = 0.0, y = -10.4693403, z = 9.71987152))
        .norm2()
        .unwrap();
        /* TODO: Check
        assert!(
            distance_3.is_negligible(),
            "Space Projection is negligible compared to expected"
        );

         */

        // 1.3.1 10mm down
        let p1 = world_center + hwa::make_vector_real!(x = 0.0, y = 0.0, z = -10.0);
        let space_absolute_pos = inverse_kinematics(&p1, &actuator).unwrap();
        let _distance_3 = (space_absolute_pos
            - hwa::make_vector_real!(x = 0.0, y = 10.4820585, z = -11.233345))
        .norm2()
        .unwrap();
        /* TODO: Check
        assert!(
            distance_3.is_negligible(),
            "Space Projection is negligible compared to expected"
        );
         */
    }
}
