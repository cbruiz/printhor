//! STILL FRESH!! Work in Progress

use crate as hwa;
use hwa::kinematics::WorldToSpaceTransformer;
use hwa::math;
use math::{CoordSel, Real, TVector};

//#region "SingleActuator"

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

    hwa::debug!("[i_kine] | xyz => [{:?}, {:?}, {:?}]", x, y, z);

    let theta1_r = y.atan2(x);

    let px = x - _actuator.l1 * theta1_r.cos();
    let py = y - _actuator.l1 * theta1_r.sin();
    let pz = z - _actuator.h;

    hwa::debug!("[i_kine] | px,py,pz => [{:?}, {:?}, {:?}]", px, py, pz);

    let cos_theta3 =
        (px.powi(2) + py.powi(2) + pz.powi(2) - _actuator.l2.powi(2) - _actuator.l3.powi(2))
            / (math::TWO * _actuator.l2 * _actuator.l3);

    let theta3_r = -((math::ONE - cos_theta3.powi(2)).atan2(cos_theta3));

    let theta2_r = pz.atan2(((px * px) + (py * py)).sqrt().unwrap())
        - (_actuator.l3 * theta3_r.sin()).atan2(_actuator.l2 + _actuator.l3 * theta3_r.cos());

    let theta1 = theta1_r.r2d() - hwa::make_real!(90.0) + _actuator.initial_theta1;
    let theta2 = -theta2_r.r2d();
    let theta3 = theta3_r.r2d();

    hwa::debug!(
        "[i_kine] theta = [{:?}, {:?}, {:?}]",
        theta1,
        theta2,
        theta3
    );

    p.set_coord(_actuator.rel_x_coord, Some(theta1));
    p.set_coord(_actuator.rel_y_coord, Some(theta2));
    p.set_coord(_actuator.rel_z_coord, Some(theta3));

    Ok(p)
}

/// Computes the direct kinematics for a 3-DOF anthropomorphic manipulator (R-R-R in 3D)
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

    hwa::debug!(
        "[kine] | theta => [{:?}, {:?}, {:?}]",
        theta1_d,
        theta2_d,
        theta3_d
    );

    let theta1 = theta1_d.d2r();
    let theta2 = theta2_d.d2r();
    let theta3 = theta3_d.d2r();

    let proj_xy = _actuator.l1 + (_actuator.l2 * theta2.cos()) + (_actuator.l3 * theta3.cos());
    let rel_x = proj_xy * theta1.sin();
    let rel_y = proj_xy * theta1.cos();
    let rel_z = _actuator.h + (_actuator.l2 * theta2.sin()) + (_actuator.l3 * theta3.sin());

    hwa::debug!(
        "[kine] | l2 sin(theta2): {:?} ",
        _actuator.l2 * theta2.sin()
    );
    hwa::debug!(
        "[kine] | l3 sin(theta3): {:?} ",
        _actuator.l3 * theta3.sin()
    );

    hwa::debug!("[kine] xyz = [{:?}, {:?}, {:?}]", rel_x, rel_y, rel_z);

    p.set_coord(_actuator.rel_x_coord, Some(rel_x));
    p.set_coord(_actuator.rel_y_coord, Some(rel_y));
    p.set_coord(_actuator.rel_z_coord, Some(rel_z));

    Ok(p)
}
