//! Motion Kinematics strategies
use crate as hwa;
use hwa::math::{Real, TVector};

pub trait WorldToSpaceTransformer {
    /// Transforms a World position to (Work)space position. By default, identity transform applied
    ///
    /// Parameters:
    ///
    /// - `_world_pos`: The world position.
    fn project_to_space(&self, _world_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        Ok(*_world_pos)
    }

    /// Transforms a (Work)space position to World position. By default, identity transform applied
    ///
    /// Parameters:
    ///
    /// - `_space_pos`: The Space position.
    fn project_to_world(&self, _space_pos: &TVector<Real>) -> Result<TVector<Real>, ()> {
        Ok(*_space_pos)
    }

    /// Returns the reachable world position closest to `_world_pos` (identity by default).
    ///
    /// Kinematics with a bounded workspace (e.g. articulated arms) override it so that the planner
    /// records the position the effector will actually reach instead of an unreachable target.
    fn clamp_to_workspace(&self, _world_pos: &TVector<Real>) -> TVector<Real> {
        *_world_pos
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion-anthropomorphic-kinematics")] {
        pub mod anthropomorphic_3dof;
    }
    else if #[cfg(feature = "with-motion-core-xy-kinematics")] {
        pub mod core_xy;
    }
    else if #[cfg(feature = "with-motion-delta-kinematics")] {
        pub mod delta;
    }
}
pub mod cartessian;

#[cfg(test)]
mod coverage_tests;
