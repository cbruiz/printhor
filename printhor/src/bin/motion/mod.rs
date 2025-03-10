//! The Motion profile trait and its implementations
//!
use crate::hwa;
pub mod profile;

/// The `MotionProfile` trait provides methods for evaluating motion_control profiles.
pub trait MotionProfile {
    /// Returns the end time of the motion_control profile.
    fn end_time(&self) -> hwa::math::Real;

    /// Returns the end position of the motion_control profile.
    fn end_pos(&self) -> hwa::math::Real;

    /// Evaluates the position at a given time `t` within the motion_control profile.
    ///
    /// # Parameters
    ///
    /// - `t`: The time at which to evaluate the position.
    ///
    /// # Returns
    ///
    /// An `Option` containing a tuple with:
    /// - The evaluated position as `Real`.
    /// - A status as `u8` (implementation-specific).
    fn eval_position(&self, t: hwa::math::Real) -> Option<hwa::math::Real>;
}

pub use profile::*;
