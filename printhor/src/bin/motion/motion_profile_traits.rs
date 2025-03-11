//! Motion Profile traits
use crate::hwa;

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
    /// The evaluated position as `Real`.
    fn eval_position(&self, t: hwa::math::Real) -> hwa::math::Real;
}
