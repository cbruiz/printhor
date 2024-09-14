use crate::math::Real;
use crate::tgeo::TVector;

/// Represents the motion status with optional real and planned positions,
/// absolute positioning flag, and an optional laser flag when the `with-laser` feature is enabled.
pub struct MotionStatus {
    /// Last recorded real position.
    pub last_real_pos: Option<TVector<Real>>,
    /// Last planned position.
    pub last_planned_pos: Option<TVector<Real>>,
    /// Flag indicating if absolute positioning is enabled.
    pub absolute_positioning: bool,
    /// Flag indicating if the laser is enabled (only present when the `with-laser` feature is enabled).
    #[cfg(feature = "with-laser")]
    #[allow(unused)]
    pub laser: bool,
}

impl MotionStatus {
    /// Creates a new `MotionStatus` with default values.
    ///
    /// # Returns
    ///
    /// A new instance of `MotionStatus` with `last_real_pos` and `last_planned_pos` set to `None`,
    /// `absolute_positioning` set to `true`, and `laser` set to `false` when the `with-laser` feature is enabled.
    pub const fn new() -> Self {
        Self {
            last_real_pos: None,
            last_planned_pos: None,
            absolute_positioning: true,
            #[cfg(feature = "with-laser")]
            laser: false,
        }
    }
}
