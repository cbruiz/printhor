use crate::math::Real;
use crate::tgeo::TVector;

pub struct MotionStatus {
    pub last_real_pos: Option<TVector<Real>>,
    pub last_planned_pos: Option<TVector<Real>>,
    pub absolute_positioning: bool,
    #[cfg(feature="with-laser")]
    #[allow(unused)]
    pub laser: bool,
}

impl MotionStatus {
    pub const fn new() -> Self {
        Self {
            last_real_pos: None,
            last_planned_pos: None,
            absolute_positioning: true,
            #[cfg(feature="with-laser")]
            laser: false,
        }
    }
}