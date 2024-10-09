use crate::{hwa, math};
use hwa::SyncMutexStrategy;
use math::{CoordSel, Real, TVector};

pub struct MotionStatus {
    cfg: hwa::StaticSyncController<hwa::types::MotionStatusMutexStrategy>
}

impl MotionStatus {
    pub fn new(cfg: hwa::StaticSyncController<hwa::types::MotionStatusMutexStrategy>) -> Self {
        Self { cfg }
    }

    pub fn set_last_planned_position(&self, position: &math::TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.last_planned_pos.replace(*position);
        })
    }

    pub fn set_last_planned_real_position(&self, pos: &TVector<Real>) {
        let p = pos.map_nan(&crate::math::ZERO);
        self.cfg.apply_mut(|m| {
            m.last_real_pos.replace(p);
        });
    }

    pub fn set_absolute_positioning(&self, absolute_is_set: bool) {
        self.cfg.apply_mut(|m| {
            m.absolute_positioning = absolute_is_set;
        });
    }

    /***
    Update last planned position. E is always ignored (So far, only relative E moves are implemented)
     */
    pub fn update_last_planned_position(&self, updated_position_coords: &TVector<Real>) {
        self.cfg.apply_mut(|m| {
            if let Some(last_position) = &mut m.last_planned_pos {
                last_position.assign_if_set(CoordSel::XYZ, updated_position_coords);
            }
        });
    }

    pub fn get_last_planned_position(&self) -> Option<TVector<Real>> {
        self.cfg.apply(|m| {
            m.last_planned_pos
        })
    }

    pub fn get_last_planned_real_position(&self) -> Option<TVector<Real>> {
        self.cfg.apply(|m| {
            m.last_real_pos
        })
    }

    pub fn is_absolute_positioning(&self) -> bool {
        self.cfg.apply(|m| {
            m.absolute_positioning
        })
    }

}

impl Clone for MotionStatus {
    fn clone(&self) -> Self {
        MotionStatus::new(self.cfg.clone())
    }
}

/// Represents the motion status with optional real and planned positions,
/// absolute positioning flag, and an optional laser flag when the `with-laser` feature is enabled.
pub struct MotionStatusContent {
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

impl MotionStatusContent {
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
