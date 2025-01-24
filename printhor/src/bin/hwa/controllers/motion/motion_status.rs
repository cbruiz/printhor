use crate::hwa;
use hwa::math::{CoordSel, Real, TVector};
use hwa::SyncMutexStrategy;
#[allow(unused)]
use hwa::{Contract, HwiContract};

/// Wraps the actual motion status
pub struct MotionStatus {
    cfg: hwa::StaticSyncController<hwa::types::MotionStatusMutexStrategy>,
}

impl MotionStatus {
    pub fn new(cfg: hwa::StaticSyncController<hwa::types::MotionStatusMutexStrategy>) -> Self {
        Self { cfg }
    }

    pub fn set_real_current_position(&self, _order_num: u32, pos: &TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.current_real_position_wu.replace(*pos);
            hwa::info!(
                "[MotionStatus] order_num:{:?} Real position set to [{:?}] {}",
                _order_num,
                m.current_real_position_wu,
                Contract::WORLD_UNIT_MAGNITUDE
            );
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
    pub fn update_last_planned_position(
        &self,
        _order_num: u32,
        updated_position_coords: &TVector<Real>,
    ) {
        self.cfg.apply_mut(|m| {
            if let Some(last_position) = &mut m.last_planned_position_wu {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-e-axis")] {
                        last_position.assign_if_set(CoordSel::all().difference(CoordSel::E), updated_position_coords);
                    }
                    else {
                        last_position.assign_if_set(CoordSel::all(), updated_position_coords);
                    }
                }
            }
            else {
                let mut pos = TVector::zero();
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-e-axis")] {
                        pos.assign_if_set(CoordSel::all().difference(CoordSel::E), updated_position_coords);
                    }
                    else {
                        pos.assign_if_set(CoordSel::all(), updated_position_coords);
                    }
                }
                m.last_planned_position_wu = Some(pos);
            }
            hwa::info!("[MotionStatus] order_num:{:?} Last planned position updated to [{:?}] {}",
                _order_num, m.last_planned_position_wu, Contract::WORLD_UNIT_MAGNITUDE
            );
        });
    }

    pub fn update_current_real_position(
        &self,
        _order_num: u32,
        updated_position_coords: &TVector<Real>,
    ) {
        self.cfg.apply_mut(|m| {
            if let Some(last_position) = &mut m.current_real_position_wu {
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-e-axis")] {
                        last_position.assign_if_set(CoordSel::all().difference(CoordSel::E), updated_position_coords);
                    }
                    else {
                        last_position.assign_if_set(CoordSel::all(), updated_position_coords);
                    }
                }
            }
            else {
                let mut pos = TVector::zero();
                cfg_if::cfg_if! {
                    if #[cfg(feature="with-e-axis")] {
                        pos.assign_if_set(CoordSel::all().difference(CoordSel::E), updated_position_coords);
                    }
                    else {
                        pos.assign_if_set(CoordSel::all(), updated_position_coords);
                    }
                }
                m.current_real_position_wu = Some(pos);
            }
            hwa::info!("[MotionStatus] order_num:{:?} Current real position updated to [{:?}] {}",
                _order_num, m.last_planned_position_wu, Contract::WORLD_UNIT_MAGNITUDE
            );
        });
    }

    /// Get the last planned position in world units
    pub fn get_last_planned_position(&self) -> Option<TVector<Real>> {
        self.cfg.apply(|m| m.last_planned_position_wu)
    }

    /// Get the current real position in world units
    pub fn get_current_real_position(&self) -> Option<TVector<Real>> {
        self.cfg.apply(|m| m.current_real_position_wu)
    }

    pub fn is_absolute_positioning(&self) -> bool {
        self.cfg.apply(|m| m.absolute_positioning)
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
    pub current_real_position_wu: Option<TVector<Real>>,
    /// Last planned position.
    pub last_planned_position_wu: Option<TVector<Real>>,
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
            current_real_position_wu: None,
            last_planned_position_wu: None,
            absolute_positioning: true,
            #[cfg(feature = "with-laser")]
            laser: false,
        }
    }
}
