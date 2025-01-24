use crate::hwa;
use hwa::math::{CoordSel, Real, TVector};
use hwa::SyncMutexStrategy;
#[allow(unused)]
use hwa::{Contract, HwiContract};

#[derive(Clone, Copy)]
/// A position in world and workspace coordinates
///
/// - is_set: Indicates the position is initialized
/// - world_pos: The position in world coordinates
/// - space_pos: The position in space coordinates
pub struct Position {
    pub is_set: bool,
    pub world_pos: TVector<Real>,
    pub space_pos: TVector<Real>,
}

impl Position {
    pub const fn new() -> Self {
        Self {
            is_set: false,
            world_pos: TVector::new_with_coord(
                CoordSel::motion_relevant_axis(),
                Some(Real::zero()),
            ),
            space_pos: TVector::new(),
        }
    }

    pub const fn new_from(world_pos: TVector<Real>, space_pos: TVector<Real>) -> Self {
        Self {
            is_set: true,
            world_pos,
            space_pos,
        }
    }

    pub fn new_from_space_projection(space_pos: &TVector<Real>) -> Self {
        let mut pos = Self::new();
        pos.update_from_space_coordinates(space_pos);
        pos
    }

    pub fn new_with_world_projection(world_position: &TVector<Real>) -> Self {
        let position = Self::new().complete_and_project(world_position);
        position
    }

    pub fn upsert(&mut self, new: &Position) {
        self.is_set = true;
        self.world_pos
            .assign_if_set(CoordSel::motion_relevant_axis(), &new.world_pos);
        self.space_pos
            .assign_if_set(CoordSel::motion_relevant_axis(), &new.space_pos);
    }

    pub fn update_from_world_coordinates(&mut self, updated_world_coordinates: &TVector<Real>) {
        self.world_pos
            .assign_if_set(CoordSel::motion_relevant_axis(), updated_world_coordinates);
        match Contract.project_to_space(&self.world_pos) {
            Ok(space_pos) => {
                self.is_set = true;
                self.space_pos = space_pos;
            }
            Err(_) => {
                hwa::error!("Unable to project world coordinate");
                self.is_set = false;
                self.space_pos = TVector::new();
            }
        }
    }

    pub fn update_from_space_coordinates(&mut self, updated_space_coordinates: &TVector<Real>) {
        self.space_pos
            .assign_if_set(CoordSel::motion_relevant_axis(), updated_space_coordinates);
        match Contract.project_to_world(&self.space_pos) {
            Ok(world_pos) => {
                self.is_set = true;
                self.world_pos = world_pos;
            }
            Err(_) => {
                hwa::error!("Unable to project world coordinate");
                self.is_set = false;
                self.world_pos = TVector::new();
            }
        }
    }

    pub fn complete_and_project(&self, updated_world_coordinates: &TVector<Real>) -> Self {
        let mut pos = *self;
        pos.update_from_world_coordinates(updated_world_coordinates);
        pos
    }
}

/// Wraps the actual motion status
pub struct MotionStatus {
    cfg: hwa::StaticSyncController<hwa::types::MotionStatusMutexStrategy>,
}

impl MotionStatus {
    pub fn new(cfg: hwa::StaticSyncController<hwa::types::MotionStatusMutexStrategy>) -> Self {
        Self { cfg }
    }

    pub fn set_absolute_positioning(&self, absolute_is_set: bool) {
        self.cfg.apply_mut(|m| {
            m.absolute_positioning = absolute_is_set;
        });
    }

    /// Update last planned position. E is always ignored (So far, only relative E moves are implemented)
    pub fn update_last_planned_position(&self, _order_num: u32, updated_position: &Position) {
        self.cfg.apply_mut(|m| {
            m.last_planned_position.upsert(updated_position);
            #[cfg(feature = "debug-motion")]
            hwa::info!(
                "[MotionStatus] order_num:{:?} Last planned position updated to world: [{:?}] {}",
                _order_num,
                m.last_planned_position.world_pos,
                Contract::WORLD_UNIT_MAGNITUDE,
            );
            #[cfg(feature = "debug-motion")]
            hwa::info!(
                "[MotionStatus] order_num:{:?} Last planned position updated to space: [{:?}] {}",
                _order_num,
                m.last_planned_position.space_pos,
                Contract::SPACE_UNIT_MAGNITUDE,
            );
        });
    }

    /// Update current position. E is always ignored (So far, only relative E moves are implemented)
    pub fn update_current_position(&self, _order_num: u32, updated_position: &Position) {
        self.cfg.apply_mut(|m| {
            m.current_position.upsert(updated_position);
            #[cfg(feature = "debug-motion")]
            hwa::info!(
                "[MotionStatus] order_num:{:?} Current position updated to world: [{:?}] {}",
                _order_num,
                m.current_position.world_pos,
                Contract::WORLD_UNIT_MAGNITUDE,
            );
            #[cfg(feature = "debug-motion")]
            hwa::info!(
                "[MotionStatus] order_num:{:?} Current position updated to space: [{:?}] {}",
                _order_num,
                m.current_position.space_pos,
                Contract::SPACE_UNIT_MAGNITUDE,
            );
        });
    }

    /// Get a copy of the last planned position.
    ///
    /// Returns the last planned position in world and (work)space units in a tuple
    pub fn get_last_planned_position(&self) -> Position {
        self.cfg.apply(|m| m.last_planned_position)
    }

    /// Get a copy of the current real position
    pub fn get_current_position(&self) -> Position {
        self.cfg.apply(|m| m.current_position)
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
    /// Last recorded real position in World Units and (Work)Space Units. See [Position]
    pub current_position: Position,
    /// Last planned position in World Units and (Work)Space Units. See [Position]
    pub last_planned_position: Position,
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
            current_position: Position::new(),
            last_planned_position: Position::new(),
            absolute_positioning: true,
            #[cfg(feature = "with-laser")]
            laser: false,
        }
    }
}
