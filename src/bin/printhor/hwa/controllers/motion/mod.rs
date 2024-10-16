/// The module for motion configuration-related functionalities.
mod motion_config;

/// The module for motion segment-related functionalities.
mod motion_segment;

/// The module for motion status-related functionalities.
mod motion_status;

/// The module for motion ring buffer functionalities.
mod motion_ring_buffer;

/// The module for motion planner functionalities.
mod motion_planner;

/// The module for motion interpolation functionalities.
mod motion_interpolation;

/// The module for motion timing functionalities.
mod motion_timing;

/// The module for motion time driver functionalities.
mod motion_time_driver;

pub use motion_config::*;
pub use motion_planner::*;
pub use motion_interpolation::*;
pub use motion_segment::*;
pub use motion_status::*;
pub use motion_timing::*;
pub use motion_time_driver::*;
use crate::hwa;

/// Represents a scheduled move in the motion system.
pub enum ScheduledMove {
    /// A movement segment.
    Move(SegmentData),
    /// A homing action.
    Homing,
    /// A dwell action.
    Dwell,
}

/// Types of movements in the motion system.
#[derive(Clone, Copy)]
pub enum MovType {
    /// A normal move with deferred action and communication channel.
    Move(hwa::DeferAction, hwa::CommChannel),
    /// A homing move with a communication channel.
    Homing(hwa::CommChannel),
    /// A dwell action with a communication channel.
    Dwell(hwa::CommChannel),
}

/// Represents an entry in the motion plan.
#[derive(Clone, Copy)]
pub enum PlanEntry {
    /// An empty plan entry.
    Empty,
    /// A planned move tuple.
    ///
    /// *_1: Segment* - The motion segment.
    ///
    /// *_2: CommChannel* - The input channel requesting the move.
    ///
    /// *_3: bool* - Indicates if motion is deferred or not.
    PlannedMove(Segment, hwa::DeferAction, hwa::CommChannel, bool),
    /// A homing action request.
    ///
    /// *_1: CommChannel* - The input channel requesting the move.
    ///
    /// *_2: bool* - Indicates if motion is deferred or not.
    Homing(hwa::CommChannel, bool),
    /// A Dwell action request.
    ///
    /// *_1: CommChannel* - The input channel requesting the move.
    ///
    /// *_2: bool* - Indicates if motion is deferred or not.
    Dwell(hwa::CommChannel, bool),
    /// An executing move.
    ///
    /// *_1: MovType* - The type of the move.
    ///
    /// *_2: bool* - Indicates if motion is deferred or not.
    Executing(MovType, bool),
}

/// Provides the default value for `PlanEntry`, which is `PlanEntry::Empty`.
impl Default for PlanEntry {
    fn default() -> Self {
        PlanEntry::Empty
    }
}
