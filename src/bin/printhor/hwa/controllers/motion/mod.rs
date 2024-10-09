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

use crate::hwa;
pub use motion_config::*;
pub use motion_interpolation::*;
pub use motion_planner::*;
pub use motion_segment::*;
pub use motion_status::*;
pub use motion_time_driver::*;
pub use motion_timing::*;

pub(in crate::hwa) use motion_ring_buffer::RingBuffer;

/// Represents a scheduled move in the motion system.
pub enum ScheduledMove {
    /// A movement segment.
    Move(SegmentData),
    /// A homing action.
    Homing,
    /// A dwell action.
    Dwell(Option<u32>),
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
    /// *_2: Option<u32>* - The number of milliseconds to delay.
    ///
    /// *_3: bool* - Indicates if motion is deferred or not.
    Dwell(hwa::CommChannel, Option<u32>, bool),
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

#[cfg(test)]
mod test {
    use crate::hwa::controllers::PlanEntry;

    #[test]
    fn plan_entry_test() {
        let sz = size_of::<PlanEntry>();
        assert!(
            sz < 512,
            "Plan entry is not so big ({} bytes. max: 512)",
            sz
        );
    }
}
