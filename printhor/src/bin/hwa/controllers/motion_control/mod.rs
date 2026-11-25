//! The module for motion management (configuration, state, parameters, constraints, software elements, devices and drivers)
mod motion_config;

/// The module to model a motion segment instance.
mod motion_segment;

/// The module to hold the real time motion status (positions mainly).
mod motion_status;

/// The module to hold the planned segment ring-buffer.
pub(crate) mod motion_ring_buffer;

/// The module with the motion planner logic.
mod motion_planner;

/// The module for motion timing (formally: the StepPlan generator).
mod motion_step_plan;

/// The module to provide a software time driver to schedule multi-axis steps with variable timings each at fixed rate
mod motion_step_plan_executor;

/// The module to drive the step pins.
mod motion_step_actuator;

/// Motion execution metrics (feature `motion-metrics`, reported by `M800`)
pub mod motion_metrics;

/// The tool actions (fans, laser/spindle, coolant) queued with the motion.
mod tool_actions;

use crate::hwa;
pub use motion_config::*;
pub use motion_planner::*;
pub use motion_segment::*;
pub use motion_status::*;
pub use motion_step_actuator::*;
pub use motion_step_plan::*;
pub use motion_step_plan_executor::*;
pub use tool_actions::*;

pub(in crate::hwa) use motion_ring_buffer::RingBuffer;

/// What a homing (`G28`) does, decided when it is planned (see [MotionPlanner::plan_homing]):
/// the axes it homes and where it leaves the machine.
///
/// `G28` alone (or with every axis) is the full homing. With some axes (Marlin's semantics, with
/// `Z_SAFE_HOMING`): Z is raised first (never lowered), the given X/Y axes are homed and left at
/// their end-stops, and Z (when given) is sampled at the safe homing point, where the machine is
/// left as after the full homing (the probe over the bed center).
#[derive(Clone, Copy, Debug)]
pub struct HomingPlan {
    /// The axes homed. Every homing axis for the full homing
    pub axes: hwa::CoordSel,
    /// The full homing: `G28` without axes, or with all of them
    pub full: bool,
    /// The native (machine) position before the homing: only the known coordinates
    pub start: hwa::math::TVector<hwa::math::Real>,
    /// The native Z to raise to before homing X and Y (never lower), when Z is known. Otherwise
    /// it is raised `HOMING_Z_RAISE_MM` from where it is (unknown)
    pub z_clearance: Option<hwa::math::Real>,
    /// The native position after the homing: only the known coordinates
    pub end: hwa::math::TVector<hwa::math::Real>,
    /// The axes whose position comes from a homing, afterwards
    pub homed: hwa::CoordSel,
}

impl HomingPlan {
    /// The axes the homing finds (by their end-stop, or the probe): X, Y and Z
    pub const fn homing_axes() -> hwa::CoordSel {
        #[allow(unused_mut)]
        let mut axes = hwa::CoordSel::empty();
        #[cfg(feature = "with-x-axis")]
        {
            axes = axes.union(hwa::CoordSel::X);
        }
        #[cfg(feature = "with-y-axis")]
        {
            axes = axes.union(hwa::CoordSel::Y);
        }
        #[cfg(feature = "with-z-axis")]
        {
            axes = axes.union(hwa::CoordSel::Z);
        }
        axes
    }

    /// Whether the whole position is known afterwards (the moves are allowed)
    pub fn position_known(&self) -> bool {
        self.full || self.homed.contains(Self::homing_axes())
    }
}

/// Represents a scheduled move in the motion system.
pub enum ScheduledMove {
    /// A movement segment.
    Move(Segment, u32),
    /// A homing action.
    Homing(u32, HomingPlan),
    /// A dwell action.
    Dwell(Option<u32>, u32),
    /// A set-position action.
    SetPosition(Position, u32),
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
    /// A set-position action with a communication channel.
    SetPosition(hwa::CommChannel),
    /// Tool actions queued alone (see [PlanEntry::Tools]).
    Tools(hwa::CommChannel),
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
    /// *_2: DeferAction* - The defer action.
    ///
    /// *_3: CommChannel* - The input channel requesting the move.
    ///
    /// *_4: bool* - Indicates if motion is deferred or not.
    ///
    /// *_5: u32* - The motion id (sequential number, for tracceability)
    PlannedMove(Segment, hwa::DeferAction, hwa::CommChannel, bool, u32),
    /// A homing action request.
    ///
    /// *_1: CommChannel* - The input channel requesting the move.
    ///
    /// *_2: bool* - Indicates if motion is deferred or not.
    ///
    /// *_4: HomingPlan* - What the homing does.
    Homing(hwa::CommChannel, bool, u32, HomingPlan),
    /// A Dwell action request.
    ///
    /// *_1: CommChannel* - The input channel requesting the action.
    ///
    /// *_2: Option&lt;u32&gt;* - The number of milliseconds to delay.
    ///
    /// *_3: bool* - Indicates if motion is deferred or not.
    Dwell(hwa::CommChannel, Option<u32>, bool, u32),

    /// A SetPosition action request.
    ///
    /// *_1: CommChannel* - The input channel requesting the action.
    ///
    /// *_2: Position* - The position to set.
    ///
    /// *_3: bool* - Indicates if motion is deferred or not.
    SetPosition(hwa::CommChannel, Position, bool, u32),

    /// Tool actions requested while the queue was empty: no motion. The actions themselves are
    /// kept in `RingBuffer::post_tools`, as those requested after any other entry (see
    /// `MotionPlanner::schedule_tool_action`).
    ///
    /// *_1: CommChannel* - The input channel requesting the action.
    ///
    /// *_2: u32* - The order number.
    Tools(hwa::CommChannel, u32),

    /// An executing move.
    ///
    /// *_1: MovType* - The type of the move.
    ///
    /// *_2: bool* - Indicates if motion is deferred or not.
    Executing(MovType, bool, u32),
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
            sz < 1024,
            "Plan entry is not so big ({} bytes. max: 1024)",
            sz
        );
    }
}
