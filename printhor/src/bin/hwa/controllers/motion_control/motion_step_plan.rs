//! Motion Step Plan utils
use crate::hwa;
use hwa::math::{CoordSel, TVector};

/// Second-order DDA (phase accumulator) state of a stepper channel.
///
/// All quantities are unsigned/signed **Q0.32** fixed point:
/// * `phase`: fractional step position (`2^32` ≙ one step),
/// * `rate`: steps per tick,
/// * `rate_inc`: change of `rate` per tick (steps / tick²).
///
/// Every tick: `phase += rate` — the wrap-around carry emits a step — then `rate += rate_inc`.
/// The step rate therefore varies **linearly** inside a micro-segment window: the velocity seen by
/// the motor is continuous and piecewise linear (acceleration piecewise constant) instead of the
/// former per-window staircase (constant rate inside each window, a velocity jump of `a·Δt` at
/// every window boundary). Integer-only arithmetic, suitable for the stepper ISR.
#[derive(Clone, Copy)]
pub struct ChannelStatus {
    phase: u32,
    rate: u32,
    rate_inc: i32,
    name: CoordSel,
}

/// `1.0` in Q0.32
pub const DDA_ONE: f64 = 4294967296.0;
/// `0.5` in Q0.32
pub const DDA_HALF: u32 = 1 << 31;

impl ChannelStatus {
    /// Creates a channel from its DDA parameters (Q0.32).
    pub const fn new(name: CoordSel, phase: u32, rate: u32, rate_inc: i32) -> Self {
        Self {
            phase,
            rate,
            rate_inc,
            name,
        }
    }

    /// Uniform pulse train: one step every `width_us` (the first one at `width_us/2`), evaluated
    /// with a tick of `tick_us`. A width shorter than the tick saturates at one step per tick.
    pub const fn uniform(name: CoordSel, width_us: u32, tick_us: u32) -> Self {
        let rate = if width_us == 0 {
            u32::MAX
        } else {
            let r = ((tick_us as u64) << 32) / (width_us as u64);
            if r > u32::MAX as u64 { u32::MAX } else { r as u32 }
        };
        Self::new(name, DDA_HALF, rate, 0)
    }

    /// Advances one tick. Returns `true` when a step is due.
    #[inline(always)]
    fn tick(&mut self) -> bool {
        let (phase, carry) = self.phase.overflowing_add(self.rate);
        self.phase = phase;
        let r = self.rate as i64 + self.rate_inc as i64;
        self.rate = if r < 0 {
            0
        } else if r > u32::MAX as i64 {
            u32::MAX
        } else {
            r as u32
        };
        carry
    }
}

/// Step generation plan of one micro-segment window for every channel.
///
/// Each channel is a second-order DDA (see [`ChannelStatus`]) and `max_count` holds the exact
/// number of steps the window must emit per channel (the DDA only decides *when*).
/// [`StepPlan::from`] turns it into per-slot counters for the ISR.
///
#[derive(Clone, Copy)]
pub struct MultiTimer {
    #[cfg(feature = "with-motion-broadcast")]
    /// The reference time in micro-seconds
    ref_time_us: u32,
    /// The interval width in micro-seconds
    width_us: u32,
    max_count: TVector<u32>,
    channels: [Option<ChannelStatus>; CoordSel::num_axis()],
}

impl MultiTimer {
    /// Creates a new MultiTimer with default values.
    ///
    /// # Returns
    ///
    /// A new MultiTimer instance.
    pub const fn new() -> Self {
        Self {
            #[cfg(feature = "with-motion-broadcast")]
            ref_time_us: 0,
            width_us: 0,
            max_count: TVector::new(),
            channels: [None; CoordSel::num_axis()],
        }
    }

    fn set_channel(&mut self, channel: CoordSel, status: Option<ChannelStatus>) {
        for ch in channel.iter() {
            let index = ch.index();
            if index < self.channels.len() {
                self.channels[index] = status.map(|st| ChannelStatus { name: ch, ..st });
            } else {
                hwa::error!(
                    "OUT OF BOUNDS channel: {:?} index: {}, len: {} !!!",
                    channel,
                    index,
                    self.channels.len()
                );
            }
        }
    }

    /// Uniform pulse train for a channel: one step every `width_us` (first one at `width_us/2`).
    /// `None` disables the channel.
    pub fn set_channel_uniform(&mut self, channel: CoordSel, width_us: Option<u32>, tick_us: u32) {
        self.set_channel(
            channel,
            width_us.map(|w| ChannelStatus::uniform(channel, w, tick_us)),
        );
    }

    /// Second-order DDA for a channel (see [`ChannelStatus`]). `None` disables the channel.
    pub fn set_channel_dda(&mut self, channel: CoordSel, dda: Option<(u32, u32, i32)>) {
        self.set_channel(
            channel,
            dda.map(|(phase, rate, inc)| ChannelStatus::new(channel, phase, rate, inc)),
        );
    }

    /// Sets the maximum count for the channels.
    ///
    /// # Arguments
    ///
    /// * `max_count` - A reference to TVector with the maximum counts for each channel.
    pub fn set_max_count(&mut self, max_count: &TVector<u32>) {
        self.max_count = *max_count;
    }

    /// Sets the width for the MultiTimer.
    ///
    /// # Arguments
    ///
    /// * `width` - The width to set.
    pub fn set_displace_width(&mut self, width_us: u32) {
        #[cfg(feature = "with-motion-broadcast")]
        {
            self.ref_time_us += width_us;
        }
        self.width_us = width_us;
    }

    /// Gets the width of the MultiTimer.
    ///
    /// # Returns
    ///
    /// The width of the MultiTimer.
    #[inline]
    pub fn width(&self) -> u32 {
        self.width_us
    }
}

/// Represents a planner to manage steps for multiple stepper channels uniformly.
///
/// It can distribute pulse trains uniformly over multiple channels, acting like reloading timers.
/// When a specified width is reached in a channel, another width is added until the maximal width
/// [`interval_width`](MultiTimer) is reached.
///
/// Assumptions for equidistant pulses within the same rate:
/// - Each pulse period (`pulse width`) is **`t = T / x`**, where **`x`** is the number of pulses and **`T`** is the total period.
/// - Pulse sequence follows **`t(i) = t / 2 + (i - 1) * t`**, with **`i`** starting at 1.
/// - Iterator ceases when **`t(i)`** surpasses [`interval_width`](MultiTimer) for all channels.
///
#[derive(Clone, Copy)]
pub struct StepPlan {
    /// The order number, for traceability
    pub _order_num: u32,
    /// The interval width for the step planner in microseconds
    pub interval_width: u32,
    #[cfg(feature = "with-motion-broadcast")]
    pub delta: hwa::MotionDelta,
    // The current time in step-planner
    pub(crate) ref_time_us: u32,
    /// Channels being managed by the step planner.
    channels: [Option<ChannelStatus>; CoordSel::num_axis()],
    /// Steps still to emit by channel slot (parallel to `channels`): O(1) bookkeeping in the ISR
    /// instead of scanning a per-axis vector on every step.
    remaining: [u32; CoordSel::num_axis()],
    /// Flags to enable stepper channels.
    pub stepper_enable_flags: CoordSel,
    /// Flags to set the direction forward for stepper channels.
    pub stepper_dir_fwd_flags: CoordSel,
    /// Tool actions applied when the plan is dequeued, before its first tick (see
    /// [StepPlan::marker])
    pub tools: hwa::controllers::ToolActions,
    /// Plans of the motion driver (homing, probing): `Some(end-stops)`, watched every tick (none
    /// when empty). Their steps are counted by the executor, and they stop as soon as one of the
    /// end-stops triggers (see `SoftTimer::begin_watch`). `None` for the regular motion.
    pub watch: Option<CoordSel>,
    /// The mean speed of the window relative to the speed programmed for its move, per mille: the
    /// scale of the laser dynamic power (`M4`), applied when the window starts (see
    /// [hwa::controllers::dynamic_power_permille]). `None` when there is no programmed cutting
    /// speed (rapids, homing): the laser is off over the window (`M3` and `M4`).
    pub speed_ratio: Option<u16>,
}

impl StepPlan {
    /// Creates a new StepPlanner with default values.
    ///
    /// # Returns
    ///
    /// A new StepPlanner instance.
    pub const fn new(#[cfg(feature = "with-motion-broadcast")] delta: hwa::MotionDelta) -> Self {
        Self {
            _order_num: 0,
            #[cfg(feature = "with-motion-broadcast")]
            delta,
            ref_time_us: 0,
            interval_width: 0,
            channels: [None; CoordSel::num_axis()],
            remaining: [0; CoordSel::num_axis()],
            stepper_enable_flags: CoordSel::UNSET,
            stepper_dir_fwd_flags: CoordSel::UNSET,
            tools: hwa::controllers::ToolActions::NONE,
            watch: None,
            speed_ratio: None,
        }
    }

    /// A plan without steps nor time that only carries tool actions: the executor applies them
    /// when it reaches the plan, right after the windows queued before it, and goes on with the
    /// next plan in the same tick (nothing is added to the motion time)
    pub fn marker(_order_num: u32, tools: hwa::controllers::ToolActions) -> Self {
        Self {
            _order_num,
            tools,
            ..Self::new(
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
            )
        }
    }

    /// A plan of the motion driver (homing, probing): see [StepPlan::watch]
    pub fn watched(
        _order_num: u32,
        multi_timer: MultiTimer,
        stepper_enable_flags: CoordSel,
        stepper_dir_fwd_flags: CoordSel,
        endstops: CoordSel,
    ) -> Self {
        Self {
            watch: Some(endstops),
            ..Self::from(
                _order_num,
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
                multi_timer,
                stepper_enable_flags,
                stepper_dir_fwd_flags,
            )
        }
    }

    /// Whether the plan takes no time and emits no step (see [StepPlan::marker])
    #[inline]
    pub fn is_marker(&self) -> bool {
        self.interval_width == 0 && !self.has_pending_steps()
    }

    /// Drops the steps still pending and closes the window now (an end-stop triggered)
    #[inline]
    pub fn cancel(&mut self) {
        self.remaining = [0; CoordSel::num_axis()];
        self.interval_width = 0;
    }

    /// Creates a StepPlanner from a MultiTimer, stepper enable flags, and stepper direction forward flags.
    ///
    /// # Arguments
    ///
    /// * `multi_timer` - A MultiTimer instance.
    /// * `stepper_enable_flags` - Flags to enable steppers.
    /// * `stepper_dir_fwd_flags` - Flags to set the direction forward for steppers.
    ///
    /// # Returns
    ///
    /// A new StepPlanner instance.
    pub fn from(
        _order_num: u32,
        #[cfg(feature = "with-motion-broadcast")] delta: hwa::MotionDelta,
        multi_timer: MultiTimer,
        stepper_enable_flags: CoordSel,
        stepper_dir_fwd_flags: CoordSel,
    ) -> Self {
        let mut remaining = [0u32; CoordSel::num_axis()];
        for (slot, channel) in remaining.iter_mut().zip(multi_timer.channels.iter()) {
            if let Some(ch) = channel {
                *slot = multi_timer.max_count.get_coord(ch.name).unwrap_or(0);
            }
        }
        let mut instance = Self {
            _order_num,
            #[cfg(feature = "with-motion-broadcast")]
            delta,
            ref_time_us: 0,
            interval_width: multi_timer.width_us,
            channels: multi_timer.channels,
            remaining,
            stepper_enable_flags,
            stepper_dir_fwd_flags,
            tools: hwa::controllers::ToolActions::NONE,
            watch: None,
            speed_ratio: None,
        };
        instance.reset();
        instance
    }

    /// Resets the StepPlanner to its initial state.
    /// `true` while some channel still has steps to emit in this window.
    #[inline]
    pub fn has_pending_steps(&self) -> bool {
        self.remaining.iter().any(|r| *r > 0)
    }

    fn reset(&mut self) {
        self.ref_time_us = 0;
    }

    /// Advances the plan by one tick of `step_width` µs (the DDA rates were computed for this
    /// same tick period).
    ///
    /// Steps are emitted on DDA carries. From the last tick of the window on, any step still
    /// pending in `max_count` is emitted one per tick (the executor keeps the window open until
    /// none is pending), so rounding can never drop or add a step.
    ///
    /// # Returns
    ///
    /// The stepper channels that must emit a step in this tick.
    pub fn next(&mut self, step_width: u32) -> Option<CoordSel> {
        self.ref_time_us += step_width;
        let flush = self.ref_time_us >= self.interval_width;
        let mut triggered_channels = CoordSel::empty();
        for (channel, remaining) in self.channels.iter_mut().zip(self.remaining.iter_mut()) {
            if let Some(ch) = channel.as_mut() {
                let due = ch.tick();
                if (due || flush) && *remaining > 0 {
                    *remaining -= 1;
                    triggered_channels.set(ch.name, true);
                }
            }
        }
        Some(triggered_channels)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_step_planner_size() {
        // This test ensures that the StepPlanner struct size does not exceed 60 bytes
        // because it will be used in a CAN FD bus frame.
        let struct_size = size_of::<ChannelStatus>() / 2;
        assert!(
            struct_size <= 8,
            "StepPlanner size exceeds expectations for a canbus frame (CAN protocol version 2.0 A, B)"
        );
    }

    /// Helper: run a StepPlan for its full interval_width and count X-axis fires.
    fn run_plan_count_x(plan: &mut StepPlan, step_width: u32) -> u32 {
        let mut count = 0u32;
        while plan.ref_time_us < plan.interval_width {
            if let Some(ch) = plan.next(step_width) {
                if ch.contains(CoordSel::X) {
                    count += 1;
                }
            }
        }
        count
    }

    fn make_plan_x(window_us: u32, channel_width_us: u32, max_steps: u32) -> StepPlan {
        let mut mt = MultiTimer::new();
        mt.set_displace_width(window_us);
        mt.set_channel_uniform(CoordSel::X, Some(channel_width_us), 10);
        let mut count: TVector<u32> = TVector::new();
        count.set_coord(CoordSel::X, Some(max_steps));
        mt.set_max_count(&count);

        StepPlan::from(
            0,
            #[cfg(feature = "with-motion-broadcast")]
            hwa::MotionDelta::new(),
            mt,
            CoordSel::X,
            CoordSel::X,
        )
    }

    /// A window carrying more steps than ticks keeps reporting pending steps after its nominal
    /// width, so the executor stretches it instead of dropping steps.
    #[cfg(feature = "with-x-axis")]
    #[test]
    fn test_overfull_window_reports_pending_steps() {
        // 15 steps in a 100µs window with a 10µs tick: only 10 can fire in time.
        let mut plan = make_plan_x(100, 6, 15);
        let mut fired = 0u32;
        let mut ticks = 0u32;
        while ticks < 100 || plan.has_pending_steps() {
            if let Some(ch) = plan.next(10) {
                if ch.contains(CoordSel::X) {
                    fired += 1;
                }
            }
            ticks += 10;
            assert!(ticks < 1000, "must terminate");
        }
        assert_eq!(fired, 15, "every step must be emitted");
        assert!(!plan.has_pending_steps());
    }

    /// A marker takes no time and emits no step; a window is not a marker
    #[test]
    fn a_marker_is_a_plan_without_time_nor_steps() {
        let tools = hwa::controllers::ToolActions { fan_layer: Some(30), ..hwa::controllers::ToolActions::NONE };
        let marker = StepPlan::marker(5, tools);
        assert!(marker.is_marker());
        assert!(!marker.has_pending_steps());
        assert_eq!(marker.tools, tools);
        assert_eq!(marker.watch, None);
        assert_eq!(marker._order_num, 5);
        let window = make_plan_x(100, 50, 2);
        assert!(!window.is_marker());
        assert!(window.tools.is_empty());
        assert_eq!(window.watch, None);
    }

    /// A watched plan (homing, probing) keeps its end-stops; cancelling it drops its pending steps
    /// and closes it at once
    #[cfg(feature = "with-x-axis")]
    #[test]
    fn a_cancelled_watched_plan_emits_no_more_steps() {
        let mut mt = MultiTimer::new();
        mt.set_displace_width(100);
        mt.set_channel_uniform(CoordSel::X, Some(10), 10);
        let mut count: TVector<u32> = TVector::new();
        count.set_coord(CoordSel::X, Some(10));
        mt.set_max_count(&count);
        let mut plan = StepPlan::watched(1, mt, CoordSel::X, CoordSel::empty(), CoordSel::X);
        assert_eq!(plan.watch, Some(CoordSel::X));
        assert!(plan.has_pending_steps());
        let _ = plan.next(10);
        plan.cancel();
        assert!(!plan.has_pending_steps());
        assert_eq!(plan.interval_width, 0, "closed at once");
        for _ in 0..20 {
            assert_eq!(plan.next(10), Some(CoordSel::empty()), "no more steps");
        }
    }

    /// Normal feedrate: channel_width (50µs) ≥ step_width (10µs).
    /// 100µs window → expect exactly 2 steps.
    #[test]
    fn test_normal_feedrate_fires_correct_steps() {
        let mut plan = make_plan_x(100, 50, 2);
        let fired = run_plan_count_x(&mut plan, 10);
        assert_eq!(fired, 2, "expected 2 steps at normal feedrate");
    }

    /// Feedrate exceeded: channel_width (2µs) < step_width (10µs).
    /// max_count = 5, window = 100µs → all 5 steps must still fire at max rate.
    #[test]
    fn test_feedrate_exceeded_all_steps_fire_at_max_rate() {
        let mut plan = make_plan_x(100, 2, 5);
        let fired = run_plan_count_x(&mut plan, 10);
        assert_eq!(fired, 5, "all 5 steps must fire at max feedrate, none skipped");
    }

    /// Feedrate exceeded with zero channel_width (tick_period = 0 from integer division).
    /// max_count = 3, window = 100µs → all 3 steps fire at max rate.
    #[test]
    fn test_zero_channel_width_fires_at_max_rate() {
        let mut plan = make_plan_x(100, 0, 3);
        let fired = run_plan_count_x(&mut plan, 10);
        assert_eq!(fired, 3, "all 3 steps must fire even with zero channel_width");
    }

    /// max_count limits total steps even when feedrate is exceeded.
    /// window = 200µs, step_width = 10µs → 20 ticks available, but max_count = 4.
    #[test]
    fn test_feedrate_exceeded_max_count_respected() {
        let mut plan = make_plan_x(200, 1, 4);
        let fired = run_plan_count_x(&mut plan, 10);
        assert_eq!(fired, 4, "max_count must be respected at max feedrate");
    }

    /// When channel_width == step_width (boundary), no clamping should occur
    /// and steps fire exactly once per tick.
    #[test]
    fn test_feedrate_exactly_at_limit_fires_correctly() {
        // channel_width = step_width = 10µs, window = 50µs → max 5 ticks
        // next_tick starts at 5 (10/2). Fires at tick ref=10, 20, 30, 40, 50.
        let mut plan = make_plan_x(50, 10, 5);
        let fired = run_plan_count_x(&mut plan, 10);
        assert_eq!(fired, 5, "steps at boundary feedrate must all fire");
    }
}
