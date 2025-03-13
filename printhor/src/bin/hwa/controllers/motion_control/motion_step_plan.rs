//! Motion Step Plan utils
use crate::hwa;
use hwa::math::{CoordSel, TVector};

/// Represents the status of a stepper channel.
/// This struct maintains the status for a specific stepper channel,
/// including the next tick time, the width of the channel, and its name.
///
/// # Fields
///
/// * `next_tick` - The next tick time for the stepper channel.
/// * `width` - The width (time) of the stepper channel in ticks.
/// * `name` - The name of the stepper channel.
#[derive(Clone, Copy)]
pub struct ChannelStatus {
    next_tick: u32,
    width: u32,
    name: CoordSel,
}

impl ChannelStatus {
    /// Creates a new ChannelStatus with the given name and width.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the stepper channel.
    /// * `width` - The width of the stepper channel.
    ///
    /// # Returns
    ///
    /// A new ChannelStatus instance.
    pub const fn new(name: CoordSel, width: u32) -> Self {
        Self {
            next_tick: 0,
            width,
            name,
        }
    }
}

/// A utility to feed forward a uniformly distributed pulse train at different rates by channel.
///
/// Basically, it works like a set of reloading timers. When a (time) width is reached in a channel,
/// another (time) width is added. Iterator ends when it reaches its maximal width [`interval_width`](MultiTimer::new()).
///
/// In order for the pulses to be equidistant within the same rate; assuming **`x`** as number of pulses and **`T`** the period to uniform distribute them:
/// - Each pulse period (pulse width) is: **`t=T/x`**.
/// - Pulse sequence is **`t(i) = t/2 + (i-1)*t`** with **`i`** starting at 1
/// - Iterator ends when **`t(i)`** exceed [`interval_width`](MultiTimer::new()) in all channels.
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

    /// Sets the ticks for a specific channel.
    ///
    /// # Arguments
    ///
    /// * `channel` - The stepper channel to set the ticks for.
    /// * `ticks` - The optional number of ticks to set. If `None`, the channel is disabled.
    pub fn set_channel_ticks(&mut self, channel: CoordSel, ticks: Option<u32>) {
        for ch in channel.iter() {
            let index = ch.index();
            if index < self.channels.len() {
                match ticks {
                    Some(_t) => {
                        self.channels[index] = Some(ChannelStatus::new(ch, _t));
                    }
                    None => {
                        self.channels[index] = None;
                    }
                }
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
    /// Maximum count vector for each channel.
    pub max_count: TVector<u32>,
    /// Flags to enable stepper channels.
    pub stepper_enable_flags: CoordSel,
    /// Flags to set the direction forward for stepper channels.
    pub stepper_dir_fwd_flags: CoordSel,
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
            max_count: TVector::new(),
            channels: [None; CoordSel::num_axis()],
            stepper_enable_flags: CoordSel::UNSET,
            stepper_dir_fwd_flags: CoordSel::UNSET,
        }
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
        let mut instance = Self {
            _order_num,
            #[cfg(feature = "with-motion-broadcast")]
            delta,
            ref_time_us: 0,
            interval_width: multi_timer.width_us,
            max_count: multi_timer.max_count,
            channels: multi_timer.channels,
            stepper_enable_flags,
            stepper_dir_fwd_flags,
        };
        instance.reset();
        instance
    }

    /// Resets the StepPlanner to its initial state.
    fn reset(&mut self) {
        self.ref_time_us = 0;
        for channel in self.channels.iter_mut() {
            match channel.as_mut() {
                None => {}
                Some(w) => w.next_tick = (w.width) >> 1,
            }
        }
    }

    /// Advances the StepPlanner by the given step width.
    ///
    /// # Arguments
    ///
    /// * `step_width` - The width of the step to advance.
    ///
    /// # Returns
    ///
    /// An Option containing the stepper channels that were triggered in this step.
    pub fn next(&mut self, step_width: u32) -> Option<CoordSel> {
        self.ref_time_us += step_width;
        let mut triggered_channels = CoordSel::empty();
        for channel in self.channels.iter_mut() {
            if let Some(_ch) = channel.as_mut() {
                if _ch.next_tick <= self.ref_time_us {
                    if _ch.width < step_width {
                        cfg_if::cfg_if! {
                            if #[cfg(not(feature = "with-motion-broadcast"))] {
                                hwa::error!(
                                    "Feedrate exceeded: width: {} min: {}",
                                    _ch.width, step_width
                                );
                            }
                        }
                    }
                    _ch.next_tick += _ch.width;
                    let coord: CoordSel = _ch.name.into();
                    if self.max_count.decrement_if_positive(coord) {
                        triggered_channels.set(_ch.name, true);
                    }
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
}
