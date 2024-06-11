use printhor_hwa_common::StepperChannel;
use crate::tgeo::{CoordSel, TVector};

#[derive(Clone, Copy)]
pub struct ChannelStatus {
    next_tick: u32,
    width: u32,
    name: StepperChannel,
}

impl ChannelStatus {
    pub const fn new(name: StepperChannel, width: u32) -> Self {
        Self{
            next_tick: 0,
            width,
            name,
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis", feature = "with-e-axis"))] {
        pub const MULTITIMER_CHANNELS: usize = 4;
    }
    else if #[cfg(all(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis"))] {
        pub const MULTITIMER_CHANNELS: usize = 3;
    }
    else if #[cfg(all(feature = "with-z-axis"))]  {
        pub const MULTITIMER_CHANNELS: usize = 1;
    }
    else {
        compile_error!("Unsupported axis configuration");
    }
}

/// A utility to feed forward a uniformly distributed pulse train at different rates by channel
///
/// Basically, it works like a set of reloading timers. When a (time) width is reached in a channel,
/// another (time) width is added. Iterator ends when it reach it's maximal width [`interval_width`](MultiTimer::new())
///
/// In order for the pulses to be equidistant within the same rate; assuming **`x`** as number of pulses and **`T`** the period to uniform distribute them:
/// - Each pulse period (pulse width) is: **`t=T/x`**.
/// - Pulse sequence is **`t(i) = t/2 + (i-1)*t`** with **`i`** starting at 1
/// - Iterator ends when **`t(i)`** exceed [`interval_width`](MultiTimer::new()) in all channels.
///
#[derive(Clone, Copy)]
pub struct MultiTimer {
    width: u32,
    max_count: TVector<u32>,
    channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
}

impl MultiTimer {
    pub const fn new() -> Self {
        Self {
            width: 0,
            max_count: TVector::new(),
            channels: [
                #[cfg(feature = "with-x-axis")]
                None,
                #[cfg(feature = "with-y-axis")]
                None,
                #[cfg(feature = "with-z-axis")]
                None,
                #[cfg(feature = "with-e-axis")]
                None,
            ],
        }
    }

    pub fn set_channel_ticks(&mut self, channel: StepperChannel, ticks: Option<u32>) {

        #[cfg(feature = "with-x-axis")]
        if channel.contains(StepperChannel::X) {
            match ticks {
                Some(_t) => {
                    self.channels[0] = Some(ChannelStatus::new(StepperChannel::X, _t));
                }
                None => {
                    self.channels[0] = None;
                }
            }
        }
        #[cfg(feature = "with-y-axis")]
        if channel.contains(StepperChannel::Y) {
            match ticks {
                Some(_t) => {
                    self.channels[1] = Some(ChannelStatus::new(StepperChannel::Y, _t));
                }
                None => {
                    self.channels[1] = None;
                }
            }
        }
        #[cfg(feature = "with-z-axis")]
        if channel.contains(StepperChannel::Z) {
            match ticks {
                Some(_t) => {
                    self.channels[2] = Some(ChannelStatus::new(StepperChannel::Z, _t));
                }
                None => {
                    self.channels[2] = None;
                }
            }
        }
        #[cfg(feature = "with-e-axis")]
        if channel.contains(StepperChannel::E) {
            match ticks {
                Some(_t) => {
                    self.channels[3] = Some(ChannelStatus::new(StepperChannel::E, _t));
                }
                None => {
                    self.channels[3] = None;
                }
            }
        }
    }

    pub fn set_max_count(&mut self, max_count: &TVector<u32>) {
        self.max_count = *max_count;
    }

    pub fn set_width(&mut self, width: u32) {
        self.width = width;
    }

    pub fn width(&self) -> u32 {
        self.width
    }
}

/// A utility to feed forward a uniformly distributed pulse train at different rates by channel
///
/// Basically, it works like a set of reloading timers. When a (time) width is reached in a channel,
/// another (time) width is added. Iterator ends when it reach it's maximal width [`interval_width`](crate::control::motion_timing::MultiTimer::new())
///
/// In order for the pulses to be equidistant within the same rate; assuming **`x`** as number of pulses and **`T`** the period to uniform distribute them:
/// - Each pulse period (pulse width) is: **`t=T/x`**.
/// - Pulse sequence is **`t(i) = t/2 + (i-1)*t`** with **`i`** starting at 1
/// - Iterator ends when **`t(i)`** exceed [`interval_width`](crate::control::motion_timing::MultiTimer::new()) in all channels.
///
#[derive(Clone, Copy)]
pub struct StepPlanner {
    pub(crate) interval_width: u32,
    ref_time: u32,
    channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
    max_count: TVector<u32>,
    pub stepper_enable_flags: StepperChannel,
    pub stepper_dir_fwd_flags: StepperChannel,
}

impl crate::control::motion_timing::StepPlanner {
    pub const fn new() -> Self {
        Self {
            interval_width: 0,
            max_count: TVector::new(),
            ref_time: 0,
            channels: [
                #[cfg(feature = "with-x-axis")]
                None,
                #[cfg(feature = "with-y-axis")]
                None,
                #[cfg(feature = "with-z-axis")]
                None,
                #[cfg(feature = "with-e-axis")]
                    None,
            ],
            stepper_enable_flags: StepperChannel::UNSET,
            stepper_dir_fwd_flags: StepperChannel::UNSET,
        }
    }

    pub fn from(multi_timer: MultiTimer,
                      stepper_enable_flags: StepperChannel,
                      stepper_dir_fwd_flags: StepperChannel,

    ) -> Self {
        let mut instance = Self {
            interval_width: multi_timer.width,
            max_count: multi_timer.max_count,
            ref_time: 0,
            channels: multi_timer.channels,
            stepper_enable_flags,
            stepper_dir_fwd_flags,
        };
        instance.reset();
        instance
    }

    fn reset(&mut self) {
        self.ref_time = 0;
        for channel in self.channels.iter_mut() {
            match channel.as_mut() {
                None => {
                }
                Some(w) => {
                    w.next_tick = (w.width) >> 1
                }
            }
        }
    }

    pub fn next(&mut self, step_width: u32) -> Option<StepperChannel> {
        self.ref_time += step_width;
        let mut triggered_channels = StepperChannel::empty();
        for channel in self.channels.iter_mut() {
            if let Some(_ch) = channel.as_mut() {
                if _ch.next_tick <= self.ref_time  {
                    if _ch.width < step_width {
                        unreachable!("Feedrate exceeded: width: {} max: {}", _ch.width, step_width);
                    }
                    _ch.next_tick += _ch.width;
                    let coord: CoordSel = _ch.name.into();
                    if self.max_count.decrement_if_positive(coord) {
                        triggered_channels.set(_ch.name, true);
                    }
                }
            }
        }
        return Some(triggered_channels);
    }
}
