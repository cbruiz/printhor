use crate::control::motion_planning::StepperChannel;

#[derive(Clone, Copy)]
pub struct ChannelStatus {
    next_tick: u64,
    width: u64,
    name: StepperChannel,
}

impl ChannelStatus {
    pub const fn new(name: StepperChannel, width: u64) -> Self {
        Self{
            next_tick: 0,
            width,
            name,
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        pub const MULTITIMER_CHANNELS: usize = 4;
    }
    else {
        pub const MULTITIMER_CHANNELS: usize = 3;
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
    max_count: u32,
    channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
}

impl MultiTimer {
    pub const fn new() -> Self {
        Self {
            max_count: 0,
            channels: [
                None,
                None,
                None,
                #[cfg(feature = "with-hot-end")]
                None,
            ],
        }
    }

    pub fn set_channel_ticks(&mut self, channel: StepperChannel, ticks: Option<u64>) {

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
        #[cfg(feature = "with-hot-end")]
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

    pub fn set_max_count(&mut self, max_count: u32) {
        self.max_count = max_count;
    }

    pub fn channels(&self) -> [Option<ChannelStatus>; MULTITIMER_CHANNELS] {
        self.channels
    }

    pub fn max_count(&self) -> u32 {
        self.max_count
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
    interval_width: u64,
    ref_time: u64,
    channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
    max_count: u32,
    pulse_count: u32,
    pub(crate) stepper_enable_flags: StepperChannel,
    pub(crate) stepper_dir_fwd_flags: StepperChannel,
}

impl crate::control::motion_timing::StepPlanner {
    pub const fn new() -> Self {
        Self {
            interval_width: 0,
            ref_time: 0,
            channels: [
                None,
                None,
                None,
                #[cfg(feature = "with-hot-end")]
                    None,
            ],
            max_count: 0,
            pulse_count: 0,
            stepper_enable_flags: StepperChannel::empty(),
            stepper_dir_fwd_flags: StepperChannel::empty(),
        }
    }

    pub const fn from(channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
                      max_count: u32,
                      stepper_enable_flags: StepperChannel,
                      stepper_dir_fwd_flags: StepperChannel,

    ) -> Self {
        Self {
            interval_width: 0,
            ref_time: 0,
            channels,
            max_count,
            pulse_count: 0,
            stepper_enable_flags,
            stepper_dir_fwd_flags,
        }
    }

    pub fn reset(&mut self, interval_width: u64) {
        self.ref_time = 0;
        self.pulse_count = 0;
        for channel in self.channels.iter_mut() {
            match channel.as_mut() {
                None => {
                }
                Some(w) => {
                    w.next_tick = (w.width + 1) >> 1
                }
            }
        }
        self.interval_width = interval_width;
    }

    pub fn next(&mut self, step_width: u64) -> Option<StepperChannel> {
        self.ref_time += step_width;
        let mut triggered_channels = StepperChannel::empty();
        for channel in self.channels.iter_mut() {
            if let Some(_ch) = channel.as_mut() {
                if _ch.next_tick <= self.ref_time  {
                    if _ch.width < step_width {
                        unreachable!("Feedrate exceeded: width: {} max: {}", _ch.width, step_width);
                    }
                    _ch.next_tick += _ch.width;
                    triggered_channels.set(_ch.name, true);
                }
            }
        }
        if triggered_channels.is_empty() {
            self.pulse_count += 1;
        }
        return Some(triggered_channels);
    }
}
