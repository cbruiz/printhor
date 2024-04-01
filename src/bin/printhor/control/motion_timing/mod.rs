use embassy_time::{Duration, Instant};
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
        const MULTITIMER_CHANNELS: usize = 4;
    }
    else {
        const MULTITIMER_CHANNELS: usize = 3;
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
#[derive(Clone)]
pub struct MultiTimer {
    interval_width: u64,
    ref_time: u64,
    channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
}

impl MultiTimer {
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
        }
    }

    pub fn reset(&mut self, interval_width: u64) {
        self.ref_time = 0;
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

    pub fn next(&mut self) -> Option<(StepperChannel, Duration)> {
        let mut next_tick = self.interval_width + 1;
        let mut triggered_channels = StepperChannel::empty();
        for channel in self.channels.iter() {
            if let Some(_ch) = &channel {
                if _ch.next_tick < next_tick  {
                    next_tick = _ch.next_tick;
                }
            }
        }
        if next_tick > self.interval_width {
            self.ref_time = self.interval_width;
            return None
        }
        for channel in self.channels.iter_mut() {
            if let Some(_ch) = channel.as_mut() {
                if _ch.next_tick <= next_tick  {
                    _ch.next_tick += _ch.width;
                    triggered_channels.set(_ch.name, true);
                }
            }

        }
        let tw = next_tick - self.ref_time;
        self.ref_time = next_tick;
        return Some((triggered_channels, Duration::from_ticks(tw)));
    }
}

#[inline(always)]
pub fn s_block_for(duration: Duration) {
    let expires_at = Instant::now() + duration;
    while Instant::now() < expires_at {}
}
