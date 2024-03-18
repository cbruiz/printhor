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

#[inline(always)]
pub fn now() -> Instant {
    #[cfg(feature = "no-real-time")]
    return Instant::from_ticks(0);
    #[cfg(not(feature = "no-real-time"))]
    return Instant::now();
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        const MULTITIMER_CHANNELS: usize = 4;
    }
    else {
        const MULTITIMER_CHANNELS: usize = 3;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "no-real-time")] {
        pub struct MultiTimer {
            ref_time: u64,
            interval_width: u64,
            channels: [ChannelStatus; MULTITIMER_CHANNELS],
        }
    }
    else {
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
        pub struct MultiTimer {
            interval_width: u64,
            channels: [ChannelStatus; MULTITIMER_CHANNELS],
        }
    }
}


impl MultiTimer {
    #[allow(unused)]
    pub fn new_using(interval_width: u64, mut s: [ChannelStatus; MULTITIMER_CHANNELS]) -> Self {
        let t0 = now().as_ticks();
        for i in s.iter_mut() {
            i.next_tick = (i.width / 2) + t0
        }
        Self {
            #[cfg(feature = "no-real-time")]
            ref_time: 0,
            interval_width: t0 + interval_width,
            channels: s,
        }
    }

    pub const fn new() -> Self {
        Self {
            #[cfg(feature = "no-real-time")]
            ref_time: 0,
            interval_width: 0,
            channels: [
                ChannelStatus::new(StepperChannel::X, 0),
                ChannelStatus::new(StepperChannel::Y, 0),
                ChannelStatus::new(StepperChannel::Z, 0),
                #[cfg(feature = "with-hot-end")]
                ChannelStatus::new(StepperChannel::E, 0),
            ],
        }
    }

    pub fn reset(&mut self, interval_width: u64) {
        let t0 = now().as_ticks();
        for i in self.channels.iter_mut() {
            i.next_tick = (i.width / 2) + t0
        }
        self.interval_width = t0 + interval_width;
    }

    pub fn set_channel_ticks(&mut self, channel: StepperChannel, ticks: u64) {

        if channel.contains(StepperChannel::X) {
            self.channels[0].width = ticks;
        }
        if channel.contains(StepperChannel::Y) {
            self.channels[1].width = ticks;
        }
        if channel.contains(StepperChannel::Z) {
            self.channels[2].width = ticks;
        }
        #[cfg(feature = "with-hot-end")]
        if channel.contains(StepperChannel::E) {
            self.channels[3].width = ticks;
        }

    }

    pub fn next(&mut self) -> Option<(StepperChannel, Duration)> {
        let mut next_tick = self.interval_width;
        let mut target_channel: Option<&mut ChannelStatus> = None;
        for c in self.channels.iter_mut() {
            if c.next_tick < next_tick && c.next_tick < (self.interval_width - (c.width / 4)) {
                next_tick = c.next_tick;
                target_channel = Some(c);
            }
        }
        if next_tick >= self.interval_width {
            return None
        }
        return if let Some(channel) = target_channel {
            cfg_if::cfg_if! {
                if #[cfg(feature = "no-real-time")] {
                    let ref_time = self.ref_time;
                    let tw = Duration::from_ticks((channel.next_tick as i64 - ref_time as i64).max(0) as u64);
                    channel.next_tick += channel.width;
                    self.ref_time += tw.as_ticks();
                    Some((channel.name, tw))
                }
                else {
                    let ref_time = now().as_ticks();
                    let tw = Duration::from_ticks((channel.next_tick as i64 - ref_time as i64).max(0) as u64);
                    //crate::hwa::trace!("Awaiting {}", tw.as_micros());
                    while now().as_ticks() < channel.next_tick {}
                    channel.next_tick += channel.width;
                    Some((channel.name, tw))
                }
            }
        } else {
            None
        };
    }

    #[cfg(feature = "no-real-time")]
    pub fn sync_clock(&mut self, d: Duration) {
        self.ref_time += d.as_ticks();
    }
}

#[inline(always)]
pub fn s_block_for(duration: Duration) {
    let expires_at = Instant::now() + duration;
    while Instant::now() < expires_at {}
}

/*
#[inline]
pub async fn s_block_for(duration: Duration) {
    //let expires_at = Instant::now() + duration;
    //while Instant::now() < expires_at {}
    embassy_time::Timer::after(duration).await
}

 */