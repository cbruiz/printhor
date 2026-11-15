//! Motion execution metrics (feature `motion-metrics`), reported by `M800` (`M800 S0` resets).
//!
//! They locate where the motion fluidity is lost on a given board:
//! - **Step ticker** (ISR): its CPU load, and the ticks it found nothing to step while a move was
//!   in progress (*stalls*: the motor stops because the stepper task did not deliver the next
//!   micro-segment in time).
//! - **Stepper task**: per segment, the cost of the motion profile and of the micro-segment
//!   interpolation, the time blocked on a full step queue (good: it is ahead) and the planned motion
//!   time against the wall-clock time it took.
//! - **Planner**: the cost of scheduling a move (motion chaining included), how often it waited on a
//!   full segment queue (good: it is ahead) and how often the stepper took a segment with nothing
//!   queued behind it (no look-ahead: it must stop at its end).
//!
//! Every counter lives in a critical-section mutex: on the Cortex-M0+ (SKR Mini E3) there are no
//! atomic read-modify-write instructions. The step ticker already runs in a critical section, so
//! its update costs only the two time reads and a few additions.
//!
//! Without the feature, every function is an empty inline stub.

#[cfg(feature = "motion-metrics")]
mod imp {
    use core::cell::RefCell;
    use core::sync::atomic::{AtomicBool, Ordering};
    use critical_section::Mutex as CsMutex;

    /// Snapshot of the metrics. Times in µs unless stated otherwise
    #[derive(Clone, Copy, Default)]
    pub struct MotionMetrics {
        /// When the metrics were reset (µs since boot)
        pub since_us: u64,

        // Step ticker (ISR)
        pub isr_ticks: u32,
        pub isr_busy_us: u64,
        pub isr_max_us: u32,
        /// Ticks without a micro-segment to execute while a move was in progress
        pub stall_ticks: u32,
        /// Number of stalls (consecutive stall ticks count as one)
        pub stall_episodes: u32,

        // Stepper task (per segment)
        pub segments: u32,
        pub profile_us: u64,
        pub profile_max_us: u32,
        pub interp_us: u64,
        pub interp_max_us: u32,
        /// Time the stepper task waited on a full step queue (it was ahead: good)
        pub push_wait_us: u64,
        /// Sum of the planned (profile) motion time of the executed segments
        pub planned_us: u64,
        /// Sum of the wall-clock time from taking each segment to completing it
        pub wall_us: u64,
        /// Distance executed (µm, space units)
        pub distance_um: u64,
        /// Segments that ended at zero speed
        pub full_stops: u32,
        /// Gaps where the stepper waited for the next segment after finishing one
        pub starved_gaps: u32,
        pub starved_us: u64,

        // Planner
        pub plans: u32,
        pub plan_us: u64,
        pub plan_max_us: u32,
        pub chain_us: u64,
        pub chain_max_us: u32,
        /// Times the planner found the segment queue full (it was ahead: good)
        pub queue_full_waits: u32,
        /// Segments taken by the stepper with nothing queued behind them (no look-ahead)
        pub no_lookahead: u32,
        /// Minimum number of segments queued behind the one taken by the stepper
        pub min_lookahead: u8,
        /// Moves slowed down by the throughput slowdown (the planner could not keep up with them)
        pub slowdowns: u32,
        /// Sum and count of the average time the planner took to produce a move, sampled at each
        /// move (µs, see `RingBuffer::sample_move_period`)
        pub move_period_us: u64,
        pub move_periods: u32,
        /// Highest average time the planner took to produce a move (µs)
        pub move_period_max_us: u32,
    }

    static METRICS: CsMutex<RefCell<MotionMetrics>> = CsMutex::new(RefCell::new(MotionMetrics {
        since_us: 0,
        isr_ticks: 0,
        isr_busy_us: 0,
        isr_max_us: 0,
        stall_ticks: 0,
        stall_episodes: 0,
        segments: 0,
        profile_us: 0,
        profile_max_us: 0,
        interp_us: 0,
        interp_max_us: 0,
        push_wait_us: 0,
        planned_us: 0,
        wall_us: 0,
        distance_um: 0,
        full_stops: 0,
        starved_gaps: 0,
        starved_us: 0,
        plans: 0,
        plan_us: 0,
        plan_max_us: 0,
        chain_us: 0,
        chain_max_us: 0,
        queue_full_waits: 0,
        no_lookahead: 0,
        min_lookahead: u8::MAX,
        slowdowns: 0,
        move_period_us: 0,
        move_periods: 0,
        move_period_max_us: 0,
    }));

    /// A move is being executed (set by the stepper task): idle ticks are stalls
    static MOTION_ACTIVE: AtomicBool = AtomicBool::new(false);
    /// The previous tick was a stall (to count episodes). Only accessed by the ISR
    static IN_STALL: AtomicBool = AtomicBool::new(false);

    #[inline]
    pub fn now_us() -> u64 {
        embassy_time::Instant::now().as_micros()
    }

    #[inline]
    fn sat_u32(v: u64) -> u32 {
        v.min(u32::MAX as u64) as u32
    }

    /// From the step ticker, already in a critical section
    #[inline]
    pub fn isr_tick(cs: critical_section::CriticalSection<'_>, start_us: u64, idle: bool) {
        let busy = now_us().saturating_sub(start_us);
        let stalled = idle && MOTION_ACTIVE.load(Ordering::Relaxed);
        let mut m = METRICS.borrow_ref_mut(cs);
        m.isr_ticks = m.isr_ticks.wrapping_add(1);
        m.isr_busy_us += busy;
        m.isr_max_us = m.isr_max_us.max(sat_u32(busy));
        if stalled {
            m.stall_ticks = m.stall_ticks.wrapping_add(1);
            if !IN_STALL.load(Ordering::Relaxed) {
                m.stall_episodes = m.stall_episodes.wrapping_add(1);
            }
        }
        IN_STALL.store(stalled, Ordering::Relaxed);
    }

    pub fn set_motion_active(active: bool) {
        MOTION_ACTIVE.store(active, Ordering::Relaxed);
    }

    fn with(f: impl FnOnce(&mut MotionMetrics)) {
        critical_section::with(|cs| f(&mut METRICS.borrow_ref_mut(cs)));
    }

    pub fn record_profile(us: u64) {
        with(|m| {
            m.profile_us += us;
            m.profile_max_us = m.profile_max_us.max(sat_u32(us));
        });
    }

    pub fn record_interp(us: u64) {
        with(|m| {
            m.interp_us += us;
            m.interp_max_us = m.interp_max_us.max(sat_u32(us));
        });
    }

    pub fn record_push_wait(us: u64) {
        with(|m| m.push_wait_us += us);
    }

    pub fn record_segment(planned_us: u64, wall_us: u64, distance_um: u64, full_stop: bool) {
        with(|m| {
            m.segments = m.segments.wrapping_add(1);
            m.planned_us += planned_us;
            m.wall_us += wall_us;
            m.distance_um += distance_um;
            if full_stop {
                m.full_stops = m.full_stops.wrapping_add(1);
            }
        });
    }

    /// The stepper took a segment with `behind` segments queued after it
    pub fn record_lookahead(behind: u8) {
        with(|m| {
            m.min_lookahead = m.min_lookahead.min(behind);
            if behind == 0 {
                m.no_lookahead = m.no_lookahead.wrapping_add(1);
            }
        });
    }

    pub fn record_starved_gap(us: u64) {
        with(|m| {
            m.starved_gaps = m.starved_gaps.wrapping_add(1);
            m.starved_us += us;
        });
    }

    pub fn record_plan(plan_us: u64, chain_us: u64) {
        with(|m| {
            m.plans = m.plans.wrapping_add(1);
            m.plan_us += plan_us;
            m.plan_max_us = m.plan_max_us.max(sat_u32(plan_us));
            m.chain_us += chain_us;
            m.chain_max_us = m.chain_max_us.max(sat_u32(chain_us));
        });
    }

    pub fn record_slowdown() {
        with(|m| m.slowdowns = m.slowdowns.wrapping_add(1));
    }

    pub fn record_move_period(us: u32) {
        with(|m| {
            if us > 0 {
                m.move_period_us += us as u64;
                m.move_periods = m.move_periods.wrapping_add(1);
            }
            m.move_period_max_us = m.move_period_max_us.max(us);
        });
    }

    pub fn record_queue_full_wait() {
        with(|m| m.queue_full_waits = m.queue_full_waits.wrapping_add(1));
    }

    pub fn reset() {
        let since_us = now_us();
        with(|m| {
            *m = MotionMetrics {
                since_us,
                min_lookahead: u8::MAX,
                ..MotionMetrics::default()
            }
        });
    }

    pub fn snapshot() -> MotionMetrics {
        critical_section::with(|cs| *METRICS.borrow_ref(cs))
    }
}

#[cfg(feature = "motion-metrics")]
pub use imp::*;

#[cfg(not(feature = "motion-metrics"))]
mod stubs {
    #[inline(always)]
    pub fn now_us() -> u64 {
        0
    }
    #[inline(always)]
    pub fn isr_tick(_cs: critical_section::CriticalSection<'_>, _start_us: u64, _idle: bool) {}
    #[inline(always)]
    pub fn set_motion_active(_active: bool) {}
    #[inline(always)]
    pub fn record_profile(_us: u64) {}
    #[inline(always)]
    pub fn record_interp(_us: u64) {}
    #[inline(always)]
    pub fn record_push_wait(_us: u64) {}
    #[inline(always)]
    pub fn record_segment(_planned_us: u64, _wall_us: u64, _distance_um: u64, _full_stop: bool) {}
    #[inline(always)]
    pub fn record_lookahead(_behind: u8) {}
    #[inline(always)]
    pub fn record_starved_gap(_us: u64) {}
    #[inline(always)]
    pub fn record_plan(_plan_us: u64, _chain_us: u64) {}
    #[inline(always)]
    pub fn record_queue_full_wait() {}
    #[inline(always)]
    pub fn record_slowdown() {}
    #[inline(always)]
    pub fn record_move_period(_us: u32) {}
}

#[cfg(not(feature = "motion-metrics"))]
pub use stubs::*;
