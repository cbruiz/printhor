//! Input shaping (Marlin's `M593`, Klipper's input shaper). Feature `with-input-shaping`.
//!
//! The frame of a machine rings at its resonance frequency after every change of acceleration
//! (ghosting/ringing on the prints). An input shaper replaces the motion `x(t)` of an axis by a
//! weighted sum of delayed copies of it, `Σ Aⱼ·x(t − Tⱼ)` (`Σ Aⱼ = 1`), whose impulses cancel each
//! other's oscillation at the frequency (and damping) the shaper is tuned to. The motion ends
//! exactly where it would without the shaper, `max Tⱼ` later; the corners are slightly smoothed.
//!
//! The shapers (formulas of Klipper's `shaper_defs.py`), by the `T` word of `M593`:
//!
//! | `T` | Shaper | Impulses | Duration | |
//! |---|---|---|---|---|
//! | 0 | ZV | 2 | ½·t_d | Marlin's. Shortest, sensitive to a frequency error |
//! | 1 | EI | 3 | t_d | Robust to a frequency error (5% vibration left at the frequency) |
//! | 2 | MZV | 3 | ¾·t_d | Robust, less smoothing than EI/ZVD |
//! | 3 | ZVD | 3 | t_d | Robust |
//!
//! (`t_d = 1/(f·√(1 − ζ²))`). The X and Y axes are shaped, each with its own shaper (`M593 X`,
//! `M593 Y`); the other axes (Z, the extruder) are not. On Core-XY the shaped axes are its motors:
//! use the same shaper for both.
//!
//! # In the step generation
//!
//! The [super::MicroSegmentInterpolator] still computes the steps of every axis, as without input
//! shaping (the commanded motion): the positions, the deviation corrections and everything that
//! depends on them are unchanged. The steps of the shaped axes are then replaced, window by window,
//! by those of the shaped motion ([InputShaper::advance]):
//!
//! * The commanded motion of an axis is kept at the end of every window (position and velocity,
//!   [ShaperHistory]), in a frame relative to the motor (a `G92` does not move it). The positions
//!   at the delayed instants are interpolated (cubic Hermite: exact for the cubic phases of the
//!   S-curve, the velocity jumps of the corners kept apart).
//! * The motor goes to the step closest to the shaped position at every window end, in either
//!   direction (a corner may reverse a shaped axis inside a segment), its rate ramping between the
//!   shaped velocities ([super::micro_segment_interpolation::SignedChannel]). Exact step counts.
//! * When the motion stops (the queue runs empty, a dwell, a homing), the shaped motion is
//!   completed ([InputShaper::flush_window]): its last window ends exactly at the commanded step,
//!   so the motor is where the position says, as without input shaping. An abort (M410) stops the
//!   motors where they are: the position is corrected by what they lag ([InputShaper::abort]).
//!
//! A changed shaper (`M593`) is applied from the next stop. The shaped axes lag the others (the
//! extruder, Z) by the time centroid of the shaper (a few ms): as Marlin's.
use crate::hwa;
use hwa::CoordSel;
use hwa::controllers::MultiTimer;
use hwa::math;
use math::{Real, TVector};

use super::micro_segment_interpolation::{RampWindowCache, SignedChannel};

#[cfg(not(all(feature = "with-x-axis", feature = "with-y-axis")))]
compile_error!("feature `with-input-shaping` requires `with-x-axis` and `with-y-axis`");
#[cfg(feature = "with-motion-broadcast")]
compile_error!("feature `with-input-shaping` is not compatible with `with-motion-broadcast`");

/// The shaped axes
pub const SHAPED_AXES: [CoordSel; 2] = [CoordSel::X, CoordSel::Y];

/// The lowest frequency (Hz) a shaper can be tuned to (Marlin's `SHAPING_MIN_FREQ`): it bounds the
/// duration of the shapers, and so the history kept
pub const MIN_FREQUENCY: f32 = 20.0;
/// The highest damping ratio a shaper can be tuned to
pub const MAX_DAMPING: f32 = 0.5;
/// The longest shaper: `t_d` at [MIN_FREQUENCY] and [MAX_DAMPING] (57.7 ms), in µs
const MAX_DURATION_US: u32 = 57_800;
/// The commanded samples are kept at least this apart (but the last one): windows shorter than this
/// (the short segments) are merged
const MIN_SPACING_US: u32 = 2_000;
/// Samples of the history: enough to cover the longest shaper
pub const HISTORY_LEN: usize = (MAX_DURATION_US / MIN_SPACING_US) as usize + 4;

/// The kind of shaper (the `T` word of `M593`)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ShaperKind {
    /// Zero Vibration (Marlin's)
    Zv,
    /// Extra Insensitive (5% vibration tolerance)
    Ei,
    /// Modified ZV
    Mzv,
    /// Zero Vibration and Derivative
    Zvd,
}

impl ShaperKind {
    /// The shaper of a `T` word: 0 ZV, 1 EI (as Marlin), 2 MZV, 3 ZVD
    pub fn from_index(index: i32) -> Option<Self> {
        match index {
            0 => Some(Self::Zv),
            1 => Some(Self::Ei),
            2 => Some(Self::Mzv),
            3 => Some(Self::Zvd),
            _ => None,
        }
    }

    /// The `T` word of the shaper
    pub fn index(self) -> u8 {
        match self {
            Self::Zv => 0,
            Self::Ei => 1,
            Self::Mzv => 2,
            Self::Zvd => 3,
        }
    }
}

/// The shaper of an axis (`M593`): its kind, frequency (Hz, 0 disables it) and damping ratio
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct ShaperConfig {
    pub kind: ShaperKind,
    pub frequency: Real,
    pub damping: Real,
}

impl ShaperConfig {
    /// No shaping
    pub const fn disabled() -> Self {
        Self {
            kind: ShaperKind::Zv,
            frequency: math::ZERO,
            damping: math::ZERO,
        }
    }

    /// Whether `frequency` can be set: 0 (disabled) or [MIN_FREQUENCY] or above
    pub fn is_valid_frequency(frequency: Real) -> bool {
        frequency.is_zero() || frequency >= Real::from_f32(MIN_FREQUENCY)
    }

    /// Whether `damping` can be set: from 0 to [MAX_DAMPING]
    pub fn is_valid_damping(damping: Real) -> bool {
        damping >= math::ZERO && damping <= Real::from_f32(MAX_DAMPING)
    }
}

/// An input shaper: the impulses that follow the first one (no delay), as amplitude and delay
/// (µs). The first impulse has the rest of the amplitude (`1 − Σ`): the shaped value is computed as
/// `x(t) + Σ Aⱼ·(x(t − Tⱼ) − x(t))`, exactly `x` at rest
#[derive(Clone, Copy, Debug)]
pub struct Shaper {
    delayed: [(Real, u32); 2],
    len: usize,
}

impl Shaper {
    /// No shaping
    pub const IDENTITY: Self = Self {
        delayed: [(math::ZERO, 0); 2],
        len: 0,
    };

    /// The shaper of a configuration: the identity when disabled (or out of range)
    pub fn new(config: &ShaperConfig) -> Self {
        let (f, zeta) = (config.frequency, config.damping);
        if !(f >= Real::from_f32(MIN_FREQUENCY)) || !ShaperConfig::is_valid_damping(zeta) {
            return Self::IDENTITY;
        }
        let df = (math::ONE - zeta * zeta).sqrt().unwrap_or(math::ONE);
        // Damped period (µs)
        let t_d = Real::from_f32(1.0e6) / (f * df);
        // Every shaper is `[c₀, c₁·K, c₂·K²]` at `[0, t₁, t₂]·t_d`, `K = exp(−s·ζπ/√(1 − ζ²))`:
        // (s, c, t)
        let (scale, c, t): (f32, [f32; 3], [f32; 2]) = match config.kind {
            ShaperKind::Zv => (1.0, [1.0, 1.0, 0.0], [0.5, 0.0]),
            ShaperKind::Zvd => (1.0, [1.0, 2.0, 1.0], [0.5, 1.0]),
            // 1 − 1/√2, √2 − 1
            ShaperKind::Mzv => (0.75, [0.29289322, 0.41421356, 0.29289322], [0.375, 0.75]),
            // Vibration tolerance: 5%
            ShaperKind::Ei => (1.0, [0.25 * 1.05, 0.5 * 0.95, 0.25 * 1.05], [0.5, 1.0]),
        };
        let k = (-(zeta * math::PI * Real::from_f32(scale)) / df).exp();
        let a = [Real::from_f32(c[1]) * k, Real::from_f32(c[2]) * k * k];
        let inv_sum = (Real::from_f32(c[0]) + a[0] + a[1]).recip();
        let len = if config.kind == ShaperKind::Zv { 1 } else { 2 };
        let mut delayed = [(math::ZERO, 0u32); 2];
        for (j, slot) in delayed.iter_mut().take(len).enumerate() {
            let delay_us = (t_d * Real::from_f32(t[j])).round().to_i64().unwrap_or(0);
            *slot = (a[j] * inv_sum, delay_us.clamp(1, MAX_DURATION_US as i64) as u32);
        }
        Self { delayed, len }
    }

    /// Whether it does not shape
    #[inline]
    pub fn is_identity(&self) -> bool {
        self.len == 0
    }

    /// The delayed impulses: amplitude and delay (µs)
    #[inline]
    pub fn delayed(&self) -> &[(Real, u32)] {
        &self.delayed[..self.len]
    }

    /// Its duration (µs): the delay of its last impulse
    pub fn duration_us(&self) -> u32 {
        self.delayed().last().map_or(0, |(_, t)| *t)
    }
}

/// A commanded sample: its instant, and per shaped axis the position and the velocities that reach
/// and leave it (different at a corner)
#[derive(Clone, Copy)]
struct Sample {
    t_us: u32,
    r: [Real; 2],
    v_in: [Real; 2],
    v_out: [Real; 2],
}

/// The recent commanded motion of the shaped axes (positions and velocities at the window ends),
/// long enough for the longest shaper
pub struct ShaperHistory {
    samples: [Sample; HISTORY_LEN],
    newest: usize,
    len: usize,
}

impl ShaperHistory {
    pub const fn new() -> Self {
        Self {
            samples: [Sample {
                t_us: 0,
                r: [math::ZERO; 2],
                v_in: [math::ZERO; 2],
                v_out: [math::ZERO; 2],
            }; HISTORY_LEN],
            newest: 0,
            len: 0,
        }
    }

    /// The motion starts from rest at `t_us`, at `r`, leaving it with velocity `v`: everything
    /// before is that rest
    pub fn start(&mut self, t_us: u32, r: [Real; 2], v: [Real; 2]) {
        self.newest = 0;
        self.len = 1;
        self.samples[0] = Sample {
            t_us,
            r,
            v_in: [math::ZERO; 2],
            v_out: v,
        };
    }

    /// The motion goes on from the last sample with velocity `v` (a corner, or a stop)
    pub fn junction(&mut self, v: [Real; 2]) {
        self.samples[self.newest].v_out = v;
    }

    /// The motion reaches `r` with velocity `v` at `t_us` (later than the last sample). Merged with
    /// the last sample when too close to the one before it (the history covers a fixed duration)
    pub fn push(&mut self, t_us: u32, r: [Real; 2], v: [Real; 2]) {
        // Merged into the last sample while it is closer than the spacing (and the interval before
        // it stays short): the samples before the last one are at least the spacing apart
        let newest = self.samples[self.newest].t_us;
        let before = self.samples[(self.newest + HISTORY_LEN - 1) % HISTORY_LEN].t_us;
        let merge = self.len >= 2
            && t_us.wrapping_sub(newest) < MIN_SPACING_US
            && t_us.wrapping_sub(before) < 2 * MIN_SPACING_US;
        if !merge {
            self.newest = (self.newest + 1) % HISTORY_LEN;
            self.len = (self.len + 1).min(HISTORY_LEN);
        }
        self.samples[self.newest] = Sample {
            t_us,
            r,
            v_in: v,
            v_out: v,
        };
    }

    /// Position and velocity of the last sample
    #[inline]
    pub fn newest(&self, axis: usize) -> (Real, Real) {
        let s = &self.samples[self.newest];
        (s.r[axis], s.v_in[axis])
    }

    /// The interval containing the instant `delay_us` before `now_us`: (older sample, newer
    /// sample, µs from the older one, interval width µs). `Err` with the sample to take when it is
    /// the last one (no delay) or before the oldest (the rest the motion started from)
    fn interval(&self, now_us: u32, delay_us: u32) -> Result<(usize, usize, u32, u32), usize> {
        let mut b = self.newest;
        let mut age_b = now_us.wrapping_sub(self.samples[b].t_us);
        if delay_us <= age_b {
            return Err(b);
        }
        for _ in 1..self.len {
            let a = (b + HISTORY_LEN - 1) % HISTORY_LEN;
            let age_a = now_us.wrapping_sub(self.samples[a].t_us);
            if age_a >= delay_us {
                return Ok((a, b, age_a - delay_us, age_a - age_b));
            }
            b = a;
            age_b = age_a;
        }
        Err(b)
    }

    /// Position and velocity of an axis `delay_us` before `now_us`
    pub fn eval(&self, now_us: u32, delay_us: u32, axis: usize) -> (Real, Real) {
        let (p, v, _) = self.eval_hermite(now_us, delay_us, axis, false);
        (p, v)
    }

    /// Position, velocity and acceleration of an axis `delay_us` before `now_us` (the acceleration
    /// of the interpolation, for the analysis)
    pub fn eval_with_acceleration(&self, now_us: u32, delay_us: u32, axis: usize) -> (Real, Real, Real) {
        self.eval_hermite(now_us, delay_us, axis, true)
    }

    fn eval_hermite(&self, now_us: u32, delay_us: u32, axis: usize, with_accel: bool) -> (Real, Real, Real) {
        let (a, b, x_us, h_us) = match self.interval(now_us, delay_us) {
            Ok(interval) => interval,
            Err(s) => {
                let s = &self.samples[s];
                return (s.r[axis], s.v_in[axis], math::ZERO);
            }
        };
        let (sa, sb) = (&self.samples[a], &self.samples[b]);
        let h = Real::from_lit(h_us as i64, 0);
        let tau = Real::from_lit(x_us as i64, 0) / h;
        let dt = h * Real::from_f32(1.0e-6);
        let (va, vb) = (sa.v_out[axis], sb.v_in[axis]);
        let delta = sb.r[axis] - sa.r[axis];
        let one_tau = math::ONE - tau;
        // Cubic Hermite basis (positions relative to the older sample)
        let h01 = tau * tau * (math::THREE - math::TWO * tau);
        let h10 = tau * one_tau * one_tau;
        let h11 = tau * tau * (tau - math::ONE);
        let p = sa.r[axis] + h01 * delta + dt * (h10 * va + h11 * vb);
        let slope = delta / dt;
        let v = math::SIX * tau * one_tau * slope
            + one_tau * (math::ONE - math::THREE * tau) * va
            + tau * (math::THREE * tau - math::TWO) * vb;
        let acc = match with_accel {
            true => {
                ((math::SIX - math::TWELVE * tau) * slope
                    + (math::SIX * tau - math::FOUR) * va
                    + (math::SIX * tau - math::TWO) * vb)
                    / dt
            }
            false => math::ZERO,
        };
        (p, v, acc)
    }
}

/// Shapes the X and Y steps of the stepper task (see the module documentation)
pub struct InputShaper {
    shapers: [Shaper; 2],
    history: ShaperHistory,
    /// Motion time (µs) since the motion started from rest
    now_us: u32,
    /// Commanded motor position (steps) of each axis, relative to where the motion started from rest
    commanded: [i32; 2],
    /// The shaped motors (same frame)
    channels: [SignedChannel; 2],
    /// The segment: commanded position (continuous, steps) at its start, and steps per unit of path
    /// (signed), of each axis
    seg_start: [Real; 2],
    seg_ratio: [Real; 2],
    /// Direction of each axis in the segment (of the commanded steps)
    seg_forward: [bool; 2],
    ramps: RampWindowCache,
    tick_s: Real,
    /// When the commanded motion stopped, while its shaped motion is being completed
    stopped_us: Option<u32>,
    at_rest: bool,
    /// Steps of the last window of each axis
    #[cfg(any(test, feature = "assert-motion"))]
    pub counts: [u32; 2],
}

impl InputShaper {
    pub fn new(tick_us: u32) -> Self {
        Self {
            shapers: [Shaper::IDENTITY; 2],
            history: ShaperHistory::new(),
            now_us: 0,
            commanded: [0; 2],
            channels: [SignedChannel::new(0, true); 2],
            seg_start: [math::ZERO; 2],
            seg_ratio: [math::ZERO; 2],
            seg_forward: [true; 2],
            ramps: RampWindowCache::new(tick_us),
            tick_s: Real::from_lit(tick_us.max(1) as i64, 0) * Real::from_f32(1.0e-6),
            stopped_us: None,
            at_rest: true,
            #[cfg(any(test, feature = "assert-motion"))]
            counts: [0; 2],
        }
    }

    /// Whether the shaped motion is complete (the motors are where the commanded motion is)
    #[inline]
    pub fn is_at_rest(&self) -> bool {
        self.at_rest
    }

    /// Starts shaping a segment about to be executed. From rest, the shapers are taken from
    /// `config`: when none shapes, nothing is done (`false`) and the segment is executed as without
    /// input shaping.
    ///
    /// * `unit_vector_dir`, `steps_per_su`: the segment (as executed: after the deviation fix).
    /// * `start_offset_steps`: per axis, the ideal start of the segment minus the step the motor is
    ///   at, in the direction of the axis (as given to the interpolator).
    /// * `v0`: speed of the motion profile at its start.
    pub fn begin(
        &mut self,
        config: &[ShaperConfig; 2],
        unit_vector_dir: &TVector<Real>,
        steps_per_su: &TVector<Real>,
        start_offset_steps: &TVector<Real>,
        v0: Real,
    ) -> bool {
        if self.at_rest {
            self.shapers = [Shaper::new(&config[0]), Shaper::new(&config[1])];
            if self.shapers.iter().all(Shaper::is_identity) {
                return false;
            }
        }
        let mut v = [math::ZERO; 2];
        for (k, axis) in SHAPED_AXES.iter().enumerate() {
            let u = unit_vector_dir.get_coord(*axis).unwrap_or(math::ZERO);
            self.seg_forward[k] = u.is_defined_positive();
            self.seg_ratio[k] = u * steps_per_su.get_coord(*axis).unwrap_or(math::ZERO);
            let commanded = Real::from_lit(self.commanded[k] as i64, 0);
            self.seg_start[k] = match u.is_zero() {
                // Still: where its commanded motion is
                true => match self.at_rest {
                    true => commanded,
                    false => self.history.newest(k).0,
                },
                false => {
                    let offset = start_offset_steps
                        .get_coord(*axis)
                        .unwrap_or(math::ZERO)
                        .clamp(-math::HALF, math::HALF);
                    commanded + if self.seg_forward[k] { offset } else { -offset }
                }
            };
            v[k] = self.seg_ratio[k] * v0;
        }
        match self.at_rest {
            true => {
                self.at_rest = false;
                self.now_us = 0;
                self.history.start(0, self.seg_start, v);
            }
            false => self.history.junction(v),
        }
        true
    }

    /// Shapes the window ending at path position `s` with speed `v`, `width_us` wide:
    /// `multi_timer` holds the commanded steps of every axis (see [super::MicroSegmentInterpolator]);
    /// those of the shaped axes are replaced by the shaped ones, and their directions set in
    /// `dir_fwd`
    pub fn advance(&mut self, multi_timer: &mut MultiTimer, dir_fwd: &mut CoordSel, width_us: u32, s: Real, v: Real) {
        let (mut r, mut vel) = ([math::ZERO; 2], [math::ZERO; 2]);
        for (k, axis) in SHAPED_AXES.iter().enumerate() {
            let n = multi_timer.count(*axis) as i32;
            self.commanded[k] += if self.seg_forward[k] { n } else { -n };
            r[k] = self.seg_start[k] + self.seg_ratio[k] * s;
            vel[k] = self.seg_ratio[k] * v;
        }
        self.now_us = self.now_us.wrapping_add(width_us);
        self.history.push(self.now_us, r, vel);
        self.shape(multi_timer, dir_fwd, width_us, false);
    }

    /// Completes the shaped motion once the commanded one stopped: the next window (`width_us`
    /// wide, the shaped axes only) and the directions of its axes, or `None` once at rest. The last
    /// window ends exactly at the commanded steps
    pub fn flush_window(&mut self, width_us: u32) -> Option<(MultiTimer, CoordSel)> {
        if self.at_rest {
            return None;
        }
        let stopped = match self.stopped_us {
            Some(t) => t,
            None => {
                // The commanded velocity drops to zero
                self.history.junction([math::ZERO; 2]);
                self.stopped_us = Some(self.now_us);
                self.now_us
            }
        };
        self.now_us = self.now_us.wrapping_add(width_us);
        let r = [self.history.newest(0).0, self.history.newest(1).0];
        self.history.push(self.now_us, r, [math::ZERO; 2]);
        let duration = self.shapers[0].duration_us().max(self.shapers[1].duration_us());
        let last = self.now_us.wrapping_sub(stopped) >= duration;
        let mut multi_timer = MultiTimer::new();
        multi_timer.set_displace_width(width_us);
        let mut dir_fwd = CoordSel::empty();
        self.shape(&mut multi_timer, &mut dir_fwd, width_us, last);
        if last {
            self.rest();
        }
        Some((multi_timer, dir_fwd))
    }

    /// Stops shaping where the motors are (abort): returns what they lag the commanded position
    /// (steps, motor minus commanded) of each axis
    pub fn abort(&mut self) -> [i32; 2] {
        let lag = match self.at_rest {
            true => [0; 2],
            false => [0, 1].map(|k| match self.shapers[k].is_identity() {
                true => 0,
                false => self.channels[k].motor - self.commanded[k],
            }),
        };
        self.rest();
        lag
    }

    /// Sets the steps of the last window of the shaped axes in `counts` (the steps checked by
    /// `assert-motion`)
    #[cfg(feature = "assert-motion")]
    pub fn window_counts(&self, counts: &mut TVector<u32>) {
        for (k, axis) in SHAPED_AXES.iter().enumerate() {
            if !self.shapers[k].is_identity() {
                counts.set_coord(*axis, Some(self.counts[k]));
            }
        }
    }

    /// At rest: the frame starts again where the motors are
    fn rest(&mut self) {
        self.at_rest = true;
        self.stopped_us = None;
        self.commanded = [0; 2];
        self.channels = [0, 1].map(|k| SignedChannel::new(0, self.channels[k].forward));
    }

    /// The shaped steps of the window ending now. `last`: the commanded position is reached
    fn shape(&mut self, multi_timer: &mut MultiTimer, dir_fwd: &mut CoordSel, width_us: u32, last: bool) {
        self.ramps.set_width(width_us);
        for (k, axis) in SHAPED_AXES.iter().enumerate() {
            let shaper = &self.shapers[k];
            if shaper.is_identity() {
                // Moved by the interpolator
                self.channels[k].motor = self.commanded[k];
                continue;
            }
            let (target, velocity) = match last {
                true => (Real::from_lit(self.commanded[k] as i64, 0), math::ZERO),
                false => {
                    let (r, v) = self.history.newest(k);
                    let (mut p, mut pv) = (r, v);
                    for (amplitude, delay) in shaper.delayed() {
                        let (rd, vd) = self.history.eval(self.now_us, *delay, k);
                        p += *amplitude * (rd - r);
                        pv += *amplitude * (vd - v);
                    }
                    (p, pv)
                }
            };
            let _n = self.channels[k].advance(multi_timer, *axis, target, velocity * self.tick_s, &mut self.ramps);
            #[cfg(any(test, feature = "assert-motion"))]
            {
                self.counts[k] = _n.unsigned_abs();
            }
            dir_fwd.set(*axis, self.channels[k].forward);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hwa::controllers::StepPlan;
    use crate::motion::{Constraints, HybridMotionProfile, MicroSegmentInterpolator, MotionProfile, SegmentSampler};

    const TICK_US: u32 = 10;
    const PERIOD_US: u64 = 2500;
    /// Steps per mm of X and Y
    const SPM: f64 = 80.0;

    fn real(x: f64) -> Real {
        Real::from_f32(x as f32)
    }

    fn config(kind: ShaperKind, f: f64, zeta: f64) -> ShaperConfig {
        ShaperConfig {
            kind,
            frequency: real(f),
            damping: real(zeta),
        }
    }

    /// The impulses of a shaper, the first one included: (amplitude, delay s)
    fn impulses(shaper: &Shaper) -> Vec<(f64, f64)> {
        let rest: f64 = shaper.delayed().iter().map(|(a, _)| a.to_f64()).sum();
        let mut all = vec![(1.0 - rest, 0.0)];
        all.extend(shaper.delayed().iter().map(|(a, t)| (a.to_f64(), *t as f64 * 1e-6)));
        all
    }

    /// Residual vibration of an oscillator (f, ζ) excited through the shaper, relative to the one
    /// without it (the classic V(ω, ζ) of the shaper theory)
    fn residual_vibration(shaper: &Shaper, f: f64, zeta: f64) -> f64 {
        let w = 2.0 * core::f64::consts::PI * f;
        let wd = w * (1.0 - zeta * zeta).sqrt();
        let imp = impulses(shaper);
        let t_n = imp.last().unwrap().1;
        let (mut c, mut s) = (0.0, 0.0);
        for (a, t) in imp.iter() {
            let e = (zeta * w * t).exp();
            c += a * e * (wd * t).cos();
            s += a * e * (wd * t).sin();
        }
        (-zeta * w * t_n).exp() * (c * c + s * s).sqrt()
    }

    /// The shapers: unit gain (exact at rest), their delays, and the vibration left at the
    /// frequency they are tuned to (none for ZV, MZV and ZVD; within the 5% tolerance of EI: 5%
    /// without damping). Disabled or out of range: no shaping
    #[test]
    fn the_shapers_cancel_the_vibration_at_their_frequency() {
        for zeta in [0.0, 0.1, 0.15] {
            for (kind, expected, t_end) in [
                (ShaperKind::Zv, 0.0, 0.5),
                (ShaperKind::Mzv, 0.0, 0.75),
                (ShaperKind::Zvd, 0.0, 1.0),
                (ShaperKind::Ei, 0.05, 1.0),
            ] {
                let shaper = Shaper::new(&config(kind, 40.0, zeta));
                let imp = impulses(&shaper);
                assert!(imp.iter().all(|(a, _)| *a > 0.0), "{:?}: positive impulses", kind);
                let t_d = 1.0 / (40.0 * (1.0f64 - zeta * zeta).sqrt());
                assert!((shaper.duration_us() as f64 * 1e-6 - t_end * t_d).abs() < 1.5e-6, "{:?}: duration", kind);
                let v = residual_vibration(&shaper, 40.0, zeta);
                let ok = match kind {
                    ShaperKind::Ei if zeta > 0.0 => v <= expected + 2e-3,
                    _ => (v - expected).abs() < 2e-3,
                };
                assert!(ok, "{:?} ζ={}: residual vibration {}", kind, zeta, v);
                // Detuned by 20%: ZV keeps far less than without shaping, the others even less
                let detuned = residual_vibration(&shaper, 48.0, zeta);
                assert!(detuned < 0.32, "{:?} ζ={}: detuned residual vibration {}", kind, zeta, detuned);
            }
        }
        // ZV without damping: two equal impulses half a period apart
        let zv = Shaper::new(&config(ShaperKind::Zv, 40.0, 0.0));
        assert_eq!(zv.delayed().len(), 1);
        assert!((zv.delayed()[0].0.to_f64() - 0.5).abs() < 1e-6);
        assert_eq!(zv.delayed()[0].1, 12_500);
        for disabled in [
            config(ShaperKind::Zv, 0.0, 0.1),
            config(ShaperKind::Ei, 19.0, 0.1),
            config(ShaperKind::Mzv, 40.0, 0.6),
            ShaperConfig::disabled(),
        ] {
            assert!(Shaper::new(&disabled).is_identity());
            assert_eq!(Shaper::new(&disabled).duration_us(), 0);
        }
        // The longest shaper fits the history
        assert!(Shaper::new(&config(ShaperKind::Ei, 20.0, 0.5)).duration_us() <= MAX_DURATION_US);
        assert!(ShaperConfig::is_valid_frequency(math::ZERO) && ShaperConfig::is_valid_frequency(real(20.0)));
        assert!(!ShaperConfig::is_valid_frequency(real(19.9)));
        assert!(ShaperConfig::is_valid_damping(real(0.5)) && !ShaperConfig::is_valid_damping(real(-0.01)));
        for k in [ShaperKind::Zv, ShaperKind::Ei, ShaperKind::Mzv, ShaperKind::Zvd] {
            assert_eq!(ShaperKind::from_index(k.index() as i32), Some(k));
        }
        assert_eq!(ShaperKind::from_index(4), None);
    }

    /// The history interpolates the commanded motion exactly where it is a cubic, keeps the
    /// velocity jump of a corner, takes the rest before the motion, and merges the samples too
    /// close to each other
    #[test]
    fn the_history_interpolates_the_commanded_motion() {
        // x = 2 + 3t + 40t² − 300t³ (mm, s) on axis 0; on axis 1 a corner at 10 ms: 0 then 50 mm/s
        let x = |t: f64| 2.0 + 3.0 * t + 40.0 * t * t - 300.0 * t * t * t;
        let xv = |t: f64| 3.0 + 80.0 * t - 900.0 * t * t;
        let y = |t: f64| if t < 0.01 { 0.0 } else { 50.0 * (t - 0.01) };
        let mut h = ShaperHistory::new();
        h.start(0, [real(x(0.0)), real(0.0)], [real(xv(0.0)), real(0.0)]);
        for k in 1..=8u32 {
            let t = k as f64 * 0.0025;
            // The corner at k = 4: reached at rest, left at 50 mm/s
            h.push(k * 2500, [real(x(t)), real(y(t))], [real(xv(t)), real(if k <= 4 { 0.0 } else { 50.0 })]);
            if k == 4 {
                h.junction([real(xv(t)), real(50.0)]);
            }
        }
        let now = 8 * 2500;
        for delay in [0u32, 1, 1250, 3333, 7500, 10_000, 12_345, 19_999] {
            let t = (now - delay) as f64 * 1e-6;
            let (p, v) = h.eval(now, delay, 0);
            assert!((p.to_f64() - x(t)).abs() < 1e-4, "x({}) = {:?} vs {}", t, p, x(t));
            assert!((v.to_f64() - xv(t)).abs() < 1e-2, "x'({}) = {:?} vs {}", t, v, xv(t));
            let (q, _, a) = h.eval_with_acceleration(now, delay, 1);
            assert!((q.to_f64() - y(t)).abs() < 1e-4, "y({}) = {:?} vs {}", t, q, y(t));
            if delay > 0 && delay != 10_000 {
                assert!(a.to_f64().abs() < 1e-2, "piecewise linear y: no acceleration ({:?})", a);
            }
        }
        // Before the motion: the rest it started from
        assert_eq!(h.eval(now, now + 5000, 0), (real(x(0.0)), math::ZERO));
        // Samples closer than the spacing are merged: the history still covers the longest shaper
        let mut h = ShaperHistory::new();
        h.start(0, [math::ZERO; 2], [math::ZERO; 2]);
        let mut t = 0u32;
        for _ in 0..1000 {
            t += 300;
            h.push(t, [Real::from_lit(t as i64, 3); 2], [math::ONE_THOUSAND; 2]);
        }
        let (p, v) = h.eval(t, MAX_DURATION_US, 1);
        assert!((p.to_f64() - (t - MAX_DURATION_US) as f64 * 1e-3).abs() < 1e-3);
        assert!((v.to_f64() - 1000.0).abs() < 1e-2);
    }

    /// The pipeline of the stepper task, through the interpolator, the shaper and the real
    /// [StepPlan]: a sequence of XY moves (dx, dy mm; v0, v1 mm/s), from rest to rest
    struct Pipeline {
        /// Signed steps emitted by each motor
        motor: [i64; 2],
        /// Commanded steps (the interpolator)
        commanded: [i64; 2],
        /// Max |motor − ideal shaped position| at the window ends (steps)
        max_tracking: f64,
        /// The motion of the motors at the window ends: instant (s), position (continuous target,
        /// steps) and velocity (steps/s), shaped (or commanded without shaping)
        samples: Vec<(f64, [f64; 2], [f64; 2])>,
        /// When each commanded segment ended (s)
        segment_ends: Vec<f64>,
        windows: usize,
    }

    fn run(cfg: [ShaperConfig; 2], moves: &[(f64, f64, f64, f64)], v_max: f64) -> Pipeline {
        let constraints = Constraints {
            v_max: real(v_max),
            a_max: real(3000.0),
            j_max: real(100000.0),
        };
        let mut shaper = InputShaper::new(TICK_US);
        let spm = {
            let mut v = TVector::new();
            v.set_coord(CoordSel::X | CoordSel::Y, Some(real(SPM)));
            v
        };
        // The ideal commanded motion (steps): segments (start instant s, start, unit vector, profile)
        let mut segments: Vec<(f64, [f64; 2], [f64; 2], HybridMotionProfile)> = Vec::new();
        let mut ideal = [0.0f64; 2];
        let mut out = Pipeline {
            motor: [0; 2],
            commanded: [0; 2],
            max_tracking: 0.0,
            samples: vec![(0.0, [0.0; 2], [0.0; 2])],
            segment_ends: Vec::new(),
            windows: 0,
        };
        let mut t = 0.0f64;
        let mut shaping = false;
        for (dx, dy, v0, v1) in moves {
            let mut d = TVector::new();
            d.set_coord(CoordSel::X, Some(real(*dx)));
            d.set_coord(CoordSel::Y, Some(real(*dy)));
            let (u, len) = d.excluding_negligible().decompose_normal();
            let p = HybridMotionProfile::compute(
                len,
                real(*v0),
                real(*v1),
                #[cfg(feature = "with-motion-chaining")]
                math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                math::ZERO,
                &constraints,
                false,
            )
            .unwrap();
            // Where the segment starts, relative to the step the motor is at (direction frame)
            let mut offset = TVector::new();
            let uu = [0, 1].map(|k| u.get_coord(SHAPED_AXES[k]).map_or(0.0, |c| c.to_f64()));
            for k in 0..2 {
                if uu[k] != 0.0 {
                    let f = ideal[k] - out.commanded[k] as f64;
                    offset.set_coord(SHAPED_AXES[k], Some(real(if uu[k] < 0.0 { -f } else { f })));
                }
            }
            let mut interp = MicroSegmentInterpolator::new_with_start_offset(u.abs(), len, spm.with_coord(u.not_negligible_coords().complement(), None), offset);
            let v_start = p.eval_velocity(math::ZERO);
            shaping = shaper.begin(&cfg, &u, &spm, &offset, v_start) || shaping;
            segments.push((t, ideal, uu, p));
            let p = &segments.last().unwrap().3;
            let mut sampler = SegmentSampler::new_us(p, PERIOD_US);
            let mut v_prev = v_start;
            let dir = [0, 1].map(|k| uu[k] > 0.0);
            while let Some(s) = sampler.next() {
                let w = sampler.dt_us() as u32;
                let v = sampler.current_speed();
                interp.advance_to_with_velocity(s, Real::from_lit(w as i64, 0), v_prev, v, TICK_US).unwrap();
                v_prev = v;
                let mut mt = *interp.state();
                for k in 0..2 {
                    let n = mt.count(SHAPED_AXES[k]) as i64;
                    out.commanded[k] += if dir[k] { n } else { -n };
                }
                let mut dir_fwd = CoordSel::empty();
                for k in 0..2 {
                    dir_fwd.set(SHAPED_AXES[k], dir[k]);
                }
                if shaping {
                    shaper.advance(&mut mt, &mut dir_fwd, w, s, v);
                }
                t += w as f64 * 1e-6;
                execute(&mut out, mt, dir_fwd, w);
                if shaping {
                    // The ideal shaped position: from the profiles themselves, not the history
                    for k in 0..2 {
                        let ideal_shaped = shaped(&segments, &shaper.shapers[k], k, t);
                        out.max_tracking = out.max_tracking.max((out.motor[k] as f64 - ideal_shaped).abs());
                    }
                }
                let sample = match shaping {
                    true => motors(&shaper),
                    false => (
                        [0, 1].map(|k| ideal[k] + uu[k] * SPM * s.to_f64()),
                        [0, 1].map(|k| uu[k] * SPM * v.to_f64()),
                    ),
                };
                out.samples.push((t, sample.0, sample.1));
            }
            for k in 0..2 {
                ideal[k] += uu[k] * SPM * len.to_f64();
            }
            out.segment_ends.push(t);
        }
        // The stop: the shaped motion is completed
        while let Some((mt, dir_fwd)) = shaper.flush_window(PERIOD_US as u32) {
            t += PERIOD_US as f64 * 1e-6;
            execute(&mut out, mt, dir_fwd, PERIOD_US as u32);
            let (p, v) = motors(&shaper);
            out.samples.push((t, p, v));
            assert!(out.windows < 1_000_000);
        }
        assert!(shaper.is_at_rest());
        out
    }

    /// The shaped motion of the motors at the end of the last window: target (steps) and velocity
    /// (steps/s)
    fn motors(shaper: &InputShaper) -> ([f64; 2], [f64; 2]) {
        (
            [0, 1].map(|k| shaper.channels[k].target.to_f64()),
            [0, 1].map(|k| shaper.channels[k].rate.to_f64() / (TICK_US as f64 * 1e-6)),
        )
    }

    /// The ideal shaped position of an axis at `t`, from the profiles
    fn shaped(segments: &[(f64, [f64; 2], [f64; 2], HybridMotionProfile)], shaper: &Shaper, k: usize, t: f64) -> f64 {
        let x = |t: f64| -> f64 {
            let mut pos = segments[0].1[k];
            for (t0, start, u, p) in segments {
                if t < *t0 {
                    break;
                }
                let s = p.eval_position(real((t - t0).min(p.end_time().to_f64()))).to_f64();
                pos = start[k] + u[k] * SPM * s;
            }
            pos
        };
        impulses(shaper).iter().map(|(a, d)| a * x(t - d)).sum()
    }

    /// Executes a window tick by tick, as the step executor: its steps must be exactly those planned
    fn execute(out: &mut Pipeline, mt: MultiTimer, dir_fwd: CoordSel, width_us: u32) {
        let planned = [0, 1].map(|k| mt.count(SHAPED_AXES[k]) as i64);
        let mut plan = StepPlan::from(
            0,
            #[cfg(feature = "with-motion-broadcast")]
            hwa::MotionDelta::new(),
            mt,
            CoordSel::X | CoordSel::Y,
            dir_fwd,
        );
        let mut emitted = [0i64; 2];
        let mut ticks = 0u32;
        while ticks * TICK_US < width_us || plan.has_pending_steps() {
            if let Some(ch) = plan.next(TICK_US) {
                for k in 0..2 {
                    if ch.contains(SHAPED_AXES[k]) {
                        emitted[k] += 1;
                    }
                }
            }
            ticks += 1;
            assert!(ticks < 1_000_000);
        }
        for k in 0..2 {
            assert_eq!(emitted[k], planned[k], "the window emits exactly its steps");
            out.motor[k] += if dir_fwd.contains(SHAPED_AXES[k]) { emitted[k] } else { -emitted[k] };
        }
        out.windows += 1;
    }

    /// A square path with corners at speed (and a reversal): every shaper ends exactly at the
    /// commanded steps, tracks the ideal shaped motion within half a step, and its steps are exact
    #[test]
    fn the_shaped_motion_is_exact_and_ends_at_the_commanded_position() {
        let moves = [
            (20.0, 0.0, 0.0, 30.0),
            (0.0, 15.3, 30.0, 30.0),
            (-20.0, 0.0, 30.0, 25.0),
            (7.77, -10.1, 25.0, 0.0),
            (-0.4, 0.3, 0.0, 0.0),
            (5.0, 0.0, 0.0, 120.0),
            (-5.0, 0.0, 120.0, 0.0),
        ];
        let unshaped = run([ShaperConfig::disabled(); 2], &moves, 300.0);
        assert_eq!(unshaped.motor, unshaped.commanded);
        for kind in [ShaperKind::Zv, ShaperKind::Ei, ShaperKind::Mzv, ShaperKind::Zvd] {
            for (fx, fy) in [(40.0, 40.0), (23.0, 61.0), (0.0, 35.0)] {
                let run = run([config(kind, fx, 0.1), config(kind, fy, 0.15)], &moves, 300.0);
                assert_eq!(run.commanded, unshaped.commanded, "{:?}: commanded steps", kind);
                assert_eq!(run.motor, run.commanded, "{:?} {}/{} Hz: at rest at the commanded steps", kind, fx, fy);
                assert!(run.max_tracking <= 0.5 + 0.02, "{:?} {}/{} Hz: tracking {}", kind, fx, fy, run.max_tracking);
                assert!(run.windows > unshaped.windows, "the tail completes the shaped motion");
            }
        }
    }

    /// The point of it: the ringing left by the motion in an oscillator tuned as the shaper (40 Hz,
    /// ζ 0.1: the head on the frame), with and without shaping. The motors move as the step
    /// executor drives them: in each window the velocity ramps between the rates at its ends,
    /// offset so that the window covers exactly its displacement (the second-order DDA). The moves
    /// cruise between the corners (sharp, at speed), so the ringing is the oscillation 30 ms to
    /// 100 ms after each corner and after the stop: longer than any of the shapers, which can not
    /// prevent the first oscillation, only cancel it with their next impulses
    #[test]
    fn the_shaped_motion_leaves_much_less_vibration() {
        let moves = [
            (30.0, 0.0, 0.0, 40.0),
            (0.0, 30.0, 40.0, 40.0),
            (-30.0, 0.0, 40.0, 40.0),
            (0.0, -30.0, 40.0, 0.0),
        ];
        let ringing = |run: &Pipeline| -> f64 {
            let (w, zeta) = (2.0 * core::f64::consts::PI * 40.0, 0.1);
            let watched = |t: f64| run.segment_ends.iter().any(|c| t >= c + 0.03 && t <= c + 0.1);
            let mut peak = 0.0f64;
            for k in 0..2 {
                // z'' + 2ζω z' + ω² z = −a: the head relative to the motor (mm)
                let (mut z, mut zv, mut v_end) = (0.0f64, 0.0f64, 0.0f64);
                let mut integrate = |t0: f64, t1: f64, a: f64, z: &mut f64, zv: &mut f64| {
                    let n = ((t1 - t0) / 2e-5).ceil().max(1.0) as usize;
                    let h = (t1 - t0) / n as f64;
                    for i in 1..=n {
                        *zv += h * (-a - 2.0 * zeta * w * *zv - w * w * *z);
                        *z += h * *zv;
                        if watched(t0 + h * i as f64) {
                            peak = peak.max(z.abs());
                        }
                    }
                };
                for pair in run.samples.windows(2) {
                    let ((t0, p0, v0), (t1, p1, v1)) = (pair[0], pair[1]);
                    let dt = t1 - t0;
                    let a = (v1[k] - v0[k]) / dt / SPM;
                    // The velocity of the window: its mean (exact displacement) ± half its ramp
                    let mean = (p1[k] - p0[k]) / dt / SPM;
                    let v_start = mean - a * dt / 2.0;
                    zv -= v_start - v_end;
                    integrate(t0, t1, a, &mut z, &mut zv);
                    v_end = mean + a * dt / 2.0;
                }
                // Stopped
                let t_end = run.samples.last().unwrap().0;
                zv += v_end;
                integrate(t_end, t_end + 0.1, 0.0, &mut z, &mut zv);
            }
            peak
        };
        let unshaped = ringing(&run([ShaperConfig::disabled(); 2], &moves, 40.0));
        for kind in [ShaperKind::Zv, ShaperKind::Ei, ShaperKind::Mzv, ShaperKind::Zvd] {
            let shaped = ringing(&run([config(kind, 40.0, 0.1); 2], &moves, 40.0));
            assert!(shaped < 0.1 * unshaped, "{:?}: ringing {} mm vs {} mm unshaped", kind, shaped, unshaped);
        }
    }

    /// Disabled (both axes), nothing is done; an abort reports what the motors lag and restarts
    /// the frame there
    #[test]
    fn disabled_or_aborted() {
        let spm = {
            let mut v = TVector::new();
            v.set_coord(CoordSel::X | CoordSel::Y, Some(real(SPM)));
            v
        };
        let (u, _) = {
            let mut d = TVector::new();
            d.set_coord(CoordSel::X, Some(real(1.0)));
            d.decompose_normal()
        };
        let mut shaper = InputShaper::new(TICK_US);
        assert!(!shaper.begin(&[ShaperConfig::disabled(); 2], &u, &spm, &TVector::new(), real(10.0)));
        assert!(shaper.is_at_rest());
        assert!(shaper.flush_window(2500).is_none());
        assert_eq!(shaper.abort(), [0, 0]);
        // Shaping X only: Y keeps the commanded steps
        assert!(shaper.begin(&[config(ShaperKind::Zv, 40.0, 0.1), ShaperConfig::disabled()], &u, &spm, &TVector::new(), real(50.0)));
        let mut mt = MultiTimer::new();
        mt.set_displace_width(2500);
        mt.set_count(CoordSel::X, 10);
        mt.set_count(CoordSel::Y, 3);
        let mut dir_fwd = CoordSel::X | CoordSel::Y;
        shaper.advance(&mut mt, &mut dir_fwd, 2500, real(0.125), real(50.0));
        assert_eq!(mt.count(CoordSel::Y), 3, "Y is not shaped");
        let shaped_x = mt.count(CoordSel::X) as i32;
        assert!(shaped_x < 10, "X lags");
        // The motors stop where they are
        assert_eq!(shaper.abort(), [shaped_x - 10, 0]);
        assert!(shaper.is_at_rest());
    }
}
