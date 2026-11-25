//! Pressure advance (Marlin's linear advance, Klipper's pressure advance). Feature
//! `with-pressure-advance`, set with `M900 K<seconds>` (`0`, the default, disables it).
//!
//! The melt pressure in the nozzle lags the extruder motor: the flow follows the filament feed
//! with a delay, so the extrusion falls short when the head accelerates and is in excess when it
//! brakes (bulging corners, blobs and gaps at the ends of the lines). Pressure advance feeds the
//! filament ahead of its nominal position by `K` times its nominal speed:
//!
//! `e_motor(t) = e(t) + K · ė(t)`
//!
//! As Marlin and Klipper, only the extruding moves are advanced: the extruder moving forward along
//! with some other axis. Neither the retractions nor the extruder-only moves are.
//!
//! # Within a segment
//!
//! A segment of path length `D` moves the extruder `u_e·D` (`u_e` is the E component of its unit
//! vector, the flow rate included) along its motion profile `s(t)` (`v = ṡ`, `a = v̇`). In
//! extruder steps (`spm` steps per space unit) the lead is `κ·v(t)`, `κ = K·u_e·spm`, and its rate
//! `κ·a(t)`: continuous, as the acceleration of the S-curve profiles is (no motor velocity jumps,
//! unlike with trapezoidal profiles).
//!
//! # Junctions
//!
//! The lead is continuous across a junction when both segments extrude at the same ratio (same
//! `u_e`: the path speed is continuous). When they do not (an extrusion followed by a travel, a
//! change of line width...), the mismatch `Δ` (the lead carried from the previous segment minus the
//! lead `κ·v₀` at the start of this one) is spread along the segment, proportionally to the path:
//! the target of the motor, relative to where it is when the segment starts, is
//!
//! `p(s, v) = (u_e·spm − Δ/D)·s + κ·(v − v₀)`
//!
//! continuous at both ends (`0` at the start, the lead `κ·v` of the segment at its end): the
//! extrusion ratio of the segment is changed by `−Δ/D`, nothing else.
//!
//! # Exactness
//!
//! At the end of every window the motor is at the step closest to its target (the half-step rule
//! of [super::MicroSegmentInterpolator]). What the motor is ahead of the nominal extruder position
//! (the lead plus the rounding residue, within ±½ step) is carried from segment to segment, so no
//! step is ever lost nor added. The steps of a window are exact (the step executor never drops
//! them); the direction of the extruder is decided per window: it retracts while braking when
//! `κ·|a| > u_e·spm·v`, the pressure relief.
//!
//! When the motion stops (the queue runs empty, a dwell, a homing) the lead is released by an
//! extruder-only window ([PressureAdvance::release]): the motor is then at the nominal position, to
//! the closest step.
//!
//! The extruder is moved by the [super::MicroSegmentInterpolator], as before, in the segments that
//! do not carry a lead: neither advanced nor following an advanced one. Then the motion is exactly
//! the one without pressure advance (also with `K = 0`): the stepper task carries the rounding
//! residue of those segments in [PressureAdvance] too (see [PressureAdvance::set_lead_steps]).
//!
//! The positions are not affected: the extruder position is tracked as a displacement from zero
//! per segment (see `MotionPlanner::schedule_displacement`), and the lead lives only here.
use crate::hwa;
use hwa::CoordSel;
use hwa::controllers::MultiTimer;
use hwa::math;
use math::{Real, TVector};

use super::micro_segment_interpolation::{RampWindowCache, SignedChannel};

/// The pressure advance state of the extruder, carried across the segments by the stepper task
pub struct PressureAdvance {
    /// Motor position minus nominal position of the extruder (steps) where the last segment
    /// ended: the lead plus the rounding residue (within ±½ step)
    lead_steps: Real,
}

impl PressureAdvance {
    pub const fn new() -> Self {
        Self {
            lead_steps: math::ZERO,
        }
    }

    /// Motor position minus nominal position of the extruder (steps)
    #[inline]
    pub fn lead_steps(&self) -> Real {
        self.lead_steps
    }

    /// Sets what the motor is ahead of its nominal position (steps) after a segment whose extruder
    /// the [super::MicroSegmentInterpolator] moved: its rounding residue, within ±½ step
    #[inline]
    pub fn set_lead_steps(&mut self, steps: Real) {
        self.lead_steps = steps;
    }

    /// Whether the motor is ahead of (or behind) its nominal position by a step or more: more
    /// than the rounding residue of its position
    #[inline]
    pub fn is_leading(&self) -> bool {
        self.lead_steps.abs() > math::HALF
    }

    /// Whether a segment is advanced with `k`: positive, the extruder moving forward along with
    /// some other axis (an extrusion, not a retraction nor an extruder-only move)
    pub fn is_advanced(k: Real, unit_vector_dir: &TVector<Real>) -> bool {
        k.is_defined_positive()
            && unit_vector_dir
                .get_coord(CoordSel::E)
                .is_some_and(|u| u.is_defined_positive())
            && unit_vector_dir
                .not_negligible_coords()
                .intersects(CoordSel::motion_relevant_axis())
    }

    /// The extruder generator of a segment about to be executed, when it is advanced (see
    /// [Self::is_advanced]) or the motor carries a lead from the previous one. `None` otherwise:
    /// the extruder is moved by the [super::MicroSegmentInterpolator], as without pressure
    /// advance, starting from the rounding residue kept (see [Self::set_lead_steps]).
    ///
    /// * `unit_vector_dir`, `displacement_su`: the segment (as executed: after the deviation fix).
    /// * `steps_per_su`: extruder steps per space unit.
    /// * `v0`, `a0`: speed and acceleration of the motion profile at its start.
    /// * `tick_us`: the period of the step executor tick.
    pub fn begin(
        &mut self,
        k: Real,
        unit_vector_dir: &TVector<Real>,
        displacement_su: Real,
        steps_per_su: Real,
        v0: Real,
        a0: Real,
        tick_us: u32,
    ) -> Option<ExtruderAdvance> {
        let advanced = Self::is_advanced(k, unit_vector_dir);
        if !(advanced || self.is_leading())
            || !displacement_su.is_defined_positive()
            || !steps_per_su.is_defined_positive()
        {
            return None;
        }
        let nominal = unit_vector_dir.get_coord(CoordSel::E).unwrap_or(math::ZERO) * steps_per_su;
        let kappa = match advanced {
            true => k * nominal,
            false => math::ZERO,
        };
        // The lead mismatch, spread along the segment
        let ratio = nominal - (self.lead_steps - kappa * v0) / displacement_su;
        let tick_s = Real::from_lit(tick_us.max(1) as i64, 0) * Real::from_f32(1.0e-6);
        let mut channel = SignedChannel::new(0, nominal.is_positive());
        channel.rate = (ratio * v0 + kappa * a0) * tick_s;
        Some(ExtruderAdvance {
            ratio,
            kappa,
            v0,
            nominal,
            lead_start: self.lead_steps,
            tick_s,
            s: math::ZERO,
            channel,
            ramps: RampWindowCache::new(tick_us),
            #[cfg(any(test, feature = "assert-motion"))]
            count: 0,
        })
    }

    /// Updates the lead with what the extruder did in the segment (all its windows, or those
    /// executed before an abort)
    pub fn end(&mut self, advance: &ExtruderAdvance) {
        // Motor minus nominal position, both relative to where the motor was at the start
        self.lead_steps = Real::from_lit(advance.channel.motor as i64, 0) + advance.lead_start
            - advance.nominal * advance.s;
    }

    /// Releases the lead (the motion stops): an extruder-only window of `width_us` that moves the
    /// motor to the step closest to its nominal position. Returns the window and its direction
    /// (`true`: forward), or `None` when there is no lead (only a rounding residue).
    pub fn release(&mut self, width_us: u32, tick_us: u32) -> Option<(MultiTimer, bool)> {
        if !self.is_leading() {
            return None;
        }
        let steps = (self.lead_steps + math::HALF).floor();
        let n = steps.to_i32()?;
        self.lead_steps -= steps;
        let count = n.unsigned_abs();
        let width_us = width_us.max(1);
        let mut multi_timer = MultiTimer::new();
        multi_timer.set_displace_width(width_us);
        multi_timer.set_channel_uniform(CoordSel::E, Some(width_us / count), tick_us);
        multi_timer.set_count(CoordSel::E, count);
        Some((multi_timer, n < 0))
    }
}

/// Generates the extruder steps of one segment with pressure advance (see the module
/// documentation), window by window, alongside the [super::MicroSegmentInterpolator] (which then
/// moves the other axes only)
pub struct ExtruderAdvance {
    /// Extruder steps per space unit along the path (the lead mismatch spread included)
    ratio: Real,
    /// The lead per unit of path speed (steps · s / space unit): `K·u_e·spm`. Zero when the
    /// segment is not advanced (a lead carried from the previous segment is being spread)
    kappa: Real,
    /// Path speed at the start of the segment
    v0: Real,
    /// Nominal extruder steps per space unit along the path (`u_e·spm`)
    nominal: Real,
    /// Lead (steps) when the segment started
    lead_start: Real,
    /// Tick period (s)
    tick_s: Real,
    /// Path position at the end of the last window
    s: Real,
    /// The motor, relative to the segment start
    channel: SignedChannel,
    ramps: RampWindowCache,
    /// Steps of the last window
    #[cfg(any(test, feature = "assert-motion"))]
    pub count: u32,
}

impl ExtruderAdvance {
    /// Adds the extruder channel of the window ending at path position `s` with speed `v` and
    /// acceleration `a` (as the profile gives them), `width_us` wide, to `multi_timer` (the window
    /// of the other axes). Its direction is then [Self::forward].
    pub fn advance(&mut self, multi_timer: &mut MultiTimer, s: Real, v: Real, a: Real, width_us: u32) {
        let target = self.ratio * s + self.kappa * (v - self.v0);
        let rate = (self.ratio * v + self.kappa * a) * self.tick_s;
        self.ramps.set_width(width_us);
        let _n = self.channel.advance(multi_timer, CoordSel::E, target, rate, &mut self.ramps);
        #[cfg(any(test, feature = "assert-motion"))]
        {
            self.count = _n.unsigned_abs();
        }
        self.s = s;
    }

    /// The direction of the extruder (`true`: forward) in the last window that moved it
    #[inline]
    pub fn forward(&self) -> bool {
        self.channel.forward
    }
}

#[cfg(all(test, feature = "with-x-axis"))]
mod tests {
    use super::*;
    use crate::hwa::controllers::StepPlan;
    use crate::motion::{Constraints, HybridMotionProfile, MotionProfile, SegmentSampler};

    const TICK_US: u32 = 10;
    const PERIOD_US: u64 = 2500;
    /// Extruder steps per mm
    const SPM: f32 = 400.0;

    fn real(x: f64) -> Real {
        Real::from_f32(x as f32)
    }

    /// The unit vector of a move of `xy` mm in X extruding `e` mm, and its length
    fn direction(xy: f64, e: f64) -> (TVector<Real>, Real) {
        let mut v = TVector::new();
        if xy != 0.0 {
            v.set_coord(CoordSel::X, Some(real(xy)));
        }
        if e != 0.0 {
            v.set_coord(CoordSel::E, Some(real(e)));
        }
        v.decompose_normal()
    }

    fn profile(d: Real, v0: f64, v1: f64) -> HybridMotionProfile {
        HybridMotionProfile::compute(
            d,
            real(v0),
            real(v1),
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
            &Constraints {
                v_max: real(100.0),
                a_max: real(3000.0),
                j_max: real(100000.0),
            },
            false,
        )
        .unwrap()
    }

    /// What the extruder did in a segment, as the step executor would do it
    #[derive(Default)]
    struct Run {
        /// Signed steps emitted (counted by executing the windows tick by tick)
        steps: i64,
        /// Steps emitted backwards
        backwards: i64,
        /// Max |motor − (nominal + κ·(v − v₀))| at the window ends (steps, relative to the start)
        max_tracking_error: f64,
    }

    /// Executes a segment through the generator (and the real [StepPlan], tick by tick): the
    /// stepper task loop. Returns `None` when the generator is not used
    fn run_segment(
        pa: &mut PressureAdvance,
        k: f64,
        (xy, e): (f64, f64),
        (v0, v1): (f64, f64),
    ) -> Option<Run> {
        let (dir, d) = direction(xy, e);
        let p = profile(d, v0, v1);
        let mut advance = pa.begin(
            real(k),
            &dir,
            d,
            real(SPM as f64),
            p.eval_velocity(math::ZERO),
            p.eval_acceleration(math::ZERO),
            TICK_US,
        )?;
        let u_e = dir.get_coord(CoordSel::E).unwrap_or(math::ZERO).to_f64();
        let kappa = match PressureAdvance::is_advanced(real(k), &dir) {
            true => k * u_e * SPM as f64,
            false => 0.0,
        };
        let mut sampler = SegmentSampler::new_us(&p, PERIOD_US);
        let mut run = Run::default();
        while let Some(s) = sampler.next() {
            let mut mt = MultiTimer::new();
            mt.set_displace_width(sampler.dt_us() as u32);
            advance.advance(&mut mt, s, sampler.current_speed(), sampler.current_acceleration(), sampler.dt_us() as u32);
            let mut plan = StepPlan::from(
                0,
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
                mt,
                CoordSel::E,
                CoordSel::E,
            );
            let mut ticks = 0u32;
            let mut emitted = 0i64;
            while ticks * TICK_US < sampler.dt_us() as u32 || plan.has_pending_steps() {
                if plan.next(TICK_US).is_some_and(|ch| ch.contains(CoordSel::E)) {
                    emitted += 1;
                }
                ticks += 1;
                assert!(ticks < 1_000_000);
            }
            assert_eq!(emitted, advance.count as i64, "the window emits exactly its steps");
            let signed = if advance.forward() { emitted } else { -emitted };
            run.steps += signed;
            if signed < 0 {
                run.backwards += emitted;
            }
            // Motor ≈ nominal + lead (plus the mismatch being spread)
            let expected = u_e * SPM as f64 * s.to_f64() + kappa * (sampler.current_speed().to_f64() - v0);
            run.max_tracking_error = run.max_tracking_error.max((run.steps as f64 - expected).abs());
        }
        pa.end(&advance);
        Some(run)
    }

    /// Without K, and with K on moves that are not extrusions, nothing is generated (the
    /// interpolator moves the extruder, as without the feature)
    #[test]
    fn only_extrusions_are_advanced() {
        let k = real(0.05);
        let (extrusion, _) = direction(10.0, 0.5);
        let (retraction_while_moving, _) = direction(10.0, -0.5);
        let (extruder_only, _) = direction(0.0, 1.0);
        let (travel, _) = direction(10.0, 0.0);
        assert!(PressureAdvance::is_advanced(k, &extrusion));
        assert!(!PressureAdvance::is_advanced(math::ZERO, &extrusion));
        assert!(!PressureAdvance::is_advanced(-k, &extrusion));
        assert!(!PressureAdvance::is_advanced(k, &retraction_while_moving));
        assert!(!PressureAdvance::is_advanced(k, &extruder_only));
        assert!(!PressureAdvance::is_advanced(k, &travel));

        let mut pa = PressureAdvance::new();
        assert!(run_segment(&mut pa, 0.0, (10.0, 0.5), (0.0, 0.0)).is_none());
        assert!(run_segment(&mut pa, 0.05, (0.0, 1.0), (0.0, 0.0)).is_none());
        assert!(run_segment(&mut pa, 0.05, (10.0, 0.0), (0.0, 0.0)).is_none());
        assert!(!pa.is_leading());
        assert_eq!(pa.release(2500, TICK_US).map(|(_, f)| f), None);
    }

    /// An advanced extrusion between rests: every step exact, the motor tracks the nominal
    /// position plus the lead within half a step, it retracts while braking (the more the larger
    /// K), and it ends at its nominal position (no lead at rest)
    #[test]
    fn an_advanced_extrusion_tracks_the_lead_and_ends_at_its_nominal_position() {
        for k in [0.02, 0.05, 0.2] {
            let mut pa = PressureAdvance::new();
            let run = run_segment(&mut pa, k, (40.0, 2.0), (0.0, 0.0)).unwrap();
            let nominal = 2.0 * SPM as f64;
            assert_eq!(run.steps, nominal as i64, "K={}: net steps", k);
            assert!(pa.lead_steps().abs() <= math::HALF, "K={}: lead at rest {:?}", k, pa.lead_steps());
            assert!(!pa.is_leading());
            assert!(run.backwards > 0, "K={}: the pressure relief (braking) retracts", k);
            // Within half a step: the closest step to the target
            assert!(run.max_tracking_error <= 0.5 + 1e-2, "K={}: tracking error {}", k, run.max_tracking_error);
        }
    }

    /// The same extrusion without K: the generator is not used; with a tiny K the motor never
    /// retracts but the steps are the same
    #[test]
    fn a_small_k_never_retracts() {
        let mut pa = PressureAdvance::new();
        let run = run_segment(&mut pa, 0.0005, (40.0, 2.0), (0.0, 0.0)).unwrap();
        assert_eq!(run.steps, (2.0 * SPM) as i64);
        assert_eq!(run.backwards, 0);
    }

    /// A chain of extrusions (same ratio) through junctions at speed, then a travel at speed and a
    /// stop: the lead is carried (continuous) through the junctions, spread along the travel, and
    /// no step is lost: the steps add up to the nominal extrusion and the residue is dropped only
    /// once nothing leads (within half a step)
    #[test]
    fn the_lead_is_carried_through_junctions_and_spread_along_a_travel() {
        let k = 0.05;
        let mut pa = PressureAdvance::new();
        let mut steps = 0i64;
        // Odd extrusions per segment: not a whole number of steps each (the residue is carried)
        for (v0, v1) in [(0.0, 50.0), (50.0, 60.0), (60.0, 60.0), (60.0, 40.0)] {
            let run = run_segment(&mut pa, k, (3.0, 0.1234), (v0, v1)).unwrap();
            steps += run.steps;
            assert!(pa.is_leading(), "a lead at speed");
        }
        let lead = pa.lead_steps().to_f64();
        let (dir, _) = direction(3.0, 0.1234);
        let expected_lead = k * dir.get_coord(CoordSel::E).unwrap().to_f64() * SPM as f64 * 40.0;
        assert!((lead - expected_lead).abs() <= 0.5 + 1e-3, "lead {} expected {}", lead, expected_lead);
        // A travel at speed: not advanced, the lead is spread along it
        let run = run_segment(&mut pa, k, (5.0, 0.0), (40.0, 0.0)).unwrap();
        steps += run.steps;
        assert!(run.steps < 0, "the travel retracts the lead");
        assert!(!pa.is_leading(), "the lead is gone after the travel: {:?}", pa.lead_steps());
        // Exact up to the last residue
        let nominal = 4.0 * 0.1234 * SPM as f64;
        assert!((steps as f64 - nominal).abs() <= 0.5, "{} steps, nominal {}", steps, nominal);
        // The next travel: nothing to carry but the residue, kept for the next extrusion
        let residue = pa.lead_steps();
        assert!(run_segment(&mut pa, k, (5.0, 0.0), (0.0, 0.0)).is_none());
        assert_eq!(pa.lead_steps(), residue);
    }

    /// A segment starting with a lead mismatch (a ratio change at speed) reaches the lead of its
    /// own ratio at its end
    #[test]
    fn a_ratio_change_is_spread_along_the_segment() {
        let k = 0.05;
        let mut pa = PressureAdvance::new();
        run_segment(&mut pa, k, (3.0, 0.1), (0.0, 50.0)).unwrap();
        run_segment(&mut pa, k, (3.0, 0.2), (50.0, 50.0)).unwrap();
        let (dir, _) = direction(3.0, 0.2);
        let expected = k * dir.get_coord(CoordSel::E).unwrap().to_f64() * SPM as f64 * 50.0;
        assert!((pa.lead_steps().to_f64() - expected).abs() <= 0.5 + 1e-3, "{:?} vs {}", pa.lead_steps(), expected);
    }

    /// The lead is released at a stop by an extruder-only window that leaves the motor at its
    /// nominal position (within the residue). Its steps are exact
    #[test]
    fn the_lead_is_released_at_a_stop() {
        let k = 0.1;
        let mut pa = PressureAdvance::new();
        let run = run_segment(&mut pa, k, (3.0, 0.1), (0.0, 30.0)).unwrap();
        let lead_before = pa.lead_steps();
        assert!(pa.is_leading());
        let (mt, forward) = pa.release(2500, TICK_US).unwrap();
        assert!(!forward, "a lead ahead is released backwards");
        assert!(pa.lead_steps().abs() <= math::HALF);
        let mut plan = StepPlan::from(
            0,
            #[cfg(feature = "with-motion-broadcast")]
            hwa::MotionDelta::new(),
            mt,
            CoordSel::E,
            CoordSel::empty(),
        );
        let (mut emitted, mut ticks) = (0i64, 0u32);
        while ticks * TICK_US < 2500 || plan.has_pending_steps() {
            if plan.next(TICK_US).is_some_and(|ch| ch.contains(CoordSel::E)) {
                emitted += 1;
            }
            ticks += 1;
            assert!(ticks < 100_000);
        }
        assert_eq!(emitted as f64, (lead_before - pa.lead_steps()).to_f64().round());
        // The motor ends at its nominal position (the extrusion), to the closest step
        assert!(((run.steps - emitted) as f64 - 0.1 * SPM as f64).abs() <= 0.5);
        // Nothing more to release
        assert!(pa.release(2500, TICK_US).is_none());
        // A lead behind (the motor retracted past its nominal position) is released forwards
        let mut behind = PressureAdvance { lead_steps: real(-3.2) };
        let (_, forward) = behind.release(2500, TICK_US).unwrap();
        assert!(forward);
        assert!((behind.lead_steps().to_f64() + 0.2).abs() < 1e-5);
    }

    /// An abort ends the segment where its last window ended: the lead is what the motor is ahead
    /// of its nominal position there
    #[test]
    fn an_interrupted_segment_keeps_its_lead_consistent() {
        let k = 0.05;
        let mut pa = PressureAdvance::new();
        let (dir, d) = direction(30.0, 1.0);
        let p = profile(d, 0.0, 0.0);
        let mut advance = pa
            .begin(real(k), &dir, d, real(SPM as f64), math::ZERO, math::ZERO, TICK_US)
            .unwrap();
        let mut sampler = SegmentSampler::new_us(&p, PERIOD_US);
        let mut motor = 0i64;
        for _ in 0..10 {
            let s = sampler.next().unwrap();
            let mut mt = MultiTimer::new();
            advance.advance(&mut mt, s, sampler.current_speed(), sampler.current_acceleration(), PERIOD_US as u32);
            motor += if advance.forward() { advance.count as i64 } else { -(advance.count as i64) };
        }
        pa.end(&advance);
        let u_e = dir.get_coord(CoordSel::E).unwrap().to_f64();
        let nominal = u_e * SPM as f64 * sampler.current_position().to_f64();
        assert!((pa.lead_steps().to_f64() - (motor as f64 - nominal)).abs() < 1e-2);
        let expected = k * u_e * SPM as f64 * sampler.current_speed().to_f64();
        assert!((pa.lead_steps().to_f64() - expected).abs() <= 0.5 + 1e-2, "{:?} vs {}", pa.lead_steps(), expected);
    }

    /// The extruder rate ramps between the rates of the target at the window ends (second-order
    /// DDA): the steps are emitted when the (advanced) target crosses them, also in the windows
    /// where the extruder reverses
    #[test]
    fn the_extruder_steps_follow_the_target_in_time() {
        let k = 0.05;
        let mut pa = PressureAdvance::new();
        let (dir, d) = direction(40.0, 2.0);
        let p = profile(d, 0.0, 0.0);
        let u_e = dir.get_coord(CoordSel::E).unwrap().to_f64();
        let mut advance = pa
            .begin(real(k), &dir, d, real(SPM as f64), math::ZERO, math::ZERO, TICK_US)
            .unwrap();
        let mut sampler = SegmentSampler::new_us(&p, PERIOD_US);
        let (mut t_us, mut motor) = (0u64, 0i64);
        let mut max_lag = 0.0f64;
        while let Some(s) = sampler.next() {
            let w = sampler.dt_us() as u32;
            let mut mt = MultiTimer::new();
            mt.set_displace_width(w);
            advance.advance(&mut mt, s, sampler.current_speed(), sampler.current_acceleration(), w);
            let step = if advance.forward() { 1 } else { -1 };
            let mut plan = StepPlan::from(
                0,
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
                mt,
                CoordSel::E,
                CoordSel::E,
            );
            let mut ticks = 0u32;
            while ticks * TICK_US < w || plan.has_pending_steps() {
                ticks += 1;
                if plan.next(TICK_US).is_some_and(|ch| ch.contains(CoordSel::E)) {
                    motor += step;
                    // The target at this instant
                    let t = (t_us + (ticks * TICK_US) as u64) as f64 * 1e-6;
                    let target = u_e * SPM as f64 * (p.eval_position(real(t)).to_f64() + k * p.eval_velocity(real(t)).to_f64());
                    max_lag = max_lag.max((motor as f64 - target).abs());
                }
            }
            t_us += w as u64;
        }
        // Each step is emitted when the target crosses its half step (within a tick)
        assert!(max_lag < 0.6, "the steps deviate {} steps from the target", max_lag);
    }
}
