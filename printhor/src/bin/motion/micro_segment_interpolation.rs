//! Motion Interpolation (sampling) strategies

use crate::hwa;
use hwa::CoordSel;
use hwa::controllers::MultiTimer;
use hwa::math;
use math::{Real, TVector};

/// A struct for interpolating microsteps along linear trajectories.
pub struct MicroSegmentInterpolator {
    /// The direction (unitary) vector in positive coordinates.
    v_dir_abs: TVector<Real>,
    /// The unit director vector
    u_steps_per_space_magnitude: TVector<Real>,

    /// Number of discrete steps along vector already advanced.
    u_steps_advanced: TVector<u32>,

    /// The number of discrete steps by axis already advanced.
    axis_steps_advanced_precise: TVector<Real>,

    /// The number of discrete steps by axis to advance.
    axis_steps_to_advance_precise: TVector<Real>,

    /// Continuous (fractional) position in steps by axis at the end of the last window.
    last_step_pos: TVector<Real>,

    /// Continuous position (steps, in the direction of motion of each axis) of the ideal start
    /// of the segment, relative to the step the motors are at. See [Self::new_with_start_offset]
    start_offset: TVector<Real>,

    /// An instance to hold and manage the stepping
    multi_timer: MultiTimer,

    /// The carried delta steps, for debugging
    #[cfg(any(test, feature = "assert-motion"))]
    pub delta: TVector<u32>,
}

const HALF_STEP: TVector<Real> = TVector::new_with_coord(CoordSel::all_axis(), Some(math::HALF));

impl MicroSegmentInterpolator {
    /// Constructs a new `LinearMicrosegmentStepInterpolator`.
    ///
    /// # Arguments
    ///
    /// * `v_dir_abs` - The direction (unitary) vector in positive coordinates.
    /// * `distance` - The distance to move in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE].
    /// * `u_steps_per_space_magnitude` - Micro-steps per [hwa::HwiContract::SPACE_UNIT_MAGNITUDE].
    ///
    /// # Returns
    ///
    /// A new `LinearMicrosegmentStepInterpolator`.
    pub fn new(
        v_dir_abs: TVector<Real>,
        distance: Real,
        u_steps_per_space_magnitude: TVector<Real>,
    ) -> Self {
        Self::new_with_start_offset(v_dir_abs, distance, u_steps_per_space_magnitude, TVector::new())
    }

    /// Like [Self::new], for a segment whose ideal start is not exactly at the step the motors
    /// are at (a curve approximated by short segments, or any target that is not a multiple of
    /// the step, leaves the motors at the closest step of an ideal, continuous position).
    ///
    /// `start_offset` is, per axis, the ideal start minus the current step position, in steps
    /// and in the direction of motion of the axis (positive: the ideal start is ahead). It is
    /// clamped to ±½ step (larger deviations must be corrected by redirecting the segment).
    ///
    /// Each step is then emitted when the *ideal* position crosses the next half step, so that
    /// the motors follow the ideal segment within ±½ step per axis: without the offset, the path
    /// would be the segment shifted by up to ½ step at its start (up to 1 step of total error).
    pub fn new_with_start_offset(
        v_dir_abs: TVector<Real>,
        distance: Real,
        u_steps_per_space_magnitude: TVector<Real>,
        start_offset: TVector<Real>,
    ) -> Self {
        let start_offset: TVector<Real> = v_dir_abs.map_values(|coord, _| {
            Some(
                start_offset
                    .get_coord(coord)
                    .unwrap_or(math::ZERO)
                    .clamp(-math::HALF, math::HALF),
            )
        });
        Self {
            v_dir_abs,
            u_steps_per_space_magnitude,
            u_steps_advanced: TVector::zero(),
            axis_steps_advanced_precise: TVector::zero(),
            // Steps whose half-step crossing lies within [start_offset, start_offset + displacement]
            axis_steps_to_advance_precise: ((v_dir_abs * distance * u_steps_per_space_magnitude)
                .clamp_higher_than(TVector::zero())
                + start_offset
                + HALF_STEP)
                .floor()
                .clamp_higher_than(TVector::zero()),
            last_step_pos: start_offset,
            start_offset,
            multi_timer: MultiTimer::new(),
            #[cfg(any(test, feature = "assert-motion"))]
            delta: TVector::zero(),
        }
    }

    /// Advances the position to the given estimated position, distributing the steps of the
    /// window **uniformly** (constant step rate inside the window).
    ///
    /// # Arguments
    ///
    /// * `estimated_position` - The estimated position to advance to.
    /// * `width` - The window width in µs.
    ///
    /// # Returns
    ///
    /// A boolean indicating whether advancement is successful.
    pub fn advance_to(&mut self, estimated_position: Real, width: Real) -> Result<bool, ()> {
        self.advance(
            estimated_position,
            width,
            None,
            crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US,
        )
    }

    /// Advances the position to the given estimated position with a **linearly varying step rate**
    /// (second-order DDA): the per-axis rate goes from the profile velocity at the start of the
    /// window (`v_start`) to the one at its end (`v_end`), adjusted so that the integrated rate is
    /// exactly the displacement of the window. The velocity seen by the motors is then
    /// continuous and piecewise linear across windows, instead of a staircase.
    ///
    /// * `v_start`, `v_end`: path speed (space units / s) at the window boundaries.
    /// * `tick_us`: period of the step generator tick ([`crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US`]).
    pub fn advance_to_with_velocity(
        &mut self,
        estimated_position: Real,
        width: Real,
        v_start: Real,
        v_end: Real,
        tick_us: u32,
    ) -> Result<bool, ()> {
        self.advance(estimated_position, width, Some((v_start, v_end)), tick_us)
    }

    fn advance(
        &mut self,
        estimated_position: Real,
        width: Real,
        velocities: Option<(Real, Real)>,
        tick_us: u32,
    ) -> Result<bool, ()> {
        let axial_pos: TVector<Real> = self.v_dir_abs * estimated_position;
        let step_pos: TVector<Real> =
            axial_pos * self.u_steps_per_space_magnitude + self.start_offset;

        let advanced_before = self.axis_steps_advanced_precise;
        let p0 = self.last_step_pos;

        let steps_to_advance = ((step_pos - self.axis_steps_advanced_precise) + HALF_STEP)
            .clamp_higher_than(TVector::zero())
            .clamp_lower_than(self.axis_steps_to_advance_precise);
        let steps_to_advance_precise = steps_to_advance.floor();
        self.axis_steps_advanced_precise += steps_to_advance_precise;
        self.last_step_pos = step_pos;

        #[cfg(any(test, feature = "assert-motion"))]
        {
            self.delta = steps_to_advance_precise.map(|_c, v| {
                v.unwrap_or(math::ZERO)
                    .to_i32()
                    .and_then(|i| Some(i as u32))
            });
        }

        let can_advance_more = self
            .axis_steps_advanced_precise
            .bounded_by(&self.axis_steps_to_advance_precise);

        let numerator = width.to_i32().unwrap_or(0).max(0) as u32;
        if numerator == 0 {
            return Err(());
        }
        let tick_us = tick_us.max(1);
        // Ticks the executor will run for this window (it closes it at tick_count >= width)
        let n_ticks = (numerator + tick_us - 1) / tick_us;

        let step_increment = steps_to_advance_precise
            .map_values(|_, cv| cv.to_i32().and_then(|c| Some(c as u32)));

        let mut mt = self.multi_timer;
        mt.set_displace_width(numerator);
        mt.set_max_count(&step_increment);
        let v_dir_abs = self.v_dir_abs;
        let spm = self.u_steps_per_space_magnitude;
        // Per-window constants, hoisted out of the per-axis computation
        let ramp = velocities.map(|(v_start, v_end)| {
            let tick_s = Real::from_lit(tick_us as i64, 6);
            (v_start.max(math::ZERO) * tick_s, v_end.max(math::ZERO) * tick_s, RampWindow::new(n_ticks))
        });
        step_increment.foreach(|coord, steps| {
            let coord: CoordSel = coord.into();
            let n = match steps {
                Some(n) if *n > 0 => *n,
                _ => {
                    mt.set_channel_dda(coord, None);
                    return;
                }
            };
            let dda = match &ramp {
                None => uniform_dda(n, numerator, tick_us),
                Some((v_start_tick, v_end_tick, window)) => {
                    let scale = v_dir_abs.get_coord(coord).unwrap_or(math::ZERO)
                        * spm.get_coord(coord).unwrap_or(math::ZERO);
                    window.dda(
                        p0.get_coord(coord).unwrap_or(math::ZERO),
                        step_pos.get_coord(coord).unwrap_or(math::ZERO),
                        advanced_before.get_coord(coord).unwrap_or(math::ZERO),
                        *v_start_tick * scale,
                        *v_end_tick * scale,
                    )
                }
            };
            mt.set_channel_dda(coord, Some(dda));
        });

        let increment = step_increment;
        self.u_steps_advanced += increment;
        self.multi_timer = mt;
        Ok(can_advance_more)
    }

    /// Returns the internal state of the `MultiTimer`.
    ///
    /// # Returns
    ///
    /// A reference to the `MultiTimer`.
    pub fn state(&self) -> &MultiTimer {
        &self.multi_timer
    }

    /// Returns the number of advanced steps.
    ///
    /// # Returns
    ///
    /// A `TVector` containing the number of advanced steps.
    pub fn advanced_steps(&self) -> TVector<u32> {
        self.u_steps_advanced
    }

    /// Returns the number of advanced [hwa::HwiContract::SPACE_UNIT_MAGNITUDE] units.
    ///
    /// # Returns
    ///
    /// A `TVector` containing the number of advanced [hwa::HwiContract::SPACE_UNIT_MAGNITUDE] units.
    pub fn advanced_units(&self) -> TVector<Real> {
        self.advanced_steps().map_values(|_, c| {
            if c > 0 {
                Some(Real::from_lit(c.into(), 0))
            } else {
                None
            }
        }) / self.u_steps_per_space_magnitude
    }

    /// Returns the width of the multi timer.
    ///
    /// # Returns
    ///
    /// The width of the multi timer as a `u32`.
    #[inline]
    pub fn width(&self) -> u32 {
        self.multi_timer.width()
    }

    /// What the motor of an axis is ahead of the ideal segment at path position `s` (the position
    /// of the last window advanced), in steps and in the direction of motion of the axis: within
    /// ±½ step. `None` when the axis does not move. The start offset of the next segment, negated
    /// (see [Self::new_with_start_offset]), for an axis whose position is not kept by the motion
    /// status (the extruder)
    #[cfg(feature = "with-e-axis")]
    pub fn deviation_steps(&self, coord: CoordSel, s: Real) -> Option<Real> {
        let advanced = self.axis_steps_advanced_precise.get_coord(coord)?;
        let ideal = self.start_offset.get_coord(coord)?
            + self.v_dir_abs.get_coord(coord)? * self.u_steps_per_space_magnitude.get_coord(coord)? * s;
        Some(advanced - ideal)
    }
}

/// `2^32` (`1.0` in Q0.32)
#[inline]
fn q32() -> Real {
    Real::from_f32(4294967296.0)
}

/// `2^-32`
#[inline]
fn q32_inv() -> Real {
    Real::from_f32(1.0 / 4294967296.0)
}

#[inline]
fn to_q32_u(x: Real) -> u32 {
    (x * q32())
        .round()
        .to_i64()
        .map(|v| v.clamp(0, u32::MAX as i64) as u32)
        .unwrap_or(0)
}

#[inline]
fn to_q32_i(x: Real) -> i32 {
    (x * q32())
        .round()
        .to_i64()
        .map(|v| v.clamp(i32::MIN as i64, i32::MAX as i64) as i32)
        .unwrap_or(0)
}

/// `n` steps uniformly spread over `width_us` (first one at half a step period).
fn uniform_dda(n: u32, width_us: u32, tick_us: u32) -> (u32, u32, i32) {
    let r = (((n as u64) * (tick_us as u64)) << 32) / (width_us.max(1) as u64);
    (
        crate::hwa::controllers::motion_control::DDA_HALF,
        r.min(u32::MAX as u64) as u32,
        0,
    )
}

/// Per-window constants of the second-order DDA (computed once per window, shared by all axes).
pub(crate) struct RampWindow {
    /// Number of ticks `N`
    n: Real,
    /// `1/N`
    inv_n: Real,
    /// `N·(N−1)/2`
    half_nn1: Real,
    /// `N − 1`
    n_1: Real,
}

impl RampWindow {
    pub(crate) fn new(n_ticks: u32) -> Self {
        let n = Real::from_lit(n_ticks.max(1) as i64, 0);
        let n_1 = n - math::ONE;
        Self {
            n,
            inv_n: n.recip(),
            half_nn1: n * n_1 * math::HALF,
            n_1,
        }
    }

    /// Second-order DDA parameters `(phase, rate, rate_inc)` (Q0.32) for one axis.
    ///
    /// * `p0`, `p1`: continuous position (steps) at the window start / end.
    /// * `advanced_before`: integer steps already emitted before the window
    ///   (a step is due each time the position crosses `k + ½`).
    /// * `r0`, `r1`: profile step rate (steps per tick) at the window boundaries.
    ///
    /// The rate ramps linearly with slope `(r1 − r0)/N`, and its start value is chosen so that the
    /// accumulated phase over the window equals `p1 − p0` exactly: the number of carries is then
    /// the number of half-step crossings, i.e. the step count of the window, and the steps happen
    /// when the (quadratically interpolated) position crosses them. If the ramp would need a
    /// negative rate (not physical) a uniform rate is used instead.
    pub(crate) fn dda(&self, p0: Real, p1: Real, advanced_before: Real, r0: Real, r1: Real) -> (u32, u32, i32) {
        let d = (p1 - p0).max(math::ZERO);
        let mut inc = (r1 - r0) * self.inv_n;
        let rate1 = (d - inc * self.half_nn1) * self.inv_n;
        if rate1 < math::ZERO || rate1 + inc * self.n_1 < math::ZERO {
            inc = math::ZERO;
        }
        let inc_q = to_q32_i(inc);
        // Start rate re-derived from the quantised increment so that Σ rate = d
        let inc_r = Real::from_lit(inc_q as i64, 0) * q32_inv();
        let rate_q = to_q32_u(((d - inc_r * self.half_nn1) * self.inv_n).max(math::ZERO));

        let frac0 = (p0 + math::HALF - advanced_before).max(math::ZERO);
        let phase_q = if frac0 >= math::ONE { u32::MAX } else { to_q32_u(frac0) };
        let _ = self.n;
        (phase_q, rate_q, inc_q)
    }
}

/// A stepper channel that follows a continuous target (steps) which may reverse, unlike the axes
/// of [MicroSegmentInterpolator] (one direction per segment): the extruder with pressure advance,
/// the shaped axes with input shaping.
///
/// At the end of every window the motor is at the step closest to the target (half-step rule), so
/// it never drifts from it. The direction is decided per window (the net steps of the window) and
/// the step rate ramps between the target rates at the window ends (second-order DDA, see
/// [RampWindow::dda]); a window whose target reverses inside it moves at a uniform rate. Its step
/// count is exact either way (the step executor never drops steps).
#[cfg(any(feature = "with-pressure-advance", feature = "with-input-shaping"))]
#[derive(Clone, Copy)]
pub(crate) struct SignedChannel {
    /// Motor position (steps)
    pub motor: i32,
    /// Target (continuous, steps) at the end of the last window
    pub target: Real,
    /// Target step rate (steps per tick) at the end of the last window
    pub rate: Real,
    /// Direction of the last window that moved the motor
    pub forward: bool,
}

#[cfg(any(feature = "with-pressure-advance", feature = "with-input-shaping"))]
impl SignedChannel {
    /// A channel at rest at `motor`, its target there
    pub fn new(motor: i32, forward: bool) -> Self {
        Self {
            motor,
            target: Real::from_lit(motor as i64, 0),
            rate: math::ZERO,
            forward,
        }
    }

    /// Moves the motor to the step closest to `target` along the window ending now (`rate`: the
    /// target rate at its end, steps per tick), setting the channel `coord` of `multi_timer`.
    /// Returns the steps of the window (signed). `ramps` provides the DDA constants of the window
    /// (only computed when the motor moves). A non-representable target (never expected) moves
    /// nothing.
    pub fn advance(
        &mut self,
        multi_timer: &mut MultiTimer,
        coord: CoordSel,
        target: Real,
        rate: Real,
        ramps: &mut RampWindowCache,
    ) -> i32 {
        let motor = (target + math::HALF).floor().to_i32().unwrap_or(self.motor);
        let n = motor - self.motor;
        if n != 0 {
            self.forward = n > 0;
            // In the direction of the motion: the DDA only counts forward
            let m = Real::from_lit(self.motor as i64, 0);
            let (q0, q1, r0, r1) = match self.forward {
                true => (self.target - m, target - m, self.rate, rate),
                false => (m - self.target, m - target, -self.rate, -rate),
            };
            let dda = ramps.window().dda(q0, q1, math::ZERO, r0.max(math::ZERO), r1.max(math::ZERO));
            multi_timer.set_channel_dda(coord, Some(dda));
        } else {
            multi_timer.set_channel_dda(coord, None);
        }
        multi_timer.set_count(coord, n.unsigned_abs());
        self.motor = motor;
        self.target = target;
        self.rate = rate;
        n
    }
}

/// The [RampWindow] of the current window, computed once per window width: all the windows of a
/// segment but its last one have the same
#[cfg(any(feature = "with-pressure-advance", feature = "with-input-shaping"))]
pub(crate) struct RampWindowCache {
    tick_us: u32,
    width_ticks: u32,
    ticks: u32,
    window: RampWindow,
}

#[cfg(any(feature = "with-pressure-advance", feature = "with-input-shaping"))]
impl RampWindowCache {
    pub fn new(tick_us: u32) -> Self {
        Self {
            tick_us: tick_us.max(1),
            width_ticks: 1,
            ticks: 1,
            window: RampWindow::new(1),
        }
    }

    /// Sets the width of the window being generated (µs)
    #[inline]
    pub fn set_width(&mut self, width_us: u32) {
        self.width_ticks = width_us.max(1).div_ceil(self.tick_us);
    }

    fn window(&mut self) -> &RampWindow {
        if self.ticks != self.width_ticks {
            self.ticks = self.width_ticks;
            self.window = RampWindow::new(self.ticks);
        }
        &self.window
    }
}

#[cfg(test)]
mod motion_interpolation_test {
    use crate::hwa;
    use crate::motion::MicroSegmentInterpolator;

    #[test]
    fn interpolation_test_1() {
        use super::MicroSegmentInterpolator;
        use crate::hwa;

        let mut lin = MicroSegmentInterpolator::new(
            hwa::make_vector_real!(x = 1.0),
            hwa::make_real!(100.0),
            hwa::make_vector_real!(x = 160.0),
        );

        assert_eq!(lin.width(), 0);
        assert_eq!(lin.state().width(), 0);

        lin.advance_to(hwa::make_real!(100.0), hwa::make_real!(100.0))
            .expect("bad advance");
        assert_eq!(lin.advanced_units(), hwa::make_vector_real!(x = 100.0))
    }

    #[test]
    fn bad_advance_test() {
        let mut lin = MicroSegmentInterpolator::new(
            hwa::make_vector_real!(x = 1.0),
            hwa::make_real!(100.0),
            hwa::make_vector_real!(x = 160.0),
        );
        assert_eq!(
            lin.advance_to(hwa::make_real!(100.0), hwa::make_real!(0.0)),
            Err(())
        );
    }

    /// When the window is shorter than the per-step period, integer division yields
    /// tick_period = 0 for the channel (width = 0 in the MultiTimer).
    /// advance_to must still return Ok and report the correct advanced-steps count.
    /// The StepPlan executor is responsible for clamping to max feedrate.
    #[test]
    fn test_short_window_many_steps_returns_ok() {
        // 10 units × 10 steps/unit = 100 steps, but window is only 5µs.
        // tick_period = floor(5 / 100) = 0  → channel width = 0 (executor will clamp).
        let mut lin = MicroSegmentInterpolator::new(
            hwa::make_vector_real!(x = 1.0),
            hwa::make_real!(10.0),
            hwa::make_vector_real!(x = 10.0),
        );
        let result = lin.advance_to(hwa::make_real!(10.0), hwa::make_real!(5.0));
        assert!(result.is_ok(), "advance_to with short window must not return Err");
        // Width must reflect the requested window, not zero.
        assert_eq!(lin.width(), 5, "multi_timer width must match the requested window");
        // All 100 steps must be registered for dispatch.
        assert_eq!(
            lin.advanced_steps().get_coord(hwa::CoordSel::X).unwrap_or(0),
            100,
            "all 100 steps must be registered even when tick_period rounds to zero"
        );
    }

    /// Variant: window exactly equals one step period (tick_period = window / steps = 1).
    /// This is on the boundary; advance_to must succeed and register all steps.
    #[test]
    fn test_window_equals_one_step_period() {
        // 1 unit × 10 steps/unit = 10 steps, window = 10µs → tick_period = 1µs per step.
        let mut lin = MicroSegmentInterpolator::new(
            hwa::make_vector_real!(x = 1.0),
            hwa::make_real!(1.0),
            hwa::make_vector_real!(x = 10.0),
        );
        let result = lin.advance_to(hwa::make_real!(1.0), hwa::make_real!(10.0));
        assert!(result.is_ok(), "advance_to at boundary tick_period must succeed");
        assert_eq!(lin.width(), 10);
        assert_eq!(
            lin.advanced_steps().get_coord(hwa::CoordSel::X).unwrap_or(0),
            10
        );
    }

    /// Feedrate exceeded end-to-end: advance_to produces a MultiTimer with width=0,
    /// then StepPlan::next clamps to step_width and fires all max_count steps.
    #[cfg(feature = "with-x-axis")]
    #[test]
    fn test_feedrate_exceeded_end_to_end() {
        use crate::hwa::controllers::StepPlan;
        use hwa::math::CoordSel;

        // 5 steps in a 2µs window → tick_period = 0.
        let mut lin = MicroSegmentInterpolator::new(
            hwa::make_vector_real!(x = 1.0),
            hwa::make_real!(5.0),
            hwa::make_vector_real!(x = 1.0),
        );
        lin.advance_to(hwa::make_real!(5.0), hwa::make_real!(2.0))
            .expect("advance_to must succeed");

        // Build a StepPlan from the resulting MultiTimer state.
        // The channel width will be 0; StepPlan::next must clamp it to step_width.
        let mt = *lin.state();
        let mut plan = StepPlan::from(
            0,
            #[cfg(feature = "with-motion-broadcast")]
            hwa::MotionDelta::new(),
            mt,
            CoordSel::X,
            CoordSel::X,
        );

        // Run for enough ticks to exhaust max_count (5 steps × 10µs = 50µs minimum).
        let step_width = 10u32;
        let mut fired = 0u32;
        for _ in 0..10 {
            if let Some(ch) = plan.next(step_width) {
                if ch.contains(CoordSel::X) {
                    fired += 1;
                }
            }
        }
        assert_eq!(fired, 5, "all 5 steps must fire at clamped max feedrate");
    }

    /// The extruder carried from segment to segment as the stepper task does: the rounding residue
    /// a segment leaves (see `deviation_steps`) is the start offset of the next one, so segments of
    /// a fraction of a step add up to the steps of their total, in both directions. Each one
    /// rounded alone (the former behaviour) they emitted none (0.44 steps) or a whole step (0.52)
    #[cfg(feature = "with-e-axis")]
    #[test]
    fn test_extruder_residue_is_carried_across_segments() {
        use hwa::math::{CoordSel, Real};
        let spm = 400.0f64;
        let segments = core::iter::repeat_n(0.0011f64, 50).chain(core::iter::repeat_n(-0.0013f64, 30));
        let (mut residue, mut motor, mut nominal, mut alone) = (0.0f64, 0i64, 0.0f64, 0i64);
        for e in segments {
            let forward = e > 0.0;
            let offset = if forward { -residue } else { residue };
            let mut lin = super::MicroSegmentInterpolator::new_with_start_offset(
                hwa::make_vector_real!(e = 1.0),
                Real::from_f32(e.abs() as f32),
                hwa::make_vector_real!(e = 400.0),
                hwa::math::TVector::new_with_coord(CoordSel::E, Some(Real::from_f32(offset as f32))),
            );
            // In two windows
            for s in [e.abs() / 2.0, e.abs()] {
                lin.advance_to(Real::from_f32(s as f32), Real::from_f32(2500.0)).unwrap();
            }
            let n = lin.advanced_steps().get_coord(CoordSel::E).unwrap_or(0) as i64;
            motor += if forward { n } else { -n };
            nominal += e * spm;
            alone += if forward { 1 } else { -1 } * (e.abs() * spm).round() as i64;
            let ahead = lin.deviation_steps(CoordSel::E, Real::from_f32(e.abs() as f32)).unwrap().to_f64();
            residue = if forward { ahead } else { -ahead };
            assert!(residue.abs() <= 0.5 + 1e-4, "residue {}", residue);
            assert!((motor as f64 - nominal - residue).abs() < 1e-3, "the residue is where the motor is");
        }
        assert_eq!(motor, nominal.round() as i64, "nominal {} steps", nominal);
        assert_ne!(alone, motor, "each segment rounded alone drifts");
    }

    /// Runs a constant-acceleration motion through the interpolator and the real `StepPlan`
    /// (tick by tick) and returns the max deviation (µs) of the step instants from the ideal
    /// ones, plus the number of emitted steps and the expected one.
    #[cfg(feature = "with-x-axis")]
    fn simulate_constant_accel(ramp: bool) -> (f64, usize, usize) {
        use crate::hwa::controllers::StepPlan;
        use hwa::math::{CoordSel, Real};
        let (spm, v0, a) = (80.0f64, 20.0f64, 1500.0f64);
        let (w_us, tick_us, windows) = (2500u32, 10u32, 12u32);
        let s = |t: f64| v0 * t + 0.5 * a * t * t;
        let v = |t: f64| v0 + a * t;
        let total_t = (w_us * windows) as f64 * 1e-6;
        let s_end = s(total_t);
        let mut lin = super::MicroSegmentInterpolator::new(
            hwa::make_vector_real!(x = 1.0),
            Real::from_f32(s_end as f32),
            hwa::make_vector_real!(x = 80.0),
        );
        let mut times: Vec<f64> = Vec::new();
        for k in 0..windows {
            let (t0, t1) = ((k * w_us) as f64 * 1e-6, ((k + 1) * w_us) as f64 * 1e-6);
            let pos = Real::from_f32(s(t1) as f32);
            let width = Real::from_f32(w_us as f32);
            let velocities = if ramp {
                Some((Real::from_f32(v(t0) as f32), Real::from_f32(v(t1) as f32)))
            } else {
                None
            };
            lin.advance(pos, width, velocities, tick_us).unwrap();
            let mut plan = StepPlan::from(
                0,
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
                *lin.state(),
                CoordSel::X,
                CoordSel::X,
            );
            let mut ticks = 0u32;
            while ticks * tick_us < w_us || plan.has_pending_steps() {
                if let Some(ch) = plan.next(tick_us) {
                    if ch.contains(CoordSel::X) {
                        times.push((k * w_us + (ticks + 1) * tick_us) as f64);
                    }
                }
                ticks += 1;
                assert!(ticks < 100_000);
            }
        }
        let expected = (s_end * spm + 0.5).floor() as usize;
        let mut max_err = 0.0f64;
        for (i, t_us) in times.iter().enumerate() {
            // Step i+1 is due when the position crosses (i + ½) steps
            let x = (i as f64 + 0.5) / spm;
            let t_ideal = (-v0 + (v0 * v0 + 2.0 * a * x).sqrt()) / a * 1e6;
            max_err = max_err.max((t_us - t_ideal).abs());
        }
        (max_err, times.len(), expected)
    }

    /// Second-order DDA: the steps follow the (accelerating) trajectory within one tick, while
    /// the former uniform distribution inside each window deviates by tens of µs (velocity
    /// staircase). No step is lost or added in either case.
    #[cfg(feature = "with-x-axis")]
    #[test]
    fn test_second_order_dda_tracks_acceleration() {
        let (err_ramp, n_ramp, expected) = simulate_constant_accel(true);
        let (err_uniform, n_uniform, _) = simulate_constant_accel(false);
        assert_eq!(n_ramp, expected, "ramp: every step must be emitted exactly once");
        assert_eq!(n_uniform, expected, "uniform: every step must be emitted exactly once");
        assert!(err_ramp <= 12.0, "ramp DDA timing error {} µs (tick = 10 µs)", err_ramp);
        assert!(err_uniform > 2.0 * err_ramp, "uniform {} µs vs ramp {} µs", err_uniform, err_ramp);
    }
}
