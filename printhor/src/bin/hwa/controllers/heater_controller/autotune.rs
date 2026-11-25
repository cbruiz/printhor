//! PID autotune (`M303`) by relay feedback.
//!
//! The procedure is Åström–Hägglund's relay experiment, the one Marlin uses, made more reliable:
//!
//! 1. **Relay oscillation**. The heater is driven as a relay around the target: `bias + d` while
//!    the temperature is below it, `bias - d` above it (both start at half power: the first
//!    heating is at full power). The relay switches once the temperature crosses the target, and
//!    not before [MIN_HALF_CYCLE_S] in the same state: the noise of the readings (or a glitch) can
//!    not toggle it. No hysteresis: it would move the oscillation away from the ultimate point
//!    (a bed oscillates by a few tenths of a degree). The closed loop oscillates at (close to) the
//!    ultimate frequency of the heater.
//! 2. **Bias balancing** (as Marlin). After each cycle the bias moves towards the power that
//!    holds the target (`bias += d·(t_high - t_low)/(t_high + t_low)`), so that the oscillation
//!    becomes symmetric (its heating and cooling halves last the same), and the amplitude `d` is
//!    the largest one the bias allows.
//! 3. **Estimation**. Each cycle from the third one gives an estimate of the ultimate gain and
//!    period (describing function of the relay):
//!    - `Tu` is the cycle period.
//!    - `Ku = |U₁| / |T₁|`: the amplitudes of the fundamentals of the relay output and of the
//!      temperature in the cycle, from their Fourier projections (accumulated reading by reading
//!      at the frequency of the previous cycle: no readings are kept). The temperature is not a
//!      sinusoid (its extremes are not the amplitude of its fundamental) and its noise averages
//!      out. Marlin takes `4·d/(π·a)`, `a` the half amplitude between the extremes, with the `d`
//!      already updated for the next cycle.
//!
//!    On simulated heaters (block, thermistor lag, dead time, noise and quantization of the
//!    readings; see the tests) the ultimate point is found within 15%: Ku a bit lower and Tu a
//!    bit longer (the relay acts at the readings, every control period), i.e. gentler constants.
//! 4. **Convergence**. It finishes after at least `C` cycles (as Marlin's `C`, at least 3) once
//!    the last three estimates agree (within [TOLERANCE] of their median), or [EXTRA_CYCLES]
//!    later anyway (reported). The result is the median of the last [ESTIMATES] estimates: one
//!    disturbed cycle can not spoil it (Marlin takes the last cycle).
//! 5. **Tuning rule** (as Marlin, so that the constants are comparable): the classic
//!    Ziegler–Nichols rule for the hot-end (`Kp = 0.6·Ku, Ti = Tu/2, Td = Tu/8`) and its "no
//!    overshoot" variant for the bed (`Kp = 0.2·Ku, Ti = Tu/2, Td = Tu/3`), in the units of
//!    `M301`/`M304` (output in [0, 255], seconds).
//!
//! Safeguards (as Marlin): it fails when the temperature exceeds the target by
//! [MAX_OVERSHOOT_C], or when a half cycle (the first heating included) lasts more than
//! [MAX_HALF_CYCLE_S]. The heater protections (MAXTEMP, MINTEMP, heating watch, sensor) still
//! apply: only the thermal runaway is not watched while oscillating on purpose.
//!
//! It runs in the temperature control loop (one step per control period): it never blocks.
//! The arithmetic is the one selected for the firmware ([Real]).
use crate::hwa;
use hwa::math::Real;

/// Max output of the controller (Marlin's `PID_MAX`)
const OUT_MAX: f32 = super::PID_MAX;

/// Min duration (s) of a relay half cycle: a glitch of the readings can not toggle the relay
const MIN_HALF_CYCLE_S: f32 = 2.0;

/// Min power of the relay below and above the bias (Marlin: 20 of 255)
const BIAS_MARGIN: f32 = 20.0;

/// The autotune fails when the temperature exceeds the target by this margin (Marlin's
/// `MAX_OVERSHOOT_PID_AUTOTUNE`)
pub const MAX_OVERSHOOT_C: f32 = 30.0;

/// The autotune fails when a half cycle (the first heating included) lasts longer (s) (Marlin's
/// `MAX_CYCLE_TIME_PID_AUTOTUNE`, 20 min)
pub const MAX_HALF_CYCLE_S: u64 = 20 * 60;

/// Estimates kept (the last ones): the result is their median
const ESTIMATES: usize = 5;

/// The last three estimates must agree within this fraction of their median to finish
const TOLERANCE: f32 = 0.15;

/// Cycles run beyond `C` waiting for the estimates to agree
const EXTRA_CYCLES: u8 = 5;

/// Readings averaged to find the extremes of the oscillation
const SMOOTHING: usize = 3;

/// How the PID constants are computed from the ultimate gain and period (Marlin's rules)
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum TuningRule {
    /// Classic Ziegler–Nichols (Marlin's for the hot-ends)
    ClassicPid,
    /// Ziegler–Nichols "no overshoot" (Marlin's for the bed and chamber)
    NoOvershoot,
}

impl TuningRule {
    /// The line Marlin reports for the rule
    pub const fn report(&self) -> &'static str {
        match self {
            TuningRule::ClassicPid => " Classic PID",
            TuningRule::NoOvershoot => " No overshoot",
        }
    }

    /// `(Kp, Ki, Kd)` from the ultimate gain and period (s)
    pub fn gains(&self, ku: Real, tu: Real) -> (Real, Real, Real) {
        let (p_factor, d_factor) = match self {
            TuningRule::ClassicPid => (Real::from_f32(0.6), Real::from_f32(0.125)),
            TuningRule::NoOvershoot => (Real::from_f32(0.2), hwa::math::THIRD),
        };
        let kp = ku * p_factor;
        (kp, hwa::math::TWO * kp / tu, kp * tu * d_factor)
    }
}

/// The ultimate gain and period estimated in a cycle, and the PID constants they give
#[derive(Clone, Copy, Debug)]
pub struct AutotuneEstimate {
    /// Ultimate gain (output units per ºC)
    pub ku: Real,
    /// Ultimate period (s)
    pub tu: Real,
    pub kp: Real,
    pub ki: Real,
    pub kd: Real,
}

/// A relay cycle completed, to be reported as Marlin does
#[derive(Clone, Copy, Debug)]
pub struct AutotuneCycle {
    /// The relay bias for the next cycle [0, 255]
    pub bias: Real,
    /// The relay amplitude for the next cycle [0, 255]
    pub d: Real,
    /// The extremes of the temperature in the cycle (ºC)
    pub min: Real,
    pub max: Real,
    /// From the third cycle
    pub estimate: Option<AutotuneEstimate>,
    /// The rule of the constants of the estimate
    pub rule: TuningRule,
}

/// Outcome of a finished autotune
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AutotuneOutcome {
    /// The PID constants. `settled`: the last estimates agreed (otherwise it stopped after the
    /// extra cycles: the result may be inaccurate)
    Done {
        kp: Real,
        ki: Real,
        kd: Real,
        settled: bool,
    },
    /// The temperature exceeded the target by [MAX_OVERSHOOT_C]
    Overshoot,
    /// A half cycle lasted longer than [MAX_HALF_CYCLE_S]
    Timeout,
    /// No estimate could be taken: the temperature does not oscillate across the target
    NoOscillation,
    /// Cancelled: a new target, the heater shut down (or faulted)
    Interrupted,
}

/// The relay autotune state (see the module documentation)
pub struct Autotune {
    target: Real,
    rule: TuningRule,
    min_cycles: u8,
    /// Relay cycles completed (Marlin's count: each switch to heating)
    cycles: u8,
    heating: bool,
    bias: Real,
    d: Real,
    /// When the relay switched last
    t_switch: embassy_time::Instant,
    /// Duration of the last heating and cooling halves (s)
    t_high: Real,
    t_low: Real,
    readings: [Real; SMOOTHING],
    n_readings: u8,
    /// The fundamentals of the cycle: projections of the relay output and of the temperature
    /// (both as deviations: from the bias, and from the target) on `cos`, `sin` of `omega·t`
    fundamentals: [Real; 4],
    /// Frequency of the projections (rad/s): that of the previous cycle (zero until known)
    omega: Real,
    /// When the running cycle started (the relay went on)
    cycle_start: embassy_time::Instant,
    /// The last reading: when, the output applied since then (deviation), the temperature
    /// (deviation)
    last: Option<(embassy_time::Instant, Real, Real)>,
    /// Max of the (averaged) temperature since the relay went off, min since it went on
    peak: Real,
    trough: Real,
    ku: [Real; ESTIMATES],
    tu: [Real; ESTIMATES],
    /// Estimates taken (the last [ESTIMATES] are kept)
    n_estimates: u8,
    /// The last cycle completed, until taken (see [Self::take_cycle])
    cycle: Option<AutotuneCycle>,
}

impl Autotune {
    /// An autotune to `target` (ºC), of at least `min_cycles` cycles (3 at least)
    pub fn new(target: f32, min_cycles: u8, rule: TuningRule, now: embassy_time::Instant) -> Self {
        let half = Real::from_f32(OUT_MAX / 2.0);
        Self {
            target: Real::from_f32(target),
            rule,
            min_cycles: min_cycles.max(3),
            cycles: 0,
            heating: true,
            bias: half,
            d: half,
            t_switch: now,
            t_high: hwa::math::ZERO,
            t_low: hwa::math::ZERO,
            readings: [hwa::math::ZERO; SMOOTHING],
            n_readings: 0,
            fundamentals: [hwa::math::ZERO; 4],
            omega: hwa::math::ZERO,
            cycle_start: now,
            last: None,
            peak: hwa::math::ZERO,
            trough: hwa::math::ZERO,
            ku: [hwa::math::ZERO; ESTIMATES],
            tu: [hwa::math::ZERO; ESTIMATES],
            n_estimates: 0,
            cycle: None,
        }
    }

    /// Takes the report of the last cycle completed, if any
    pub fn take_cycle(&mut self) -> Option<AutotuneCycle> {
        self.cycle.take()
    }

    /// Advances the autotune with a new reading. Returns the output to apply [0, 255], or the
    /// outcome when it finished
    pub fn step(&mut self, temp: f32, now: embassy_time::Instant) -> Result<f32, AutotuneOutcome> {
        let temp = Real::from_f32(temp);
        if temp > self.target + Real::from_f32(MAX_OVERSHOOT_C) {
            return Err(AutotuneOutcome::Overshoot);
        }
        let elapsed = now.saturating_duration_since(self.t_switch);
        if elapsed.as_secs() > MAX_HALF_CYCLE_S {
            return Err(AutotuneOutcome::Timeout);
        }
        let elapsed = Real::from_lit(elapsed.as_millis() as i64, 3);
        self.project(now);
        let smoothed = self.smooth(temp);
        if self.heating {
            self.trough = self.trough.min(smoothed);
        } else {
            self.peak = self.peak.max(smoothed);
        }
        let debounced = elapsed >= Real::from_f32(MIN_HALF_CYCLE_S);
        // The relay acts at the readings: on average half a period after the temperature crossed
        // the target. That latency lowers the frequency of the oscillation a bit (on a hot-end,
        // Tu ~8% longer and Ku ~10% lower than the ultimate point of the sampled loop): a
        // conservative bias (gentler constants). Deciding on the temperature extrapolated ahead
        // removes it, but the noise of the readings then switches the relay early (worse on a
        // bed, whose oscillation is a few tenths of a degree)
        if self.heating && debounced && temp > self.target {
            self.heating = false;
            self.t_high = elapsed;
            self.t_switch = now;
            self.peak = smoothed;
        } else if !self.heating && debounced && temp < self.target {
            self.heating = true;
            self.t_low = elapsed;
            self.t_switch = now;
            // The first heating (from the start) is not a cycle
            if self.cycles > 0 {
                self.complete_cycle();
            }
            self.cycles = self.cycles.saturating_add(1);
            self.trough = smoothed;
            // The next cycle is projected at the frequency of this one
            self.fundamentals = [hwa::math::ZERO; 4];
            self.cycle_start = now;
            self.omega = match (self.t_high + self.t_low).is_defined_positive() {
                true => hwa::math::TWO * hwa::math::PI / (self.t_high + self.t_low),
                false => hwa::math::ZERO,
            };
            if let Some(outcome) = self.outcome() {
                return Err(outcome);
            }
        }
        let deviation = match self.heating {
            true => self.d,
            false => -self.d,
        };
        self.last = Some((now, deviation, temp - self.target));
        Ok(((self.bias + deviation).to_f64() as f32).clamp(0.0, OUT_MAX))
    }

    /// Adds the interval since the last reading to the projections of the running cycle: the
    /// output applied over it (exact integral of a constant) and the last reading (rectangle)
    fn project(&mut self, now: embassy_time::Instant) {
        let (Some((t_prev, output, temp)), true) = (self.last, self.omega.is_defined_positive()) else {
            return;
        };
        let start = self.cycle_start;
        let seconds = |t: embassy_time::Instant| {
            Real::from_lit(t.saturating_duration_since(start).as_millis() as i64, 3)
        };
        let (a, b) = (self.omega * seconds(t_prev), self.omega * seconds(now));
        let (sin_a, cos_a, sin_b, cos_b) = (a.sin(), a.cos(), b.sin(), b.cos());
        let omega = self.omega;
        let [uc, us, yc, ys] = &mut self.fundamentals;
        // ∫ output·e^(-jωt) dt over [t_prev, now]
        *uc += output * (sin_b - sin_a) / omega;
        *us += output * (cos_a - cos_b) / omega;
        // temp·e^(-jωt_prev)·Δt
        let dt = (b - a) / omega;
        *yc += temp * cos_a * dt;
        *ys += temp * sin_a * dt;
    }

    /// The average of the last readings, `temp` included
    fn smooth(&mut self, temp: Real) -> Real {
        let n = self.n_readings as usize;
        self.readings.rotate_right(1);
        if let Some(first) = self.readings.first_mut() {
            *first = temp;
        }
        if n == 0 {
            // The extremes start at the first reading
            self.peak = temp;
            self.trough = temp;
        }
        let n = (n + 1).min(SMOOTHING);
        self.n_readings = n as u8;
        let sum = self
            .readings
            .iter()
            .take(n)
            .fold(hwa::math::ZERO, |acc, v| acc + *v);
        sum / Real::from_lit(n as i64, 0)
    }

    /// A relay cycle ended (the relay switched to heating): the bias is balanced and, from the
    /// third cycle, the ultimate gain and period are estimated
    fn complete_cycle(&mut self) {
        let period = self.t_high + self.t_low;
        if !period.is_defined_positive() {
            return;
        }
        let d = self.d;
        let out_max = Real::from_f32(OUT_MAX);
        let margin = Real::from_f32(BIAS_MARGIN);
        self.bias = (self.bias + d * (self.t_high - self.t_low) / period)
            .max(margin)
            .min(out_max - margin);
        self.d = self.bias.min(out_max - self.bias);

        let estimate = match self.cycles > 2 && self.omega.is_defined_positive() {
            true => {
                // The ratio of the fundamentals (their common factor 2/T cancels)
                let [uc, us, yc, ys] = self.fundamentals;
                let output = (uc * uc + us * us).sqrt();
                let temp = (yc * yc + ys * ys).sqrt().filter(|t| t.is_defined_positive());
                output
                    .zip(temp)
                    .map(|(output, temp)| output / temp)
                    .filter(|ku| ku.is_defined_positive())
                    .map(|ku| {
                        self.push_estimate(ku, period);
                        let (kp, ki, kd) = self.rule.gains(ku, period);
                        AutotuneEstimate {
                            ku,
                            tu: period,
                            kp,
                            ki,
                            kd,
                        }
                    })
            }
            false => None,
        };
        self.cycle = Some(AutotuneCycle {
            bias: self.bias,
            d: self.d,
            min: self.trough,
            max: self.peak,
            estimate,
            rule: self.rule,
        });
    }

    fn push_estimate(&mut self, ku: Real, tu: Real) {
        let slot = self.n_estimates as usize % ESTIMATES;
        if let (Some(k), Some(t)) = (self.ku.get_mut(slot), self.tu.get_mut(slot)) {
            *k = ku;
            *t = tu;
        }
        self.n_estimates = self.n_estimates.saturating_add(1);
    }

    /// The last `n` estimates (gain, period), the newest first
    fn last_estimates(&self, n: usize) -> impl Iterator<Item = (Real, Real)> + '_ {
        let total = self.n_estimates as usize;
        (0..n.min(total).min(ESTIMATES)).filter_map(move |i| {
            let slot = (total - 1 - i) % ESTIMATES;
            Some((*self.ku.get(slot)?, *self.tu.get(slot)?))
        })
    }

    /// The outcome once finished: at least `C` cycles, and the last estimates agree (or
    /// [EXTRA_CYCLES] more)
    fn outcome(&self) -> Option<AutotuneOutcome> {
        if self.cycles <= self.min_cycles {
            return None;
        }
        let settled = self.n_estimates >= 3 && {
            let (ku, tu) = self.medians(3);
            let agree = |median: Real, value: Real| {
                (value - median).abs() <= median * Real::from_f32(TOLERANCE)
            };
            self.last_estimates(3)
                .all(|(k, t)| agree(ku, k) && agree(tu, t))
        };
        if !settled && self.cycles <= self.min_cycles.saturating_add(EXTRA_CYCLES) {
            return None;
        }
        if self.n_estimates == 0 {
            return Some(AutotuneOutcome::NoOscillation);
        }
        let (ku, tu) = self.medians(ESTIMATES);
        let (kp, ki, kd) = self.rule.gains(ku, tu);
        Some(AutotuneOutcome::Done {
            kp,
            ki,
            kd,
            settled,
        })
    }

    /// The medians of the gain and the period of the last `n` estimates
    fn medians(&self, n: usize) -> (Real, Real) {
        let mut ku = [hwa::math::ZERO; ESTIMATES];
        let mut tu = [hwa::math::ZERO; ESTIMATES];
        let mut len = 0;
        for ((k, t), (dk, dt)) in self.last_estimates(n).zip(ku.iter_mut().zip(tu.iter_mut())) {
            *dk = k;
            *dt = t;
            len += 1;
        }
        (median(ku.get_mut(..len).unwrap_or(&mut [])), median(tu.get_mut(..len).unwrap_or(&mut [])))
    }
}

/// The median of a few values (sorted in place). The mean of the central ones when even
fn median(values: &mut [Real]) -> Real {
    // Insertion sort: a handful of values, and no panic paths
    for i in 1..values.len() {
        let mut j = i;
        while j > 0 {
            let (Some(a), Some(b)) = (values.get(j - 1).copied(), values.get(j).copied()) else {
                break;
            };
            if a <= b {
                break;
            }
            values.swap(j - 1, j);
            j -= 1;
        }
    }
    let n = values.len();
    let high = values.get(n / 2).copied().unwrap_or(hwa::math::ZERO);
    match (n % 2 == 0).then(|| (n / 2).checked_sub(1)).flatten() {
        Some(low) => (high + values.get(low).copied().unwrap_or(high)) * hwa::math::HALF,
        None => high,
    }
}

#[cfg(test)]
mod tests;
