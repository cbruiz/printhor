//! The autotune against simulated heaters more realistic than the first order model of the
//! native board: a heater block (first order, losses to the ambient), a thermistor that lags
//! behind it (first order), a dead time, noise and the quantization of the readings. Every
//! result is checked against the ultimate point of the linear model, and the constants found
//! drive the firmware PID ([super::super::PidControl]) on the same heater.
use super::*;
use embassy_time::{Duration, Instant};

/// Control period of the firmware (s)
const DT: f32 = 0.5;

#[derive(Clone, Copy)]
struct Heater {
    ambient: f32,
    /// Heating rate of the block at full power, without losses (ºC/s)
    heating: f32,
    /// Losses (1/s): the block time constant is its inverse
    cooling: f32,
    /// Thermistor time constant (s)
    sensor_lag: f32,
    /// Dead time between the power and the block (s)
    dead_time: f32,
    /// Peak noise of the readings (ºC, uniform)
    noise: f32,
    /// Resolution of the readings (ºC)
    quantum: f32,
}

/// A hot-end: ~6 ºC/s at full power, 50 s block time constant (it would settle at 325 ºC), a
/// thermistor 3 s behind the block, 1 s of dead time
const HOT_END: Heater = Heater {
    ambient: 25.0,
    heating: 6.0,
    cooling: 0.02,
    sensor_lag: 3.0,
    dead_time: 1.0,
    noise: 0.1,
    quantum: 0.1,
};

/// A bed: 0.5 ºC/s at full power, 250 s time constant (it would settle at 150 ºC), a thermistor
/// 15 s behind the plate, 3 s of dead time
const BED: Heater = Heater {
    ambient: 25.0,
    heating: 0.5,
    cooling: 0.004,
    sensor_lag: 15.0,
    dead_time: 3.0,
    noise: 0.05,
    quantum: 0.05,
};

struct Plant {
    model: Heater,
    block: f32,
    sensor: f32,
    /// Power applied in the last periods (dead time)
    history: std::collections::VecDeque<f32>,
    seed: u32,
}

impl Plant {
    fn new(model: Heater) -> Self {
        let delay = (model.dead_time / DT).round() as usize;
        Self {
            model,
            block: model.ambient,
            sensor: model.ambient,
            history: core::iter::repeat(0.0).take(delay).collect(),
            seed: 0x1234_5678,
        }
    }

    /// One control period with the given output [0, 255]
    fn step(&mut self, output: f32) {
        self.history.push_back((output / OUT_MAX).clamp(0.0, 1.0));
        let power = self.history.pop_front().unwrap_or(0.0);
        let m = self.model;
        const SUBSTEPS: usize = 50;
        let h = DT / SUBSTEPS as f32;
        for _ in 0..SUBSTEPS {
            self.block += (m.heating * power - m.cooling * (self.block - m.ambient)) * h;
            self.sensor += (self.block - self.sensor) / m.sensor_lag * h;
        }
    }

    /// The temperature the firmware reads
    fn read(&mut self) -> f32 {
        // xorshift32: deterministic noise
        self.seed ^= self.seed << 13;
        self.seed ^= self.seed >> 17;
        self.seed ^= self.seed << 5;
        let uniform = (self.seed as f32 / u32::MAX as f32) * 2.0 - 1.0;
        let t = self.sensor + uniform * self.model.noise;
        (t / self.model.quantum).round() * self.model.quantum
    }
}

/// The ultimate gain (output units per ºC) and period (s) of the linear model, plus half a period
/// of delay (the zero-order hold of the control period)
fn ultimate_point(m: &Heater) -> (f32, f32) {
    let (tau1, tau2) = (1.0 / m.cooling as f64, m.sensor_lag as f64);
    let delay = m.dead_time as f64 + DT as f64 / 2.0;
    let gain = m.heating as f64 / m.cooling as f64 / OUT_MAX as f64;
    let phase = |w: f64| (w * tau1).atan() + (w * tau2).atan() + w * delay;
    let (mut low, mut high) = (1e-6f64, 10.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if phase(mid) < core::f64::consts::PI {
            low = mid;
        } else {
            high = mid;
        }
    }
    let w = 0.5 * (low + high);
    let magnitude = gain / ((1.0 + (w * tau1).powi(2)).sqrt() * (1.0 + (w * tau2).powi(2)).sqrt());
    ((1.0 / magnitude) as f32, (2.0 * core::f64::consts::PI / w) as f32)
}

struct Run {
    outcome: AutotuneOutcome,
    cycles: std::vec::Vec<AutotuneCycle>,
    seconds: f32,
}

/// Runs an autotune on the heater, from the ambient temperature
fn autotune(model: Heater, target: f32, min_cycles: u8, rule: TuningRule) -> Run {
    let mut plant = Plant::new(model);
    let t0 = Instant::from_ticks(0);
    let mut at = Autotune::new(target, min_cycles, rule, t0);
    let mut cycles = std::vec::Vec::new();
    for k in 0u64.. {
        let now = t0 + Duration::from_millis(k * (DT * 1000.0) as u64);
        let step = at.step(plant.read(), now);
        if let Some(cycle) = at.take_cycle() {
            cycles.push(cycle);
        }
        match step {
            Ok(output) => plant.step(output),
            Err(outcome) => {
                return Run {
                    outcome,
                    cycles,
                    seconds: k as f32 * DT,
                };
            }
        }
        assert!(k < 4 * 3600 * 2, "the autotune does not end");
    }
    unreachable!()
}

fn constants(outcome: AutotuneOutcome) -> (f32, f32, f32, bool) {
    match outcome {
        AutotuneOutcome::Done { kp, ki, kd, settled } => {
            (kp.to_f64() as f32, ki.to_f64() as f32, kd.to_f64() as f32, settled)
        }
        other => panic!("autotune failed: {:?}", other),
    }
}

/// The ultimate gain and period the constants come from (inverse of the rule)
fn ultimate_from(rule: TuningRule, kp: f32, ki: f32) -> (f32, f32) {
    let p_factor = match rule {
        TuningRule::ClassicPid => 0.6,
        TuningRule::NoOvershoot => 0.2,
    };
    (kp / p_factor, 2.0 * kp / ki)
}

struct Response {
    overshoot: f32,
    /// Largest error once settled (the last `tail_s` seconds)
    settled_error: f32,
}

/// The firmware PID with the given constants heats the plant from the ambient temperature to
/// `target` during `seconds`
fn closed_loop(model: Heater, target: f32, (kp, ki, kd): (f32, f32, f32), seconds: f32, tail_s: f32) -> Response {
    let mut plant = Plant::new(model);
    let mut pid = super::super::PidControl::new(kp, ki, kd);
    let steps = (seconds / DT) as usize;
    let tail = (tail_s / DT) as usize;
    let mut response = Response {
        overshoot: f32::MIN,
        settled_error: 0.0,
    };
    for k in 0..steps {
        let temp = plant.read();
        let output = pid.next(target, temp, DT);
        plant.step(output);
        response.overshoot = response.overshoot.max(plant.sensor - target);
        if k + tail >= steps {
            response.settled_error = response.settled_error.max((plant.sensor - target).abs());
        }
    }
    response
}

fn close(value: f32, expected: f32, tolerance: f32) -> bool {
    (value - expected).abs() <= tolerance * expected.abs()
}

/// Marlin's estimate from the last cycle: `Ku = 4·d/(π·a)` (`a` the half amplitude between the
/// extremes, `d` the one for the next cycle), `Tu` the period. For comparison only
fn marlin_estimate(run: &Run) -> Option<(f32, f32)> {
    let last = run.cycles.last()?;
    let f = |v: Real| v.to_f64() as f32;
    let amplitude = (f(last.max) - f(last.min)) / 2.0;
    let tu = f(last.estimate?.tu);
    Some((4.0 * f(last.d) / (core::f32::consts::PI * amplitude), tu))
}

#[test]
fn hot_end_autotune_finds_the_ultimate_point() {
    let run = autotune(HOT_END, 200.0, 5, TuningRule::ClassicPid);
    let (kp, ki, kd, settled) = constants(run.outcome);
    assert!(settled, "the estimates did not agree: {:?}", run.cycles.len());
    let (ku, tu) = ultimate_from(TuningRule::ClassicPid, kp, ki);
    let (ku_model, tu_model) = ultimate_point(&HOT_END);
    std::println!(
        "hot-end: Kp {} Ki {} Kd {} | Ku {} Tu {} | model Ku {} Tu {} | Marlin's {:?} | {} cycles in {} s",
        kp, ki, kd, ku, tu, ku_model, tu_model, marlin_estimate(&run), run.cycles.len(), run.seconds
    );
    assert!(close(ku, ku_model, 0.15), "Ku {} vs {}", ku, ku_model);
    assert!(close(tu, tu_model, 0.15), "Tu {} vs {}", tu, tu_model);
    // As Marlin's C5: 3 estimates (cycles 3 to 5) when they agree at once
    assert_eq!(run.cycles.iter().filter(|c| c.estimate.is_some()).count(), 3);
}

/// The constants found make the firmware PID well damped on the hot-end. Regression: the
/// derivative filter was Marlin's `PID_K1` (0.95) applied every control period (0.5 s instead of
/// Marlin's ~0.13 s): a ~10 s time constant that left the Ziegler–Nichols constants barely stable
/// (6 ºC overshoot, and with constants 5% higher a sustained ±4.5 ºC oscillation). See
/// [super::super::PID_D_FILTER_TIME_S]
#[test]
fn hot_end_pid_from_the_autotune_is_well_damped() {
    let run = autotune(HOT_END, 200.0, 5, TuningRule::ClassicPid);
    let (kp, ki, kd, _) = constants(run.outcome);
    // The constants found, and a bit higher (another run, another heater)
    for scale in [1.0f32, 1.05] {
        let gains = (kp * scale, ki * scale, kd * scale);
        let response = closed_loop(HOT_END, 200.0, gains, 600.0, 120.0);
        std::println!("hot-end PID {:?}: overshoot {} error {}", gains, response.overshoot, response.settled_error);
        assert!(response.overshoot < 4.0, "overshoot {}", response.overshoot);
        assert!(response.settled_error < 0.5, "settled error {}", response.settled_error);
    }
}

#[test]
fn bed_autotune_uses_the_no_overshoot_rule() {
    let run = autotune(BED, 60.0, 5, TuningRule::NoOvershoot);
    let (kp, ki, kd, settled) = constants(run.outcome);
    assert!(settled);
    let (ku, tu) = ultimate_from(TuningRule::NoOvershoot, kp, ki);
    let (ku_model, tu_model) = ultimate_point(&BED);
    std::println!(
        "bed: Kp {} Ki {} Kd {} | Ku {} Tu {} | model Ku {} Tu {} | Marlin's {:?} | {} cycles in {} s",
        kp, ki, kd, ku, tu, ku_model, tu_model, marlin_estimate(&run), run.cycles.len(), run.seconds
    );
    assert!(close(ku, ku_model, 0.2), "Ku {} vs {}", ku, ku_model);
    assert!(close(tu, tu_model, 0.15), "Tu {} vs {}", tu, tu_model);
    assert!(close(kd, kp * tu / 3.0, 1e-3), "the derivative time of the rule is Tu/3");
    let response = closed_loop(BED, 60.0, (kp, ki, kd), 3600.0, 600.0);
    std::println!("bed PID: overshoot {} error {}", response.overshoot, response.settled_error);
    assert!(response.overshoot < 3.0, "overshoot {}", response.overshoot);
    assert!(response.settled_error < 0.5, "settled error {}", response.settled_error);
}

#[test]
fn noisy_readings_do_not_spoil_the_result() {
    let clean = constants(autotune(HOT_END, 200.0, 5, TuningRule::ClassicPid).outcome);
    // Readings 6 times noisier, and coarser
    let noisy_heater = Heater {
        noise: 0.6,
        quantum: 0.25,
        ..HOT_END
    };
    let run = autotune(noisy_heater, 200.0, 5, TuningRule::ClassicPid);
    let noisy = constants(run.outcome);
    std::println!("clean {:?} noisy {:?}", clean, noisy);
    assert!(close(noisy.0, clean.0, 0.15), "Kp {} vs {}", noisy.0, clean.0);
    assert!(close(noisy.1, clean.1, 0.2), "Ki {} vs {}", noisy.1, clean.1);
    assert!(close(noisy.2, clean.2, 0.15), "Kd {} vs {}", noisy.2, clean.2);
}

#[test]
fn the_bias_balances_a_heater_far_from_half_power() {
    // Holding 200 ºC takes ~35% of the power of this one
    let strong = Heater {
        heating: 10.0,
        ..HOT_END
    };
    let run = autotune(strong, 200.0, 5, TuningRule::ClassicPid);
    let (kp, ki, _, settled) = constants(run.outcome);
    assert!(settled);
    let holding = (200.0 - strong.ambient) * strong.cooling / strong.heating * OUT_MAX;
    let last = run.cycles.last().expect("cycles");
    let bias = last.bias.to_f64() as f32;
    assert!((bias - holding).abs() < 0.2 * holding, "bias {} vs {}", bias, holding);
    let (ku, tu) = ultimate_from(TuningRule::ClassicPid, kp, ki);
    let (ku_model, tu_model) = ultimate_point(&strong);
    assert!(close(ku, ku_model, 0.2), "Ku {} vs {}", ku, ku_model);
    assert!(close(tu, tu_model, 0.2), "Tu {} vs {}", tu, tu_model);
}

#[test]
fn every_cycle_is_reported_and_the_cycles_are_at_least_c() {
    for c in [3u8, 5, 8] {
        let run = autotune(HOT_END, 200.0, c, TuningRule::ClassicPid);
        constants(run.outcome);
        // The first heating is not a cycle: the end comes at the switch that completes cycle `C`
        assert!(run.cycles.len() >= c as usize, "{} cycles for C{}", run.cycles.len(), c);
        // Estimates from the third cycle (as Marlin)
        assert!(run.cycles.iter().take(2).all(|c| c.estimate.is_none()));
        assert!(run.cycles.iter().skip(2).all(|c| c.estimate.is_some()));
        for cycle in &run.cycles {
            assert!(cycle.min < Real::from_f32(200.0) && cycle.max > Real::from_f32(200.0));
            assert!(cycle.bias + cycle.d <= Real::from_f32(OUT_MAX));
        }
    }
}

#[test]
fn a_target_far_below_the_temperature_fails_at_once() {
    let t0 = Instant::from_ticks(0);
    let mut at = Autotune::new(150.0, 5, TuningRule::ClassicPid, t0);
    assert_eq!(at.step(150.0 + MAX_OVERSHOOT_C + 1.0, t0), Err(AutotuneOutcome::Overshoot));
}

#[test]
fn a_heater_that_can_not_reach_the_target_times_out() {
    // It would settle at 145 ºC
    let weak = Heater {
        heating: 2.4,
        ..HOT_END
    };
    let run = autotune(weak, 200.0, 5, TuningRule::ClassicPid);
    assert_eq!(run.outcome, AutotuneOutcome::Timeout);
    assert!((run.seconds - MAX_HALF_CYCLE_S as f32).abs() <= 1.0, "{} s", run.seconds);
}

#[test]
fn a_glitch_does_not_toggle_the_relay() {
    let t0 = Instant::from_ticks(0);
    let mut at = Autotune::new(200.0, 5, TuningRule::ClassicPid, t0);
    let at_ms = |ms: u64| t0 + Duration::from_millis(ms);
    // Heating: full power
    assert_eq!(at.step(100.0, at_ms(0)), Ok(OUT_MAX));
    // A single reading above the target right after the start: too early, still heating
    assert_eq!(at.step(225.0, at_ms(500)), Ok(OUT_MAX));
    // At the target: still heating
    assert_eq!(at.step(200.0, at_ms(5000)), Ok(OUT_MAX));
    // Above it: off
    assert_eq!(at.step(200.1, at_ms(5500)), Ok(0.0));
    // And not on again before the min half cycle
    assert_eq!(at.step(199.0, at_ms(6000)), Ok(0.0));
    assert_eq!(at.step(199.0, at_ms(7500)), Ok(OUT_MAX));
}

#[test]
fn marlin_tuning_rules() {
    let (ku, tu) = (Real::from_f32(50.0), Real::from_f32(20.0));
    let f = |(p, i, d): (Real, Real, Real)| (p.to_f64() as f32, i.to_f64() as f32, d.to_f64() as f32);
    let (kp, ki, kd) = f(TuningRule::ClassicPid.gains(ku, tu));
    assert!(close(kp, 30.0, 1e-5) && close(ki, 3.0, 1e-5) && close(kd, 75.0, 1e-5));
    let (kp, ki, kd) = f(TuningRule::NoOvershoot.gains(ku, tu));
    assert!(close(kp, 10.0, 1e-5) && close(ki, 1.0, 1e-5) && close(kd, 200.0 / 3.0, 1e-5));
}

#[test]
fn median_of_a_few_values() {
    let r = |v: &[f32]| v.iter().map(|x| Real::from_f32(*x)).collect::<std::vec::Vec<_>>();
    assert_eq!(median(&mut r(&[3.0, 1.0, 2.0])), Real::from_f32(2.0));
    assert_eq!(median(&mut r(&[4.0, 1.0, 3.0, 2.0])), Real::from_f32(2.5));
    assert_eq!(median(&mut r(&[7.0])), Real::from_f32(7.0));
    assert_eq!(median(&mut []), hwa::math::ZERO);
}

