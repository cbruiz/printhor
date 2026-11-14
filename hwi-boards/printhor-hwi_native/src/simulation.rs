//! Physical simulation of the native board: the machine the firmware is driving.
//!
//! The mocked peripherals do not just store what the firmware writes: they feed this model, and
//! what the firmware reads back (end-stops, thermistors) comes from it. Everything lives in the
//! native board, so the firmware runs exactly the same code and checks it runs on real hardware.
//!
//! - **Head**: the position of every axis is tracked in steps, from the step pulses and the
//!   direction pins. The end-stop of an axis is at step 0: its pin is active at 0 and the axis
//!   cannot go below it (the carriage is against the end-stop, steps are lost). Neither can it go
//!   beyond its travel (the end of the frame), whatever the firmware believes (e.g. after a G92).
//! - **Heaters**: a first order thermal model per heater, driven by the PWM duty the firmware
//!   applies, and read through the thermistor (β equation) as an ADC sample.
//!
//! The initial state is configurable ([configure_head], [configure_travel], [configure_heater]) and [reset] brings
//! the machine back to it, so every test starts from a known, deterministic situation.

// Parts of the model are unused by machine profiles without heaters or end-stops
#![allow(dead_code)]

use std::sync::Mutex;

/// The axes with a simulated end-stop
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
}

/// Number of axes with a simulated end-stop
pub const NUM_AXES: usize = 3;

/// The default initial position of the head: 100 steps away from every end-stop
pub const DEFAULT_HEAD_POSITION_STEPS: [i64; NUM_AXES] = [100; NUM_AXES];

/// The simulated heaters
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Heater {
    HotEnd = 0,
    HotBed = 1,
}

const NUM_HEATERS: usize = 2;

/// The PWM channel that powers each heater (see the board setup)
pub(crate) const HEATER_PWM_CHANNEL: [u8; NUM_HEATERS] = [1, 2];
/// The ADC pin that samples the thermistor of each heater (see the board setup)
pub(crate) const HEATER_ADC_PIN: [u8; NUM_HEATERS] = [23, 24];

/// The thermal behavior of a heater, as a first order model:
///
/// `dT/dt = heating_rate * power - cooling_coefficient * (T - ambient)`
///
/// where `power` is the applied PWM duty (0 to 1). With a constant power the temperature tends
/// exponentially to `ambient + heating_rate * power / cooling_coefficient`, with time constant
/// `1 / cooling_coefficient`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalProfile {
    /// Ambient temperature (ºC): the temperature of the heater when off for long
    pub ambient_c: f32,
    /// Temperature at the beginning (ºC)
    pub initial_c: f32,
    /// Heating rate at full power, without losses (ºC/s)
    pub heating_rate_c_per_s: f32,
    /// Newton cooling coefficient (1/s)
    pub cooling_coefficient: f32,
}

impl ThermalProfile {
    /// A hot-end that heats at full power at 10ºC/s from 25ºC, reaching 200ºC in ~25 seconds
    /// (it would settle at ~358ºC, beyond MAXTEMP, as real cartridges do)
    pub const HOT_END: Self = Self {
        ambient_c: 25.0,
        initial_c: 25.0,
        heating_rate_c_per_s: 10.0,
        cooling_coefficient: 0.03,
    };
    /// A hot-bed that heats at full power from 25ºC to 60ºC in ~13 seconds
    pub const HOT_BED: Self = Self {
        ambient_c: 25.0,
        initial_c: 25.0,
        heating_rate_c_per_s: 3.0,
        cooling_coefficient: 0.02,
    };

    /// The temperature reached after `dt_s` seconds at constant `power`, from `temp`.
    ///
    /// Exact solution of the model, so the result does not depend on how often it is sampled.
    pub fn evolve(&self, temp: f32, power: f32, dt_s: f32) -> f32 {
        if self.cooling_coefficient <= 0.0 {
            return temp + self.heating_rate_c_per_s * power * dt_s;
        }
        let equilibrium =
            self.ambient_c + self.heating_rate_c_per_s * power / self.cooling_coefficient;
        equilibrium + (temp - equilibrium) * (-self.cooling_coefficient * dt_s).exp()
    }
}

/// The thermistor and ADC that read a heater
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Thermistor {
    pub beta: f32,
    pub nominal_resistance: f32,
    pub pull_up_resistance: f32,
}

impl Thermistor {
    /// The 12-bit ADC sample of the thermistor at `temp_c`: the inverse of the conversion done by
    /// the firmware (β equation and voltage divider).
    pub fn sample(&self, temp_c: f32) -> u16 {
        let r = self.nominal_resistance
            * (self.beta * (1.0 / (temp_c + 273.15) - 1.0 / 298.15)).exp();
        let sample = 4095.0 * r / (r + self.pull_up_resistance);
        sample.round().clamp(0.0, 4095.0) as u16
    }
}

#[derive(Clone, Copy)]
struct HeaterState {
    profile: ThermalProfile,
    temp_c: f32,
    power: f32,
    last_update: Option<embassy_time::Instant>,
}

impl HeaterState {
    const fn new(profile: ThermalProfile) -> Self {
        Self {
            profile,
            temp_c: profile.initial_c,
            power: 0.0,
            last_update: None,
        }
    }

    /// Integrates the model up to now with the power applied since the last update
    fn update(&mut self) {
        let now = embassy_time::Instant::now();
        if let Some(last) = self.last_update {
            let dt_s = (now - last).as_micros() as f32 / 1_000_000.0;
            self.temp_c = self.profile.evolve(self.temp_c, self.power, dt_s);
        }
        self.last_update = Some(now);
    }
}

struct Machine {
    /// Initial head position (steps)
    initial_head: [i64; NUM_AXES],
    /// Travel of each axis (steps from the end-stop to the end of the frame).
    /// `None`: the world size of the board contract.
    travel: Option<[i64; NUM_AXES]>,
    /// Position of each motor (steps). For cartesian machines, the position of each axis.
    motors: [i64; NUM_AXES],
    /// Last direction applied to each motor (true: towards positive)
    forward: [bool; NUM_AXES],
    /// Thermal profile of each heater the machine starts with
    profiles: [ThermalProfile; NUM_HEATERS],
    heaters: [HeaterState; NUM_HEATERS],
}

impl Machine {
    const fn new() -> Self {
        Self {
            initial_head: DEFAULT_HEAD_POSITION_STEPS,
            travel: None,
            motors: head_to_motors(DEFAULT_HEAD_POSITION_STEPS),
            forward: [true; NUM_AXES],
            profiles: [ThermalProfile::HOT_END, ThermalProfile::HOT_BED],
            heaters: [
                HeaterState::new(ThermalProfile::HOT_END),
                HeaterState::new(ThermalProfile::HOT_BED),
            ],
        }
    }

    fn reset(&mut self) {
        self.motors = head_to_motors(self.initial_head);
        self.forward = [true; NUM_AXES];
        for (h, profile) in self.heaters.iter_mut().zip(self.profiles.iter()) {
            *h = HeaterState::new(*profile);
        }
    }
}

static MACHINE: Mutex<Machine> = Mutex::new(Machine::new());

fn machine() -> std::sync::MutexGuard<'static, Machine> {
    // A panicking test must not break the following ones
    MACHINE.lock().unwrap_or_else(|e| e.into_inner())
}

//#region Kinematics of the head

// The head position is kept in "doubled" world units so Core-XY needs no fractions:
// for cartesian machines the motors are the axes; for Core-XY A = X + Y, B = X - Y.

#[cfg(not(feature = "with-motion-core-xy-kinematics"))]
const fn head_to_motors(head: [i64; NUM_AXES]) -> [i64; NUM_AXES] {
    head
}

#[cfg(not(feature = "with-motion-core-xy-kinematics"))]
const fn motors_to_head(motors: [i64; NUM_AXES]) -> [i64; NUM_AXES] {
    motors
}

#[cfg(feature = "with-motion-core-xy-kinematics")]
const fn head_to_motors(head: [i64; NUM_AXES]) -> [i64; NUM_AXES] {
    [head[0] + head[1], head[0] - head[1], head[2]]
}

/// For Core-XY, X and Y are doubled (2X = A + B, 2Y = A - B): only the sign matters
#[cfg(feature = "with-motion-core-xy-kinematics")]
const fn motors_to_head(motors: [i64; NUM_AXES]) -> [i64; NUM_AXES] {
    [motors[0] + motors[1], motors[0] - motors[1], motors[2]]
}

//#endregion

/// The travel of each axis according to the board contract: world size * steps per world unit.
/// Unlimited when the world is not cartesian-like (the axes of the head are not the world axes).
fn default_travel() -> [i64; NUM_AXES] {
    cfg_if::cfg_if! {
        if #[cfg(all(feature = "with-motion", not(any(
            feature = "with-motion-delta-kinematics",
            feature = "with-motion-anthropomorphic-kinematics"))))] {
            use printhor_hwa_common::HwiContract;
            use printhor_hwa_common::math::CoordSel;
            type C = crate::board::Contract;
            let mut travel = [i64::MAX; NUM_AXES];
            for (i, coord) in [CoordSel::X, CoordSel::Y, CoordSel::Z].iter().enumerate() {
                let size = C::DEFAULT_WORLD_SIZE_WU.get_coord(*coord).map(|v| v.to_f64());
                let units = C::DEFAULT_UNITS_PER_WU.get_coord(*coord).map(|v| v.to_f64());
                let micro_steps = C::DEFAULT_MICRO_STEPS_PER_AXIS.get_coord(*coord);
                if let (Some(size), Some(units), Some(micro_steps)) = (size, units, micro_steps) {
                    travel[i] = (size * units * f64::from(micro_steps)).round() as i64;
                }
            }
            travel
        }
        else {
            [i64::MAX; NUM_AXES]
        }
    }
}

/// The limits of the head, in the units of [motors_to_head]
fn head_limits(m: &Machine) -> [i64; NUM_AXES] {
    let travel = m.travel.unwrap_or_else(default_travel);
    #[cfg(feature = "with-motion-core-xy-kinematics")]
    let travel = [travel[0].saturating_mul(2), travel[1].saturating_mul(2), travel[2]];
    travel
}

//#region Head API

/// Sets the travel of each axis (steps from the end-stop to the end of the frame).
/// `None` restores the world size of the board contract.
pub fn configure_travel(travel_steps: Option<[i64; NUM_AXES]>) {
    machine().travel = travel_steps;
}

/// Sets the position (steps from the end-stop of each axis) the head starts at, and moves it
/// there. Negative values are taken as 0 (at the end-stop).
pub fn configure_head(position_steps: [i64; NUM_AXES]) {
    let mut m = machine();
    m.initial_head = position_steps.map(|v| v.max(0));
    m.motors = head_to_motors(m.initial_head);
}

/// The current position of the head (steps from the end-stop of each axis)
pub fn head_position() -> [i64; NUM_AXES] {
    let head = motors_to_head(machine().motors);
    #[cfg(feature = "with-motion-core-xy-kinematics")]
    let head = [head[0] / 2, head[1] / 2, head[2]];
    head
}

/// Whether the end-stop of the axis is active (the head is at it)
pub fn end_stop_active(axis: Axis) -> bool {
    motors_to_head(machine().motors)[axis as usize] <= 0
}

/// The direction pin of a motor was set
pub(crate) fn set_direction(motor: Axis, forward: bool) {
    machine().forward[motor as usize] = forward;
}

/// A step pulse on a motor. It is lost when it would take the head beyond an end-stop.
pub(crate) fn step(motor: Axis) {
    let mut m = machine();
    let i = motor as usize;
    let mut next = m.motors;
    next[i] += if m.forward[i] { 1 } else { -1 };
    let head = motors_to_head(next);
    let limits = head_limits(&m);
    if head.iter().zip(limits.iter()).all(|(v, max)| *v >= 0 && v <= max) {
        m.motors = next;
    }
}

//#endregion

//#region Heater API

/// Sets the thermal behavior of a heater, and brings it to the initial state of the profile
pub fn configure_heater(heater: Heater, profile: ThermalProfile) {
    let mut m = machine();
    m.profiles[heater as usize] = profile;
    m.heaters[heater as usize] = HeaterState::new(profile);
}

/// Forces the temperature of a heater now (e.g. to simulate a cold draft or a sensor glitch).
/// The model goes on from it.
pub fn set_heater_temperature(heater: Heater, temp_c: f32) {
    let mut m = machine();
    let h = &mut m.heaters[heater as usize];
    h.update();
    h.temp_c = temp_c;
}

/// The current temperature of a heater (ºC)
pub fn heater_temperature(heater: Heater) -> f32 {
    let mut m = machine();
    let h = &mut m.heaters[heater as usize];
    h.update();
    h.temp_c
}

/// The power (0 to 1) the firmware is applying to a heater
pub fn heater_power(heater: Heater) -> f32 {
    machine().heaters[heater as usize].power
}

/// The duty of a PWM channel changed: it is the power of the heater it drives (if any)
pub(crate) fn set_pwm_power(channel: u8, power: f32) {
    if let Some(i) = HEATER_PWM_CHANNEL.iter().position(|c| *c == channel) {
        let mut m = machine();
        let h = &mut m.heaters[i];
        // The previous power applied until now
        h.update();
        h.power = power.clamp(0.0, 1.0);
    }
}

/// The heater whose thermistor is read by the ADC pin (if any) and its current temperature
pub(crate) fn thermistor_temperature(adc_pin: u8) -> Option<(Heater, f32)> {
    let i = HEATER_ADC_PIN.iter().position(|p| *p == adc_pin)?;
    let heater = [Heater::HotEnd, Heater::HotBed][i];
    Some((heater, heater_temperature(heater)))
}

//#endregion

/// Brings the machine back to the configured initial state: the head at its initial position
/// and the heaters at the initial temperature of their profile, off.
pub fn reset() {
    machine().reset();
}

/// Restores the default configuration (see [DEFAULT_HEAD_POSITION_STEPS], [ThermalProfile::HOT_END]
/// and [ThermalProfile::HOT_BED]) and resets the machine.
pub fn reset_to_defaults() {
    *machine() = Machine::new();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thermal_model_is_exact_whatever_the_sampling() {
        let p = ThermalProfile::HOT_END;
        let once = p.evolve(25.0, 1.0, 10.0);
        let mut stepped = 25.0;
        for _ in 0..1000 {
            stepped = p.evolve(stepped, 1.0, 0.01);
        }
        assert!((once - stepped).abs() < 1e-2, "{} vs {}", once, stepped);
        // Off, it cools down to the ambient
        assert!((p.evolve(200.0, 0.0, 1e4) - p.ambient_c).abs() < 1e-3);
    }

    #[test]
    fn thermistor_sample_inverts_the_beta_equation() {
        let t = Thermistor {
            beta: 3950.0,
            nominal_resistance: 100_000.0,
            pull_up_resistance: 4700.0,
        };
        for temp in [25.0f32, 60.0, 200.0, 250.0] {
            let s = t.sample(temp) as f32;
            let r = s * t.pull_up_resistance / (4095.0 - s);
            let back = 1.0 / ((r / t.nominal_resistance).ln() / t.beta + 1.0 / 298.15) - 273.15;
            assert!((back - temp).abs() < 1.5, "{} -> {}", temp, back);
        }
    }
}
