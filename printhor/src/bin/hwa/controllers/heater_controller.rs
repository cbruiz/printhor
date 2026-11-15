//! Mostly functional
//! Currently, this is a very naive approach using a simple control pid,
//! but willing to improve with a formal method.
//!
//!
//! AS-IS:
//!
//! ```
//! --+--> power -> pid -> diff --+
//!   |                           |
//!   +---------------------------+
//! ```
//!
//! TO-BE:
//!
//! <dl>
//!   <dt>1. Mathematical model</dt>
//!   <dd>
//!     The differential equation describing the temperature  $\(T(t)\)$ respecting the time $\(t\)$ is the following:
//!     <p>$ \[C \frac{dT(t)}{dt} = P(t) - R \cdot [T(t) - T_{\text{amb}}]\] $</p>
//!     <p>To solve the differential equation and obtain an expression for \(T(t)\), we separate variables and integrate the first order differential equation.</p>
//!     <p>Assuming initial conditions $\(T(0) = T_0\)$, The general solution:</p>
//!     <p>$ \[T(t) = T_{\text{amb}} + [T_0 - T_{\text{amb}}] \cdot e^{-\frac{t}{\tau}} + \frac{P_{\text{max}}}{R} \cdot \left(1 - e^{-\frac{t}{\tau}}\right)\] $</p>
//!     <p>
//!        Where:
//!        <ol>
//!          <li>\(T_{\text{amb}}\) is the ambient temperature.</li>
//!          <li>\(T_0\) is the initial temperature of the block.</li>
//!          <li>\(P_{\text{max}}\) is the maximum power of heating.</li>
//!          <li>\(\tau = \frac{C}{R}\) is the system time constant, which characterizes the speed of thermal response.</li>
//!        </ol>
//!        The equation provides a description of how the temperature evolves \(T(t)\) with the time \(t\) in response of the heating power \(P(t)\)
//!      </p>
//!   </dd>
//!   <dt>2. Temperature sensor</dt>
//!   <dd>
//!     <p>
//!       <dl>
//!         <dt>2.1.</dt>
//!         <dd>ADC mV Sampling with vref callibration if harware supports it</dd>
//!         <dt>2.2.</dt>
//!         <dd>Computation of thermistor resistive value (R_0) depending on circuit model.
//!            <p>Asuming a rload (R_1) in serie with thermisthor (R_0) connected to ground:</p>
//!            <p>V_{adc} = \frac{R_0}{R_0 + R_1}*V_{ref}</p>
//!            <p>Then:</p>
//!            <p>R_0 = \frac{V_{adc}*R_1}{V_{ref}-V_{adc}}</p>
//!         </dd>
//!         <dt>2.3.</dt>
//!         <dd>
//!            <p>With the measured resistive value and thermistor factors, convert to degrees leverating the Steinhart-Hart equation https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation:</p>
//!            <p>\frac{1}{T}=\frac{1}{T_0}+\frac{1}{B}*\ln{\frac{R}{R_0}}</p>
//!            <p>where:</p>
//!            <dl>
//!              <dt>T</dt><dd>Target temperature in Kelvin degrees</dd>
//!              <dt>T_0</dt><dd>Nominal temperature of thermistor at 25ºC</dd>
//!              <dt>B</dt><dd>Beta factor of the thermistor</dd>
//!              <dt>R</dt><dd>Nominal resistance of the thermistor</dd>
//!              <dt>R_0</dt><dd>Measured resistive value of thermisthor</dd>
//!            </dl>
//!         </dd>
//!       </dl>
//!     </p>
//!   </dd>
//!   <dt>3. Control model</dt>
//!   <dd>
//!    <p>
//!        Control system uses a continuous feedback PID to adjust
//!        the amount of heating supplied by a electric resistor to maintain the temperature of the heating block
//!        The POD controller computes \(P(t)\) respecting to the error (\(e(t) = T_{\text{ref}} - T(t)\)),
//!        The general equation of the PWM power control is:
//!    </p>
//!    <p>$\[P(t) = \text{PWM}(e(t)) \cdot P_{\text{max}}\]$</p>
//!    <p>where:</p>
//!    <dl>
//!      <dt>\(\text{PWM}(e(t))\)</dt><dd>The function that converts the error in a PWM duty cycle (between 0 and 1).</dd>
//!      <dt>\(P_{\text{max}}\)</dt><dd>The maximum power that electric resitor can supply.</dd>
//!    </dl>
//!  </dd>
//! </dl>
//!
//!
//! # Safety
//!
//! The controller implements the usual protections of a FDM firmware. On any fault, the heater is
//! shut down, the fault is latched (until `M999` or reset) and `SYS_ALARM` is raised, so that every
//! other heater is shut down too (see `task_temperature`).
//!
//! - **MAXTEMP**: The measured temperature reached the configured maximum.
//! - **MINTEMP**: The measured temperature is below the configured minimum while heating (typically
//!   a disconnected or broken thermistor).
//! - **Sensor error**: The measurement is not a number.
//! - **Heating failed** (watch): While heating, the temperature did not increase the configured
//!   amount in the configured period (heater cartridge or thermistor out of the block, broken heater...).
//! - **Thermal runaway**: Once the target was reached, the temperature fell below
//!   `target - hysteresis` for longer than the configured period.
//!
//! # Control
//!
//! A PID controller in physical units (compatible with Marlin's `M301`/`M304` constants), with the
//! output in the range [0, 255], derivative on the measurement and anti-windup. Out of the
//! functional range (±[PID_FUNCTIONAL_RANGE] ºC around the target) the heater runs in bang-bang mode.
//!
//! `M303` relay autotune (Åström–Hägglund + Ziegler–Nichols classic rules) is also provided.
use crate::hwa;
use hwa::DeferEvent::{AwaitRequested, Completed};
#[allow(unused)]
use hwa::math::ArithmeticOps;
#[allow(unused)]
use hwa::math::Real;
use hwa::{AsyncMutexStrategy, SyncMutexStrategy};
use hwa::{CommChannel, DeferAction};
use hwa::{EventFlags, EventStatus};

/// Max value of the PID output (Marlin's PID_MAX)
const PID_MAX: f32 = 255.0;

/// Out of this range (ºC) around the target, the PID is bypassed (full on/off) and its
/// integral term reset (Marlin's PID_FUNCTIONAL_RANGE)
const PID_FUNCTIONAL_RANGE: f32 = 10.0;

/// Smoothing factor of the derivative term (Marlin's PID_K1)
const PID_D_FILTER: f32 = 0.95;

/// Targets must be at least this margin (ºC) below the max temperature (Marlin's HOTEND_OVERSHOOT)
pub const TARGET_MAX_TEMP_MARGIN: f32 = 15.0;

/// Autotune is aborted when the temperature exceeds the target by this margin (ºC)
const AUTOTUNE_MAX_OVERSHOOT: f32 = 30.0;

/// Autotune is aborted after this time (seconds)
const AUTOTUNE_TIMEOUT_S: u64 = 20 * 60;

/// The safety and control settings of a heater
#[derive(Clone, Copy, Debug)]
pub struct HeaterConfig {
    /// MAXTEMP (ºC)
    pub max_temp: f32,
    /// MINTEMP (ºC)
    pub min_temp: f32,
    /// Thermal runaway hysteresis (ºC)
    pub protection_hysteresis: f32,
    /// Thermal runaway period (s)
    pub protection_period_s: u32,
    /// Heating watch increase (ºC)
    pub watch_increase: f32,
    /// Heating watch period (s)
    pub watch_period_s: u32,
    /// Temperature window to consider the target reached (ºC)
    pub temp_window: f32,
    /// Default PID constants (Kp, Ki, Kd)
    pub pid: (f32, f32, f32),
}

/// The reason of a heater fault
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum HeaterFault {
    /// Measured temperature reached MAXTEMP
    MaxTemp,
    /// Measured temperature below MINTEMP while heating
    MinTemp,
    /// The measurement is not a number
    SensorError,
    /// The temperature did not increase as expected while heating
    HeatingFailed,
    /// The temperature fell below the target after having reached it
    ThermalRunaway,
}

impl HeaterFault {
    pub const fn description(&self) -> &'static str {
        match self {
            HeaterFault::MaxTemp => "MAXTEMP triggered",
            HeaterFault::MinTemp => "MINTEMP triggered",
            HeaterFault::SensorError => "Temperature sensor error",
            HeaterFault::HeatingFailed => "Heating failed",
            HeaterFault::ThermalRunaway => "Thermal Runaway",
        }
    }
}

/// The reasons why a target temperature is rejected
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum HeaterError {
    /// The heater is in fault state. Requires `M999`
    Faulted(HeaterFault),
    /// The requested target exceeds the allowed max
    TargetTooHigh,
}

/// Outcome of a finished autotune, to be reported by the temperature task
#[derive(Clone, Copy, Debug)]
pub enum AutotuneOutcome {
    /// Computed PID constants (Kp, Ki, Kd). Already applied.
    Done(f32, f32, f32),
    /// Aborted: overshoot
    Overshoot,
    /// Aborted: timeout
    Timeout,
}

/// PID controller in physical units
struct PidControl {
    kp: f32,
    ki: f32,
    kd: f32,
    /// Integral of the error (ºC·s)
    integral: f32,
    /// Filtered derivative term
    d_term: f32,
    last_temp: Option<f32>,
}

impl PidControl {
    const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            d_term: 0.0,
            last_temp: None,
        }
    }

    fn reset(&mut self) {
        self.integral = 0.0;
        self.d_term = 0.0;
        self.last_temp = None;
    }

    /// Computes the output in [0, PID_MAX]
    fn next(&mut self, target: f32, temp: f32, dt: f32) -> f32 {
        let error = target - temp;
        let derivative = match self.last_temp {
            Some(last) if dt > 0.0 => (temp - last) / dt,
            _ => 0.0,
        };
        self.last_temp = Some(temp);
        // Derivative on measurement (no kick on target changes), low-pass filtered
        self.d_term = PID_D_FILTER * self.d_term + (1.0 - PID_D_FILTER) * (self.kd * derivative);

        // Out of the functional range the integral is frozen (not reset), so that the learned
        // bias is kept and the transfer back to PID control is smooth
        if target <= 0.0 {
            self.integral = 0.0;
            0.0
        } else if error > PID_FUNCTIONAL_RANGE {
            PID_MAX
        } else if error < -PID_FUNCTIONAL_RANGE {
            0.0
        } else {
            self.integral += error * dt;
            // Anti-windup: the integral term can not exceed the output range
            if self.ki > 0.0 {
                let max_integral = PID_MAX / self.ki;
                self.integral = self.integral.clamp(0.0, max_integral);
            }
            (self.kp * error + self.ki * self.integral - self.d_term).clamp(0.0, PID_MAX)
        }
    }
}

/// Thermal protection state machine
#[derive(Clone, Copy, Debug, PartialEq)]
enum ProtectionState {
    /// Heater off
    Inactive,
    /// Heating up to the target (watched)
    FirstHeating,
    /// Target was reached (runaway protected)
    Stable,
}

/// Relay autotune state
struct Autotune {
    target: f32,
    cycles: u8,
    max_cycles: u8,
    heating: bool,
    bias: f32,
    d: f32,
    t_start: embassy_time::Instant,
    t1: embassy_time::Instant,
    t2: embassy_time::Instant,
    t_high: f32,
    t_low: f32,
    max_t: f32,
    min_t: f32,
    gains: Option<(f32, f32, f32)>,
}

impl Autotune {
    fn new(target: f32, max_cycles: u8, now: embassy_time::Instant) -> Self {
        Self {
            target,
            cycles: 0,
            max_cycles: max_cycles.max(3),
            heating: true,
            bias: PID_MAX / 2.0,
            d: PID_MAX / 2.0,
            t_start: now,
            t1: now,
            t2: now,
            t_high: 0.0,
            t_low: 0.0,
            max_t: f32::MIN,
            min_t: f32::MAX,
            gains: None,
        }
    }

    /// Advances the relay autotune. Returns the power to apply, or the outcome when finished.
    fn step(&mut self, temp: f32, now: embassy_time::Instant) -> Result<f32, AutotuneOutcome> {
        if temp > self.target + AUTOTUNE_MAX_OVERSHOOT {
            return Err(AutotuneOutcome::Overshoot);
        }
        if (now - self.t_start).as_secs() > AUTOTUNE_TIMEOUT_S {
            return Err(AutotuneOutcome::Timeout);
        }
        self.max_t = self.max_t.max(temp);
        self.min_t = self.min_t.min(temp);
        let min_half_cycle = embassy_time::Duration::from_secs(5);

        if self.heating && temp > self.target && (now - self.t2) > min_half_cycle {
            self.heating = false;
            self.t1 = now;
            self.t_high = (self.t1 - self.t2).as_millis() as f32 / 1000.0;
            self.max_t = self.target;
        }
        if !self.heating && temp < self.target && (now - self.t1) > min_half_cycle {
            self.heating = true;
            self.t2 = now;
            self.t_low = (self.t2 - self.t1).as_millis() as f32 / 1000.0;
            if self.cycles > 0 {
                let period = self.t_low + self.t_high;
                if period > 0.0 {
                    self.bias += (self.d * (self.t_high - self.t_low)) / period;
                }
                self.bias = self.bias.clamp(20.0, PID_MAX - 20.0);
                self.d = if self.bias > PID_MAX / 2.0 {
                    PID_MAX - 1.0 - self.bias
                } else {
                    self.bias
                };
                let amplitude = (self.max_t - self.min_t) / 2.0;
                if self.cycles > 2 && amplitude > 0.0 && period > 0.0 {
                    let ku = (4.0 * self.d) / (core::f32::consts::PI * amplitude);
                    let tu = period;
                    let kp = 0.6 * ku;
                    let ki = 2.0 * kp / tu;
                    let kd = kp * tu / 8.0;
                    self.gains = Some((kp, ki, kd));
                }
            }
            self.cycles += 1;
            self.min_t = self.target;
        }
        if self.cycles > self.max_cycles {
            if let Some((kp, ki, kd)) = self.gains {
                return Err(AutotuneOutcome::Done(kp, ki, kd));
            }
        }
        Ok(match self.heating {
            true => self.bias + self.d,
            false => self.bias - self.d,
        })
    }
}

/// A controller struct for managing a heater device.
///
/// This controller interacts with both an ADC (Analog-Digital Converter)
/// and a PWM (Pulse-Width Modulation) controller to measure and regulate the temperature.
pub struct HeaterController<HA, HP>
where
    HA: AsyncMutexStrategy + 'static,
    HA::Resource: hwa::traits::UnifiedAdc16 + 'static,
    HP: SyncMutexStrategy + 'static,
    HP::Resource: hwa::traits::Pwm + 'static,
    <HP::Resource as hwa::traits::Pwm>::Channel: Copy,
    <HP::Resource as hwa::traits::Pwm>::Duty:
        core::fmt::Debug + Copy + Ord + Into<u32> + From<u16> + TryFrom<u32>,
{
    /// Shared Analog-Digital Converter (ADC) controller used to measure temperature.
    adc: hwa::controllers::GenericAdcController<HA>,
    /// Precomputed ratio of volts per ADC unit, used for voltage to temperature conversion.
    v_ratio: f32,
    beta: f32,
    r_nominal: f32,
    r_pull_up: f32,
    /// Shared Pulse-Width Modulation (PWM) controller used to apply heating power.
    pwm: hwa::controllers::GenericPwmController<HP>,
    /// The target or expected temperature value in Celsius.
    target_temp: f32,
    /// Last measured resistance value in Ohms, cached for quick access.
    current_resistance: f32,
    current_channel: CommChannel,
    /// Order number of the most recently registered temperature-await request.
    pending_order_num: u32,
    /// Whether the pending await must also wait for cooling down (M109 R)
    wait_cooling: bool,
    state_machine: HeaterStateMachine,
    defer_channel: hwa::types::DeferChannel,
    defer_action: DeferAction,
    temperature_flags: EventFlags,
    /// Cache of the heater controller's operational state (on or off).
    on: bool,
    config: HeaterConfig,
    pid: PidControl,
    /// Last applied power in [0, 1]
    power: f32,
    last_update: Option<embassy_time::Instant>,
    fault: Option<HeaterFault>,
    protection: ProtectionState,
    /// Heating watch: (deadline, temperature to reach)
    watch: Option<(embassy_time::Instant, f32)>,
    /// Thermal runaway deadline
    runaway_deadline: Option<embassy_time::Instant>,
    autotune: Option<Autotune>,
    autotune_outcome: Option<AutotuneOutcome>,
}
impl<HA, HP> HeaterController<HA, HP>
where
    HA: AsyncMutexStrategy + 'static,
    HA::Resource: 'static,
    HA::Resource: hwa::traits::UnifiedAdc16 + 'static,
    HP: SyncMutexStrategy + 'static,
    HP::Resource: hwa::traits::Pwm + 'static,
    <HP::Resource as hwa::traits::Pwm>::Channel: Copy,
    <HP::Resource as hwa::traits::Pwm>::Duty:
        core::fmt::Debug + Copy + Ord + Into<u32> + From<u16> + TryFrom<u32>,
{
    pub fn new(
        adc: hwa::controllers::GenericAdcController<HA>,
        pwm: hwa::controllers::GenericPwmController<HP>,
        beta: f32,
        r_nominal: f32,
        r_pull_up: f32,
        defer_channel: hwa::types::DeferChannel,
        defer_action: DeferAction,
        defer_flags: EventFlags,
        config: HeaterConfig,
    ) -> Self {
        Self {
            adc,
            v_ratio: 1.0f32,
            pwm,
            beta,
            r_nominal,
            r_pull_up,
            target_temp: 0.0f32,
            current_resistance: 0.0f32,
            current_channel: CommChannel::Internal,
            pending_order_num: 0,
            wait_cooling: false,
            state_machine: HeaterStateMachine::new(),
            on: false,
            defer_channel,
            defer_action,
            temperature_flags: defer_flags,
            config,
            pid: PidControl::new(config.pid.0, config.pid.1, config.pid.2),
            power: 0.0,
            last_update: None,
            fault: None,
            protection: ProtectionState::Inactive,
            watch: None,
            runaway_deadline: None,
            autotune: None,
            autotune_outcome: None,
        }
    }
    pub async fn init(&mut self, v_ref_default: u16) {
        self.adc.init(v_ref_default).await;
    }

    pub fn get_target_temp(&self) -> f32 {
        self.target_temp
    }

    /// The highest target temperature accepted
    pub fn max_target_temp(&self) -> f32 {
        self.config.max_temp - TARGET_MAX_TEMP_MARGIN
    }

    pub fn get_config(&self) -> &HeaterConfig {
        &self.config
    }

    /// Test hook: changes the configuration (e.g. a shorter heating watch period, so that a
    /// protection scenario does not wait as long as on a real machine)
    #[cfg(test)]
    pub fn config_mut(&mut self) -> &mut HeaterConfig {
        &mut self.config
    }

    /// The latched fault, if any
    pub fn fault(&self) -> Option<HeaterFault> {
        self.fault
    }

    /// Clears the latched fault (M999). The heater remains off.
    pub fn clear_fault(&mut self) {
        self.fault = None;
        self.protection = ProtectionState::Inactive;
        self.watch = None;
        self.runaway_deadline = None;
    }

    /// Current PID constants (Kp, Ki, Kd)
    pub fn get_pid(&self) -> (f32, f32, f32) {
        (self.pid.kp, self.pid.ki, self.pid.kd)
    }

    /// Sets the PID constants. `None` keeps the current value (M301/M304)
    pub fn set_pid(&mut self, kp: Option<f32>, ki: Option<f32>, kd: Option<f32>) {
        if let Some(kp) = kp {
            self.pid.kp = kp;
        }
        if let Some(ki) = ki {
            self.pid.ki = ki;
        }
        if let Some(kd) = kd {
            self.pid.kd = kd;
        }
        self.pid.reset();
    }

    /// Restores the default PID constants (M502)
    pub fn reset_pid(&mut self) {
        let (kp, ki, kd) = self.config.pid;
        self.set_pid(Some(kp), Some(ki), Some(kd));
    }

    /// Sets the target temperature for the heater.
    ///
    /// # Parameters
    ///
    /// * `channel`: The communication channel that has sent the current request. When it is not
    ///   [CommChannel::Internal], the response is deferred until the target is reached.
    /// * `action`: The action to be deferred.
    /// * `requested_temp`: The newly requested temperature in Celsius degrees. Zero or less turns
    ///   the heater off.
    /// * `wait_cooling`: When deferred, also wait if the heater has to cool down (M109 R). Otherwise
    ///   the response is not deferred when the current temperature is above the target.
    ///
    /// # Returns
    ///
    /// * `Ok(true)` if the response is deferred.
    /// * `Err` if the request is rejected (faulted heater or target too high).
    pub async fn set_target_temp(
        &mut self,
        channel: CommChannel,
        action: DeferAction,
        requested_temp: f32,
        order_num: u32,
        wait_cooling: bool,
    ) -> Result<bool, HeaterError> {
        if requested_temp > 0.0f32 {
            if let Some(fault) = self.fault {
                return Err(HeaterError::Faulted(fault));
            }
            if requested_temp > self.max_target_temp() {
                return Err(HeaterError::TargetTooHigh);
            }
            // A new target cancels a running autotune
            self.autotune = None;
            let previous_target = self.target_temp;
            let was_on = self.is_on();
            self.target_temp = requested_temp;
            self.on();
            if !was_on || requested_temp != previous_target {
                self.pid.reset();
                self.start_protection();
            }
            // M104/M140 use CommChannel::Internal: they are immediate commands and must not block
            // the host. Only defer when the caller holds a real host channel.
            if channel == CommChannel::Internal {
                return Ok(false);
            }
            Ok(self
                .subscribe_if_needed(channel, action, order_num, wait_cooling)
                .await)
        } else {
            self.target_temp = 0.0f32;
            self.current_channel = CommChannel::Internal;
            self.autotune = None;
            self.off();
            Ok(false)
        }
    }

    /// Defers the response until the current target is reached (M109/M190 with no `S`).
    ///
    /// Returns `true` if the response is deferred.
    pub async fn ping_subscribe(
        &mut self,
        channel: CommChannel,
        action: DeferAction,
        order_num: u32,
    ) -> bool {
        if self.is_on() && self.target_temp > 0.0f32 {
            self.subscribe_if_needed(channel, action, order_num, false)
                .await
        } else {
            false
        }
    }

    async fn subscribe_if_needed(
        &mut self,
        channel: CommChannel,
        action: DeferAction,
        order_num: u32,
        wait_cooling: bool,
    ) -> bool {
        let current = self.state_machine.current_temperature;
        let must_wait = if wait_cooling {
            !self.is_within_window()
        } else {
            current < self.target_temp - self.config.temp_window
        };
        if must_wait {
            self.current_channel = channel;
            self.pending_order_num = order_num;
            self.wait_cooling = wait_cooling;
            self.defer_channel
                .send(AwaitRequested(action, channel, order_num))
                .await;
            true
        } else {
            self.current_channel = CommChannel::Internal;
            false
        }
    }

    fn is_within_window(&self) -> bool {
        (self.state_machine.current_temperature - self.target_temp).abs()
            <= self.config.temp_window
    }

    /// Starts a relay PID autotune at the given target (M303)
    pub fn start_autotune(&mut self, target: f32, cycles: u8) -> Result<(), HeaterError> {
        if let Some(fault) = self.fault {
            return Err(HeaterError::Faulted(fault));
        }
        if target > self.max_target_temp() {
            return Err(HeaterError::TargetTooHigh);
        }
        self.target_temp = target;
        self.on();
        self.start_protection();
        self.autotune = Some(Autotune::new(target, cycles, embassy_time::Instant::now()));
        self.autotune_outcome = None;
        Ok(())
    }

    pub fn is_autotuning(&self) -> bool {
        self.autotune.is_some()
    }

    /// Takes the outcome of the last finished autotune, if any
    pub fn take_autotune_outcome(&mut self) -> Option<AutotuneOutcome> {
        self.autotune_outcome.take()
    }

    fn start_protection(&mut self) {
        self.protection = ProtectionState::FirstHeating;
        self.runaway_deadline = None;
        self.arm_watch();
    }

    /// Arms the heating watch, only when the temperature is far enough from the target
    fn arm_watch(&mut self) {
        let current = self.state_machine.current_temperature;
        let threshold = self.target_temp
            - (self.config.watch_increase + self.config.protection_hysteresis + 1.0);
        self.watch = if current < threshold {
            Some((
                embassy_time::Instant::now()
                    + embassy_time::Duration::from_secs(self.config.watch_period_s as u64),
                current + self.config.watch_increase,
            ))
        } else {
            None
        };
    }

    // Updates the latest measured temperature in Celsius degrees
    pub fn set_current_temp(&mut self, current_temp: f32) {
        self.state_machine.current_temperature = current_temp;
    }

    // Gets the latest measured temperature in Celsius degrees
    pub fn get_current_temp(&mut self) -> f32 {
        self.state_machine.current_temperature
    }

    // Gets the latest measured resistance in Ohms
    pub fn get_current_resistance(&mut self) -> f32 {
        self.current_resistance
    }

    // Sets the applied power in scale between 0 and 100
    pub fn set_power(&mut self, power: u8) {
        self.power = (power.min(100) as f32) / 100.0;
        self.pwm.set_power(power);
    }

    // Gets the applied power in scale between 0.0 and 1.0
    pub fn get_current_power(&mut self) -> f32 {
        self.pwm.get_power()
    }

    /// Reads the current temperature in Celsius degrees.
    ///
    /// Returns the measured resistance of the thermistor.
    pub async fn read_temp(&mut self) -> f32 {
        let value = self.adc.read_sample().await;
        (
            self.state_machine.current_temperature,
            self.current_resistance,
        ) = self.convert_to_celcius(value);
        self.current_resistance
    }

    /// Measures the temperature aplying the SteinHart-Hart equation
    /// Steps:
    /// 1 - Compute the resistance of the thermistor
    /// 2 - Compute the temperature applying the β parameter Steinhart–Hart equation
    /// Returns a tuple with:
    /// * The Temperature in Celsius degrees
    /// * Even though it's not strictly necessary, the measured resistance, which is really useful to debug and review
    fn convert_to_celcius(&self, sample: u16) -> (f32, f32) {
        if sample > 4095 {
            // Out of the 12-bit range: not a valid reading (sensor error)
            return (f32::NAN, f32::NAN);
        }
        let sample_v = f32::from(sample) * self.v_ratio;
        let measured_resistance: f32 =
            (sample_v * self.r_pull_up) / ((4095.0f32 * self.v_ratio) - sample_v);

        let log_mr_by_r_nominal: f32 = accurate_ln(measured_resistance / self.r_nominal);
        (
            (1.0 / (log_mr_by_r_nominal / self.beta + 1.0 / 298.15)) - 273.15,
            measured_resistance,
        )
    }

    // Sends and consume the deferred action
    pub async fn flush_notification(&mut self, action: DeferAction, order_num: u32) {
        self.defer_channel
            .send(Completed(action, self.current_channel, order_num))
            .await;
        self.current_channel = CommChannel::Internal;
    }

    pub fn is_awaited(&self) -> bool {
        !matches!(self.current_channel, CommChannel::Internal)
    }

    // Checks if heater is enabled
    pub fn is_on(&self) -> bool {
        self.on
    }
    pub fn on(&mut self) {
        self.on = true;
    }
    pub fn off(&mut self) {
        self.on = false;
        self.protection = ProtectionState::Inactive;
        self.watch = None;
        self.runaway_deadline = None;
        self.pid.reset();
        self.set_power(0);
    }

    /// Shuts the heater down because of a safety reason (SYS_ALARM): target is reset and any
    /// running autotune is cancelled.
    pub fn shutdown(&mut self) {
        self.target_temp = 0.0;
        self.autotune = None;
        self.off();
    }

    /// Latches a fault and shuts the heater down
    fn trigger_fault(&mut self, fault: HeaterFault) {
        hwa::error!(
            "[Heater {:?}] {} (temp: {:?} target: {:?}). Shutting down",
            self.defer_action,
            fault.description(),
            Real::from_f32(self.state_machine.current_temperature),
            Real::from_f32(self.target_temp),
        );
        self.fault = Some(fault);
        self.shutdown();
    }

    /// Checks the safety conditions with the latest measurement
    fn check_safety(&mut self, temp: f32, now: embassy_time::Instant) -> Option<HeaterFault> {
        if temp.is_nan() {
            // A disconnected sensor of an unused heater must not halt the machine
            return self.is_on().then_some(HeaterFault::SensorError);
        }
        if temp >= self.config.max_temp {
            return Some(HeaterFault::MaxTemp);
        }
        if !self.is_on() {
            return None;
        }
        if temp < self.config.min_temp {
            return Some(HeaterFault::MinTemp);
        }
        let target = self.target_temp;
        let hysteresis = self.config.protection_hysteresis;
        match self.protection {
            ProtectionState::Inactive => {}
            ProtectionState::FirstHeating => {
                if temp >= target - hysteresis {
                    self.protection = ProtectionState::Stable;
                    self.watch = None;
                    self.runaway_deadline = None;
                } else if let Some((deadline, watch_temp)) = self.watch {
                    if now >= deadline {
                        if temp < watch_temp {
                            return Some(HeaterFault::HeatingFailed);
                        }
                        self.arm_watch();
                    }
                }
            }
            ProtectionState::Stable if self.autotune.is_some() => {
                // The relay autotune oscillates around the target on purpose. MAXTEMP, MINTEMP and
                // the autotune overshoot and timeout limits still apply.
                self.runaway_deadline = None;
            }
            ProtectionState::Stable => {
                if temp < target - hysteresis {
                    match self.runaway_deadline {
                        None => {
                            self.runaway_deadline = Some(
                                now + embassy_time::Duration::from_secs(
                                    self.config.protection_period_s as u64,
                                ),
                            );
                        }
                        Some(deadline) if now >= deadline => {
                            return Some(HeaterFault::ThermalRunaway);
                        }
                        _ => {}
                    }
                } else {
                    self.runaway_deadline = None;
                }
            }
        }
        None
    }

    /// Updates the state machine with the current temperature readings and
    /// performs the required actions based on the new temperature state:
    ///
    /// 1. Reads the current temperature.
    /// 2. Checks the safety conditions. On fault, the heater is shut down and the fault latched.
    /// 3. Computes the power to apply (PID or autotune).
    /// 4. Publishes the temperature flags and flushes the deferred responses when the target is reached.
    ///
    /// Returns the fault detected in this update, if any.
    pub async fn update(&mut self, event_bus: &hwa::types::EventBus) -> Option<HeaterFault> {
        hwa::trace!("Updating HEATER...");
        let now = embassy_time::Instant::now();
        let dt = match self.last_update {
            Some(last) => ((now - last).as_micros() as f32 / 1_000_000.0).clamp(0.0, 10.0),
            None => 0.0,
        };
        self.last_update = Some(now);

        let _ = self.read_temp().await;

        let temp = self.state_machine.current_temperature;

        let new_fault = match self.fault {
            Some(_) => None,
            None => self.check_safety(temp, now),
        };
        if let Some(fault) = new_fault {
            self.trigger_fault(fault);
            event_bus
                .publish_event(
                    EventStatus::containing(EventFlags::SYS_ALARM)
                        .and_not_containing(self.temperature_flags),
                )
                .await;
            self.state_machine.state = State::Duty;
            return new_fault;
        }

        let new_state = {
            hwa::trace!(
                "MEASURED_TEMP[{:?}] {:?}",
                self.defer_action,
                Real::from_f32(temp)
            );
            if self.fault.is_none() && self.is_on() {
                let output = match self.autotune.as_mut().map(|at| at.step(temp, now)) {
                    Some(Ok(power)) => power,
                    Some(Err(outcome)) => {
                        self.autotune = None;
                        self.autotune_outcome = Some(outcome);
                        if let AutotuneOutcome::Done(kp, ki, kd) = outcome {
                            self.set_pid(Some(kp), Some(ki), Some(kd));
                        }
                        // Autotune leaves the heater off, as Marlin does
                        self.target_temp = 0.0;
                        self.off();
                        0.0
                    }
                    None => self.pid.next(self.target_temp, temp, dt),
                };
                if self.is_on() {
                    let percent = ((output / PID_MAX) * 100.0).clamp(0.0, 100.0);
                    self.set_power(percent as u8);
                }
                if !self.is_on() {
                    State::Duty
                } else if self.is_within_window() {
                    State::Maintaining
                } else {
                    State::Targeting
                }
            } else {
                if self.power > 0.0 {
                    self.set_power(0);
                }
                State::Duty
            }
        };

        if new_state != self.state_machine.state {
            hwa::trace!("Temp changed to {:?}", new_state);
            match new_state {
                State::Duty | State::Targeting => {
                    event_bus
                        .publish_event(EventStatus::not_containing(self.temperature_flags))
                        .await;
                }
                State::Maintaining => {
                    hwa::debug!("Temperature reached. Firing {:?}.", self.temperature_flags);
                    event_bus
                        .publish_event(EventStatus::containing(self.temperature_flags))
                        .await;
                }
            }
            self.state_machine.state = new_state;
        }
        if self.is_awaited() {
            let reached = match self.wait_cooling {
                true => new_state == State::Maintaining,
                false => {
                    new_state == State::Maintaining
                        || (self.is_on()
                            && temp >= self.target_temp - self.config.temp_window)
                }
            };
            if reached {
                self.flush_notification(self.defer_action, self.pending_order_num)
                    .await;
            }
        }
        None
    }
}

/// The `State` enum represents the different states in which the HeaterStateMachine can be during its execution.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
enum State {
    /// The `Duty` state indicates that the system is operating in its default duty mode.
    Duty,
    /// The `Targeting` state is when the system is trying to reach a target temperature.
    Targeting,
    /// The `Maintaining` state indicates that the system is maintaining the current temperature.
    Maintaining,
}

/// Represents the finite state machine for handling the heater's operation states.
struct HeaterStateMachine {
    /// Current temperature being read from the heater.
    current_temperature: f32,
    /// The current state of the heater's finite state machine (FSM).
    state: State,
}

impl HeaterStateMachine {
    fn new() -> Self {
        Self {
            current_temperature: 0f32,
            state: State::Duty,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_full_power_far_below_target() {
        let mut pid = PidControl::new(22.2, 1.08, 114.0);
        assert_eq!(pid.next(200.0, 25.0, 0.5), PID_MAX);
        // Integral frozen out of the functional range
        assert_eq!(pid.integral, 0.0);
    }

    #[test]
    fn pid_off_far_above_target() {
        let mut pid = PidControl::new(22.2, 1.08, 114.0);
        assert_eq!(pid.next(200.0, 230.0, 0.5), 0.0);
        assert_eq!(pid.next(0.0, 25.0, 0.5), 0.0);
    }

    #[test]
    fn pid_integral_is_bounded() {
        let mut pid = PidControl::new(22.2, 1.08, 114.0);
        for _ in 0..10_000 {
            let _ = pid.next(200.0, 195.0, 0.5);
        }
        assert!(pid.integral * pid.ki <= PID_MAX + 0.001);
        let out = pid.next(200.0, 195.0, 0.5);
        assert!(out >= 0.0 && out <= PID_MAX);
    }

    /// Closed loop with a first-order plant: the PID must settle near the target
    #[test]
    fn pid_settles_on_first_order_plant() {
        let mut pid = PidControl::new(21.73, 1.54, 76.55);
        let (amb, gain, tau, dt) = (22.0f32, 330.0f32, 20.0f32, 0.5f32);
        let mut temp = amb;
        for _ in 0..2400 {
            let power = pid.next(200.0, temp, dt) / PID_MAX;
            let t_inf = amb + gain * power;
            temp += (t_inf - temp) * (1.0 - (-dt / tau).exp());
        }
        assert!((temp - 200.0).abs() < 2.0, "settled at {}", temp);
    }

    /// Relay autotune on a first-order plant: finishes with positive gains that make the PID
    /// settle on the same plant
    #[test]
    fn autotune_on_first_order_plant() {
        let (amb, gain, tau, dt) = (22.0f32, 330.0f32, 20.0f32, 0.5f32);
        let t0 = embassy_time::Instant::from_ticks(0);
        let mut now = t0;
        let mut at = Autotune::new(200.0, 5, t0);
        let mut temp = amb;
        let gains = loop {
            match at.step(temp, now) {
                Ok(output) => {
                    let t_inf = amb + gain * (output / PID_MAX);
                    temp += (t_inf - temp) * (1.0 - (-dt / tau).exp());
                    now += embassy_time::Duration::from_millis(500);
                }
                Err(AutotuneOutcome::Done(kp, ki, kd)) => break (kp, ki, kd),
                Err(other) => panic!("autotune failed: {:?}", other),
            }
            assert!((now - t0).as_secs() < 3600, "autotune does not converge");
        };
        assert!(gains.0 > 0.0 && gains.1 > 0.0 && gains.2 > 0.0, "gains: {:?}", gains);
        let mut pid = PidControl::new(gains.0, gains.1, gains.2);
        let mut temp = amb;
        for _ in 0..2400 {
            let power = pid.next(200.0, temp, dt) / PID_MAX;
            temp += (amb + gain * power - temp) * (1.0 - (-dt / tau).exp());
        }
        assert!((temp - 200.0).abs() < 2.0, "tuned PID settled at {}", temp);
    }

    #[test]
    fn heater_fault_descriptions() {
        assert_eq!(HeaterFault::ThermalRunaway.description(), "Thermal Runaway");
        assert_eq!(HeaterFault::MaxTemp.description(), "MAXTEMP triggered");
    }
}

#[cfg(test)]
mod safety_tests;

/// Natural logarithm accurate to 1 ulp in f32 (micromath's `ln` is a coarse approximation that
/// yields several ºC of error in the thermistor conversion). Pure f32 (fdlibm's `logf` scheme):
/// f64 would link the soft-double routines (~3.5 KB) on the boards without double-precision FPU.
///
/// `x = m·2^e` with `m ∈ [√½, √2)`, `f = m-1` (exact), `s = f/(2+f)`, `|s| < 0.172`, and
/// `ln(m) = f - f²/2 + s·(f²/2 + R(s²))` where `R` is the odd atanh series (four terms reach f32
/// precision). `e·ln2` is split in hi/lo parts so that the sum keeps full precision.
/// Verified exhaustively against f64 `ln` (1 ulp max; < 0.0001 ºC in the thermistor range).
fn accurate_ln(x: f32) -> f32 {
    if x.is_nan() || x < 0.0 {
        return f32::NAN;
    }
    if x == 0.0 {
        return f32::NEG_INFINITY;
    }
    if x.is_infinite() {
        return f32::INFINITY;
    }
    let (mut bits, mut e) = (x.to_bits(), 0i32);
    if bits < 0x0080_0000 {
        // Subnormal: normalize first (×2^24)
        bits = (x * 16_777_216.0).to_bits();
        e -= 24;
    }
    e += ((bits >> 23) as i32) - 127;
    let mut m = f32::from_bits((bits & 0x007F_FFFF) | 0x3F80_0000);
    if m > core::f32::consts::SQRT_2 {
        m *= 0.5;
        e += 1;
    }
    // ln2 = LN2_HI + LN2_LO, LN2_HI with trailing zero bits so that `e·LN2_HI` is exact
    const LN2_HI: f32 = 6.931_381_225_6e-1;
    const LN2_LO: f32 = 9.058_000_614_5e-6;
    let f = m - 1.0;
    let s = f / (2.0 + f);
    let s2 = s * s;
    let r = s2 * (2.0 / 3.0 + s2 * (2.0 / 5.0 + s2 * (2.0 / 7.0 + s2 * (2.0 / 9.0))));
    let hfsq = 0.5 * f * f;
    let ef = e as f32;
    ef * LN2_HI - ((hfsq - (s * (hfsq + r) + ef * LN2_LO)) - f)
}
