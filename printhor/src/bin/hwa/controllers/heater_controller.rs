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
use crate::hwa;
use hwa::DeferEvent::{AwaitRequested, Completed};
#[allow(unused)]
use hwa::math::ArithmeticOps;
#[allow(unused)]
use hwa::math::Real;
use hwa::{AsyncMutexStrategy, SyncMutexStrategy};
use hwa::{CommChannel, DeferAction};
use hwa::{EventFlags, EventStatus};

/// A controller struct for managing a heater device.
///
/// This controller interacts with both an ADC (Analog-Digital Converter)
/// and a PWM (Pulse-Width Modulation) controller to measure and regulate the temperature.
///
/// # Generics
/// TBD
///
/// # Fields
/// * `adc`: A reference to the ADC controller used for temperature measurement.
/// * `adc_pin`: The pin used by the ADC controller for reading values.
/// * `v_ratio`: The precomputed ratio of volts per ADC unit for voltage to temperature conversion.
/// * `pwm`: A PWM controller used to apply heating power.
/// * `defer_channel`: A channel for submitting status changes for deferred processing.
/// * `target_temp`: The target or desired temperature value in Celsius.
/// * `current_resistance`: The last measured resistance value in Ohms, cached for quick access.
/// * `commander_channel`: The communication channel that sent the current request.
/// * `on`: A boolean indicating whether the heater is currently on.
/// * `thermistor_properties`: Properties of the thermistor including its resistance and beta coefficient.
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
    state_machine: HeaterStateMachine,
    defer_channel: hwa::types::DeferChannel,
    defer_action: DeferAction,
    temperature_flags: EventFlags,
    /// Cache of the heater controller's operational state (on or off).
    on: bool,
    #[cfg(feature = "native")]
    t0: embassy_time::Instant,
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
            state_machine: HeaterStateMachine::new(),
            on: false,
            defer_channel,
            defer_action,
            temperature_flags: defer_flags,
            #[cfg(feature = "native")]
            t0: embassy_time::Instant::now(),
        }
    }
    pub async fn init(&mut self, v_ref_default: u16) {
        self.adc.init(v_ref_default).await;
    }

    pub fn get_target_temp(&self) -> f32 {
        self.target_temp
    }

    /// Sets the target temperature for the heater.
    ///
    /// This method is responsible for setting a new target temperature and determining whether
    /// the command to reach that temperature should be deferred. If the requested temperature
    /// is significantly different from the current target temperature, the method will trigger
    /// a defer event.
    ///
    /// # Parameters
    ///
    /// * `channel`: The communication channel that has sent the current request.
    /// * `action`: The action to be deferred.
    /// * `requested_temp`: The newly requested temperature in Celsius degrees.
    ///
    /// # Returns
    ///
    /// * `bool` - Returns `true` if the command is deferred (i.e., it needs to delay ACK to reach the desired temperature).
    ///
    /// # Notes
    ///
    /// * If the requested temperature is above `0.0f32`, the heater will be turned on.
    /// * If the difference between the requested temperature and the current target temperature
    ///   is significant (more than 10%), a defer event will be triggered.
    /// * If the requested temperature is `0.0f32` or less, the heater will be turned off.
    pub async fn set_target_temp(
        &mut self,
        channel: CommChannel,
        action: DeferAction,
        requested_temp: f32,
        order_num: u32,
    ) -> bool {
        if requested_temp > 0.0f32 {
            let tdiff = requested_temp - self.target_temp;
            self.target_temp = requested_temp;
            self.on();
            if tdiff.abs() > 0.1f32 * self.target_temp {
                self.current_channel = channel;
                self.defer_channel
                    .send(AwaitRequested(action, channel, order_num))
                    .await;
                true
            } else {
                self.current_channel = CommChannel::Internal;
                false
            }
        } else {
            self.target_temp = 0.0f32;
            self.current_channel = CommChannel::Internal;
            self.off();
            false
        }
    }

    /// Checks if the temperature is within an acceptable range compared to the target temperature.
    ///
    /// If the heater is on and the measured temperature differs from the target temperature by
    /// more than 10%, this method will trigger a defer event, requesting to delay processing until
    /// the temperature stabilizes.
    ///
    /// # Parameters
    ///
    /// * `channel`: The communication channel that sent the current request.
    /// * `action`: The action to be deferred if the temperature is not within the acceptable range.
    ///
    /// # Returns
    ///
    /// * `bool` - Returns `true` if the command is deferred (indicating a need to delay ACK
    ///   to allow the temperature to stabilize), otherwise returns `false`.
    ///
    /// # Notes
    ///
    /// * If the heater is off or the current temperature is not greater than zero, the method
    ///   returns `false` without triggering a defer event.
    /// * Otherwise, it compares the current temperature with the target temperature.
    /// * If the difference exceeds 10% of the target temperature, a defer event is triggered,
    ///   and the method returns `true`. The commander's channel is updated to the provided channel.
    /// * If the temperature difference is within the 10% range, the method returns `false`
    ///   and sets the commander's channel to `CommChannel::Internal`.
    pub async fn ping_subscribe(
        &mut self,
        channel: CommChannel,
        action: DeferAction,
        order_num: u32,
    ) -> bool {
        if self.is_on() && self.state_machine.current_temperature > 0.0f32 {
            let tdiff = self.state_machine.current_temperature - self.target_temp;
            if tdiff.abs() > 0.1f32 * self.target_temp {
                self.current_channel = channel;
                self.defer_channel
                    .send(AwaitRequested(action, channel, order_num))
                    .await;
                true
            } else {
                self.current_channel = CommChannel::Internal;
                false
            }
        } else {
            false
        }
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
        self.pwm.set_power(power);
    }

    // Gets the applied power in scale between 0.0 and 1.0
    pub fn get_current_power(&mut self) -> f32 {
        self.pwm.get_power()
    }

    // Sets the applied power in scale between 0 and 100
    /// Reads the current temperature in Celsius degrees.
    ///
    /// # Returns
    ///
    /// * `f32` - The current temperature measurement in Celsius degrees.
    ///
    /// # Implementation Note
    ///
    /// This asynchronous function locks the ADC bus to perform a temperature
    /// measurement. Depending on the feature configuration, the measurement
    /// can be performed asynchronously or synchronously. The raw ADC value read
    /// from the ADC pin is then converted to a temperature value, using the
    /// `convert_to_celsius` method, which is also responsible for updating the
    /// cached temperature and resistance values.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let mut controller = HeaterController::new(adc, adc_pin, pwm, defer_channel, thermistor_properties);
    /// let current_temp = controller.read_temp().await;
    /// println!("Current Temperature: {} °C", current_temp);
    /// ```
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
        let sample_v = f32::from(sample) * self.v_ratio;
        let measured_resistance: f32 =
            (sample_v * self.r_pull_up) / ((4095.0f32 * self.v_ratio) - sample_v);

        /*
        hwa::trace!("sample: {:?} : v: {:?} Volt measured_resistance: {:?} Ohm",
            sample, Real::from_f32(sample_v), Real::from_f32(measured_resistance),
        );
        */

        let log_mr_by_r_nominal: f32 = micromath::F32::from(measured_resistance / self.r_nominal)
            .ln()
            .into();
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
        match &self.current_channel {
            CommChannel::Internal => false,
            #[cfg(any(
                feature = "with-serial-usb",
                feature = "with-serial-port-1",
                feature = "with-serial-port-2"
            ))]
            _ => true,
        }
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
        self.pwm.set_power(0);
    }

    /// Updates the state machine with the current temperature readings and
    /// performs the required actions based on the new temperature state.
    ///
    /// # Arguments
    ///
    /// * `ctrl` - A reference to the heater controller.
    ///
    /// This method carries out the following operations:
    ///
    /// 1. Locks the ADC to ensure exclusive access during the update process.
    /// 2. Reads the current temperature.
    /// 3. If the "native" feature is enabled, checks whether the heater has been on for more than 5 seconds and
    ///    adjusts the current temperature reading accordingly.
    /// 4. Determines the new state of the heater based on the current temperature and target temperature:
    ///    - If the heater is on, computes the control output using the PID controller and adjusts the power supplied to the heater.
    ///    - If the heater is off, resets the current temperature to zero and sets the state to `Duty`.
    /// 5. Updates the current state of the state machine if it has changed.
    /// 6. Publishes events to the event bus based on the new state.
    ///
    pub async fn update(&mut self, event_bus: &hwa::types::EventBus) {
        use num_traits::ToPrimitive;
        hwa::debug!("Updating HEATER...");

        let temp = self.read_temp().await;

        self.set_current_temp(temp);

        #[cfg(feature = "native")]
        {
            if self.is_on() {
                if self.t0.elapsed() > embassy_time::Duration::from_secs(5) {
                    self.state_machine.current_temperature = self.get_target_temp();
                }
            }
        }
        let new_state = {
            hwa::trace!(
                "MEASURED_TEMP[{:?}] {:?}",
                self.defer_action,
                Real::from_f32(self.state_machine.current_temperature)
            );
            if self.is_on() {
                let target_temp = self.get_target_temp();
                self.state_machine.pid.setpoint(target_temp);

                self.state_machine.last_temperature = self.state_machine.current_temperature;

                let delta = self
                    .state_machine
                    .pid
                    .next_control_output(self.state_machine.current_temperature)
                    .output;
                let power = if delta > 0.0f32 {
                    if delta < 100.0f32 {
                        delta / 100.0f32
                    } else {
                        1.0f32
                    }
                } else {
                    0.0f32
                };

                /*
                hwa::trace!(
                    "TEMP {:?} -> {:?}, {:?} P={:?} [{:?}]",
                    Real::from_f32(self.state_machine.last_temperature),
                    Real::from_f32(self.state_machine.current_temperature),
                    Real::from_f32(delta), Real::from_f32(power), Real::from_f32(target_temp),
                );
                */

                self.set_power((power * 100.0f32).to_u8().unwrap_or(0));

                if (self.state_machine.current_temperature - self.get_target_temp()).abs()
                    / target_temp
                    < 0.25
                {
                    State::Maintaining
                } else {
                    State::Targeting
                }
            } else {
                if self.state_machine.last_temperature != self.state_machine.current_temperature {
                    self.state_machine.last_temperature = self.state_machine.current_temperature;
                }
                State::Duty
            }
        };

        if new_state != self.state_machine.state {
            hwa::trace!("Temp changed to {:?}", new_state);
            match new_state {
                State::Duty => {
                    #[cfg(feature = "native")]
                    {
                        self.t0 = embassy_time::Instant::now();
                    }
                    event_bus
                        .publish_event(EventStatus::not_containing(self.temperature_flags))
                        .await;
                }
                State::Maintaining => {
                    #[cfg(feature = "native")]
                    hwa::debug!("Temperature reached. Firing {:?}.", self.temperature_flags);
                    // FIXME: proper order_num
                    self.flush_notification(self.defer_action, 0).await;
                    event_bus
                        .publish_event(EventStatus::containing(self.temperature_flags))
                        .await;
                }
                State::Targeting => {
                    #[cfg(feature = "native")]
                    {
                        self.t0 = embassy_time::Instant::now();
                    }
                    event_bus
                        .publish_event(EventStatus::not_containing(self.temperature_flags))
                        .await;
                }
            }
            self.state_machine.state = new_state;
        } else if self.is_awaited() {
            match new_state {
                State::Maintaining => {
                    // FIXME: proper order_num
                    self.flush_notification(self.defer_action, 0).await;
                }
                _ => {}
            }
        }
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
    /// PID controller used to regulate the heater's temperature.
    pid: pid::Pid<f32>,
    /// Current temperature being read from the heater.
    current_temperature: f32,
    /// The last recorded temperature of the heater.
    last_temperature: f32,
    /// The current state of the heater's finite state machine (FSM).
    state: State,
}

impl HeaterStateMachine {
    fn new() -> Self {
        let mut pid: pid::Pid<f32> = pid::Pid::new(0.0f32, 100.0f32);
        pid.p(5.0f32, 100.0f32);
        pid.i(0.01f32, 100.0f32);
        pid.d(1.5f32, 100.0f32);

        Self {
            pid,
            current_temperature: 0f32,
            last_temperature: 0f32,
            state: State::Duty,
        }
    }
}
