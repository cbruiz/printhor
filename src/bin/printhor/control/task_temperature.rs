//! The Temperature controller task
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
use crate::hwa;
use crate::hwa::controllers::HeaterController;
use embassy_time::{Duration, Ticker};
use hwa::{DeferAction, EventBusRef};
use hwa::{EventFlags, EventStatus};
#[cfg(not(feature = "native"))]
use num_traits::float::FloatCore;
use num_traits::ToPrimitive;

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
    current_temp: f32,
    /// The last recorded temperature of the heater.
    last_temp: f32,
    /// The current state of the heater's finite state machine (FSM).
    state: State,
    /// The time instant when the FSM was started, available only if the "native" feature is enabled.
    #[cfg(feature = "native")]
    t0: embassy_time::Instant,
}

impl HeaterStateMachine {
    fn new() -> Self {
        let mut pid: pid::Pid<f32> = pid::Pid::new(0.0f32, 100.0f32);
        pid.p(5.0f32, 100.0f32);
        pid.i(0.01f32, 100.0f32);
        pid.d(1.5f32, 100.0f32);

        Self {
            #[cfg(feature = "native")]
            t0: embassy_time::Instant::now(),
            pid,
            current_temp: 0f32,
            last_temp: 0f32,
            state: State::Duty,
        }
    }

    /// Updates the state machine with the current temperature readings and
    /// performs the required actions based on the new temperature state.
    ///
    /// # Arguments
    ///
    /// * `ctrl` - A reference to the heater controller.
    /// * `event_bus` - A reference to the event bus.
    /// * `temperature_flag` - The event flag indicating temperature-related events.
    /// * `action` - The defer action to be executed.
    ///
    /// # Type Parameters
    ///
    /// * `AdcPeri` - The ADC peripheral type.
    /// * `AdcPin` - The ADC pin type.
    /// * `PwmHwaDevice` - The PWM device type.
    ///
    /// This method carries out the following operations:
    ///
    /// 1. Locks the heater controller to ensure exclusive access during the update process.
    /// 2. Reads the current temperature from the heater.
    /// 3. If the "native" feature is enabled, checks whether the heater has been on for more than 5 seconds and
    ///    adjusts the current temperature reading accordingly.
    /// 4. Determines the new state of the heater based on the current temperature and target temperature:
    ///    - If the heater is on, computes the control output using the PID controller and adjusts the power supplied to the heater.
    ///    - If the heater is off, resets the current temperature to zero and sets the state to `Duty`.
    /// 5. Updates the current state of the state machine if it has changed.
    /// 6. Publishes events to the event bus based on the new state.
    ///
    /// This method requires the `AdcPeri`, `AdcPin`, and `PwmHwaDevice` types to implement specific traits as shown in the where clause.
    async fn update<AdcPeri, AdcPin, PwmHwaDevice>(
        &mut self,
        ctrl: &hwa::InterruptControllerRef<HeaterController<AdcPeri, AdcPin, PwmHwaDevice>>,
        event_bus: &EventBusRef,
        temperature_flag: EventFlags,
        action: DeferAction,
    ) where
        AdcPeri: hwa::device::AdcTrait + 'static,
        AdcPin: hwa::device::AdcPinTrait<AdcPeri>,
        PwmHwaDevice: embedded_hal_02::Pwm<Duty = u32> + 'static,
        <PwmHwaDevice as embedded_hal_02::Pwm>::Channel: Copy,
        crate::hwa::device::VrefInt: crate::hwa::device::AdcPinTrait<AdcPeri>,
    {
        let mut m = ctrl.lock().await;

        self.current_temp = m.read_temp().await;
        #[cfg(feature = "native")]
        {
            if m.is_on() {
                if self.t0.elapsed() > Duration::from_secs(5) {
                    self.current_temp = m.get_target_temp();
                }
            }
        }
        let new_state = {
            hwa::debug!("MEASURED_TEMP[{:?}] {}", action, self.current_temp);
            if m.is_on() {
                let target_temp = m.get_target_temp();
                self.pid.setpoint(target_temp);

                self.last_temp = self.current_temp;

                let delta = self.pid.next_control_output(self.current_temp).output;
                m.set_current_temp(self.current_temp);
                let power = if delta > 0.0f32 {
                    if delta < 100.0f32 {
                        delta / 100.0f32
                    } else {
                        1.0f32
                    }
                } else {
                    0.0f32
                };
                hwa::debug!(
                    "TEMP {} -> {}, {} P={} [{}]",
                    self.last_temp,
                    self.current_temp,
                    delta,
                    power,
                    target_temp
                );

                m.set_power((power * 100.0f32).to_u8().unwrap_or(0)).await;

                if (self.current_temp - m.get_target_temp()).abs() / target_temp < 0.25 {
                    State::Maintaining
                } else {
                    State::Targeting
                }
            } else {
                self.current_temp = 0.0;
                if self.last_temp != self.current_temp {
                    self.last_temp = self.current_temp;
                }
                State::Duty
            }
        };

        if new_state != self.state {
            hwa::trace!("Temp changed to {:?}", new_state);
            match new_state {
                State::Duty => {
                    #[cfg(feature = "native")]
                    {
                        self.t0 = embassy_time::Instant::now();
                    }
                    event_bus
                        .publish_event(EventStatus::not_containing(temperature_flag))
                        .await;
                }
                State::Maintaining => {
                    #[cfg(feature = "native")]
                    hwa::debug!("Temperature reached. Firing {:?}.", temperature_flag);
                    m.flush_notification(action).await;
                    event_bus
                        .publish_event(EventStatus::containing(temperature_flag))
                        .await;
                }
                State::Targeting => {
                    #[cfg(feature = "native")]
                    {
                        self.t0 = embassy_time::Instant::now();
                    }
                    event_bus
                        .publish_event(EventStatus::not_containing(temperature_flag))
                        .await;
                }
            }
            self.state = new_state;
        } else if m.is_awaited() {
            match new_state {
                State::Maintaining => {
                    m.flush_notification(action).await;
                }
                _ => {}
            }
        }
    }
}

///
/// # Asynchronous Task Entry Point
///
/// This function serves as the entry point for the temperature task that runs asynchronously.
///
/// # Arguments
///
/// * `event_bus` - A reference to the event bus.
///
/// `#[cfg(feature = "with-hot-end")]` - If enabled, receives a reference to the hot end controller.
///
/// `#[cfg(feature = "with-hot-bed")]` - If enabled, receives a reference to the hot bed controller.
///
/// # Description
///
/// This method initializes and runs an infinite loop where the state machines for the hot end and hot bed (if enabled) are updated.
/// Within the loop:
///
/// 1. It waits for an interval of 2 seconds.
/// 2. It calls the `update` method of the heater state machines (hot end and hot bed) if they are enabled, providing references to their respective controllers and the event bus.
///
/// Note that this function uses async/await to coordinate asynchronous operations.
#[embassy_executor::task(pool_size = 1)]
pub async fn task_temperature(
    event_bus: EventBusRef,
    #[cfg(feature = "with-hot-end")] hotend_controller: hwa::controllers::HotendControllerRef,
    #[cfg(feature = "with-hot-bed")] hotbed_controller: hwa::controllers::HotbedControllerRef,
) {
    hwa::info!("[task_temperature] Started");

    let mut ticker = Ticker::every(Duration::from_secs(2));

    #[cfg(feature = "with-hot-end")]
    let mut hotend_sm = HeaterStateMachine::new();
    #[cfg(feature = "with-hot-bed")]
    let mut hotbed_sm = HeaterStateMachine::new();

    loop {
        // TODO: Park on SYS_ALARM
        ticker.next().await;

        #[cfg(test)]
        if crate::control::task_integration::INTEGRATION_STATUS.signaled() {
            hwa::info!("[task_temperature] Ending gracefully");
            return ();
        }

        #[cfg(feature = "with-hot-end")]
        hotend_sm
            .update(
                &hotend_controller,
                &event_bus,
                EventFlags::HOT_END_TEMP_OK,
                DeferAction::HotEndTemperature,
            )
            .await;
        #[cfg(feature = "with-hot-bed")]
        hotbed_sm
            .update(
                &hotbed_controller,
                &event_bus,
                EventFlags::HOT_BED_TEMP_OK,
                DeferAction::HotbedTemperature,
            )
            .await;
    }
}
