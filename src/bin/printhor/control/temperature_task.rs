//! The Temperature controller task
//! TODO: Format
//! Currently, this is a very naive approach using a simple control pid,
//! but willing to improve with a formal method.
//!
//! AS-IS:
//!
//! --+--> power -> pid -> diff --+
//!   |                           |
//!   +---------------------------+
//!
//! TO-BE:
//!
//! 1. Mathematical model
//! The differential equation describing the temperature  $\(T(t)\)$ respecting the time $\(t\)$ is the following:
//!
//! $ \[C \frac{dT(t)}{dt} = P(t) - R \cdot [T(t) - T_{\text{amb}}]\] $
//!
//! To solve the differential equation and obtain an expression for \(T(t)\), we separate variables and integrate the first order differential equation.
//! Assuming initial conditions $\(T(0) = T_0\)$, The general solution:
//!
//! $ \[T(t) = T_{\text{amb}} + [T_0 - T_{\text{amb}}] \cdot e^{-\frac{t}{\tau}} + \frac{P_{\text{max}}}{R} \cdot \left(1 - e^{-\frac{t}{\tau}}\right)\] $
//!
//! Where:
//!
//! - \(T_{\text{amb}}\) is the ambient temperature.
//! - \(T_0\) is the initial temperature of the block.
//! - \(P_{\text{max}}\) is the maximum power of heating.
//! - \(\tau = \frac{C}{R}\) is the system time constant, which characterizes the speed of thermal response.
//!
//! The equation provides a description of how the temperature evolves \(T(t)\) with the time \(t\) in response of the heating power \(P(t)\)
//!
//! 2. Temperature sensor
//!  2.1. ADC mV Sampling with vref callibration if harware supports it
//!  2.2. Computation of thermistor resistive value (R_0) depending on circuit model.
//!  Asuming a rload (R_1) in serie with thermisthor (R_0) connected to ground:
//! V_{adc} = \frac{R_0}{R_0 + R_1}*V_{ref}
//! Then:
//! R_0 = \frac{V_{adc}*R_1}{V_{ref}-V_{adc}}
//!  2.3. With the measured resistive value and thermistor factors, convert to degrees leverating the Steinhart-Hart equation https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation:
//! \frac{1}{T}=\frac{1}{T_0}+\frac{1}{B}*\ln{\frac{R}{R_0}}
//! where:
//! T –> Target temperature in Kelvin degrees
//! T_0–> Nominal temperature of thermistor at 25ºC
//! B –> Beta factor of the thermistor
//! R –> Nominal resistance of the thermistor
//! R_0 –> Measured resistive value of thermisthor
//! 3. Control model
//! Control system uses a continuous feedback PID to adjust
//! the amount of heating supplied by a electric resistor to maintain the temperature of the heating block
//! The POD controller computes \(P(t)\) respecting to the error (\(e(t) = T_{\text{ref}} - T(t)\)),
//! The general equation of the PWM power control is:
//!
//! $\[P(t) = \text{PWM}(e(t)) \cdot P_{\text{max}}\]$
//!
//! where:
//!
//! - \(\text{PWM}(e(t))\) the function that converts the error in a PWM duty cycle (between 0 and 1).
//! - \(P_{\text{max}}\) the maximum power that electric resitor can supply.

use crate::hwa;
use embassy_time::{Duration, Ticker};
#[cfg(not(feature = "native"))]
use num_traits::float::FloatCore;
use num_traits::ToPrimitive;
use printhor_hwa_common::EventBusRef;
use printhor_hwa_common::EventStatus;
use printhor_hwa_common::EventFlags;

use pid;

#[cfg(feature = "with-defmt")]
use crate::hwa::defmt;

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
enum State {
    Duty,
    Targeting,
    Maintaining,
}

#[embassy_executor::task(pool_size=1)]
pub async fn temp_task(
    event_bus: EventBusRef,
    hotend_controller: hwa::controllers::HotendControllerRef
) -> ! {
    hwa::debug!("temperature_task started");

    let mut pid: pid::Pid<f32> = pid::Pid::new(0.0f32, 100.0f32);
    pid.p(5.0f32, 100.0f32);
    pid.i(0.01f32, 100.0f32);
    pid.d(1.5f32, 100.0f32);

    let mut ticker = Ticker::every(Duration::from_secs(2));
    let mut _t0 = embassy_time::Instant::now();

    let mut current_temp = 0f32;
    let mut last_temp = current_temp;

    hotend_controller.lock().await.init().await;

    let mut state = State::Duty;
    loop {
        ticker.next().await;
        let new_state = {
            let mut m = hotend_controller.lock().await;

            current_temp = m.read_temp().await;
            #[cfg(feature = "native")]
            if _t0.elapsed().as_secs() > 5 {
                current_temp = (m.get_target_temp()) as f32;
            }

            hwa::trace!("MEASURED_TEMP: {}", current_temp);

            if m.is_on() {
                let target_temp = m.get_target_temp() as f32;
                pid.setpoint(target_temp);

                last_temp = current_temp;

                let delta = pid.next_control_output(current_temp).output;
                m.set_current_temp(current_temp);
                let power = if delta > 0.0f32 {
                    if delta < 100.0f32 {
                        delta / 100.0f32
                    }
                    else {
                        1.0f32
                    }
                } else {
                    0.0f32
                };
                hwa::trace!("TEMP {} -> {}, {} P={} [{}]", last_temp, current_temp, delta, power, target_temp);

                m.set_power((power * 255.0f32).to_u8().unwrap_or(0)).await;

                if (current_temp - m.get_target_temp() as f32).abs() / target_temp < 0.25 {
                    State::Maintaining
                } else {
                    State::Targeting
                }
            } else {
                current_temp = 0.0;
                if last_temp != current_temp {
                    last_temp = current_temp;
                }
                State::Duty
            }
        };

        if new_state != state {
            hwa::trace!("Temp changed to {:?}", new_state);
            match new_state {
                State::Duty => {
                    event_bus.publish_event(EventStatus::not_containing(EventFlags::HOTEND_TEMP_OK)).await;
                }
                State::Maintaining => {
                    event_bus.publish_event(EventStatus::containing(EventFlags::HOTEND_TEMP_OK)).await;
                }
                State::Targeting => {
                    _t0 = embassy_time::Instant::now();
                    event_bus.publish_event(EventStatus::not_containing(EventFlags::HOTEND_TEMP_OK)).await;
                }
            }
            state = new_state;
        }
    }
}