//! Mostly functional

use crate::hwa;
use crate::hwa::controllers::pwm_controller::PwmController;
#[allow(unused)]
use crate::math::Real;
#[allow(unused)]
use crate::tgeo::ArithmeticOps;
use hwa::DeferEvent::{AwaitRequested, Completed};
use hwa::{CommChannel, DeferAction};

/// A controller struct for managing a heater device.
///
/// This controller interacts with both an ADC (Analog-Digital Converter)
/// and a PWM (Pulse-Width Modulation) controller to measure and regulate the temperature.
///
/// # Generics
/// * `AdcPeri`: Represents the ADC peripheral, it must implement `AdcTrait`.
/// * `AdcPin`: Represents the specific pin used by the ADC, it must implement `AdcPinTrait`.
/// * `PwmHwaDevice`: Represents the PWM device, it must implement `embedded_hal_02::Pwm` with `u32` type for `Duty`.
///
/// # Fields
/// * `adc`: A reference to the ADC controller used for temperature measurement.
/// * `adc_pin`: The pin used by the ADC controller for reading values.
/// * `v_ratio`: The precomputed ratio of volts per ADC unit for voltage to temperature conversion.
/// * `pwm`: A PWM controller used to apply heating power.
/// * `defer_channel`: A channel for submitting status changes for deferred processing.
/// * `target_temp`: The target or desired temperature value in Celsius.
/// * `current_temp`: The last measured temperature value in Celsius, cached for quick access.
/// * `current_resistance`: The last measured resistance value in Ohms, cached for quick access.
/// * `commander_channel`: The communication channel that sent the current request.
/// * `on`: A boolean indicating whether the heater is currently on.
/// * `thermistor_properties`: Properties of the thermistor including its resistance and beta coefficient.
pub struct HeaterController<MutexAdc, MutexPwm, AdcPeri, AdcPin, PwmHwaDevice>
where
    MutexAdc: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    MutexPwm: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    AdcPeri: hwa::device::AdcTrait + 'static,
    AdcPin: hwa::device::AdcPinTrait<AdcPeri>,
    PwmHwaDevice: embedded_hal_02::Pwm<Duty = u32> + 'static,
    <PwmHwaDevice as embedded_hal_02::Pwm>::Channel: Copy,
{
    /// Shared Analog-Digital Converter (ADC) controller used to measure temperature.
    adc: hwa::StaticController<MutexAdc, hwa::device::AdcImpl<AdcPeri>>,
    /// The specific pin used by the ADC.
    adc_pin: AdcPin,
    /// Precomputed ratio of volts per ADC unit, used for voltage to temperature conversion.
    v_ratio: f32,
    /// Shared Pulse-Width Modulation (PWM) controller used to apply heating power.
    pwm: PwmController<MutexPwm, PwmHwaDevice>,
    /// The defer channel to submit status changes for deferred processing.
    defer_channel: hwa::DeferChannel<hwa::DeferChannelMutexType>,
    /// The target or expected temperature value in Celsius.
    target_temp: f32,
    /// Last measured temperature value in Celsius, cached for quick access.
    current_temp: f32,
    /// Last measured resistance value in Ohms, cached for quick access.
    current_resistance: f32,
    /// The communication channel that commanded the current request.
    commander_channel: CommChannel,
    /// Cache of the heater controller's operational state (on or off).
    on: bool,
    /// The properties of the thermistor being used, including its resistance and beta coefficient.
    thermistor_properties: &'static hwa::ThermistorProperties,
}
#[allow(dead_code)]
impl<MutexAdc, MutexPwm, AdcPeri, AdcPin, PwmHwaDevice> HeaterController<MutexAdc, MutexPwm, AdcPeri, AdcPin, PwmHwaDevice>
where
    MutexAdc: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    MutexPwm: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    AdcPeri: hwa::device::AdcTrait + 'static,
    AdcPin: hwa::device::AdcPinTrait<AdcPeri>,
    PwmHwaDevice: embedded_hal_02::Pwm<Duty = u32> + 'static,
    <PwmHwaDevice as embedded_hal_02::Pwm>::Channel: Copy,
    hwa::device::VrefInt: hwa::device::AdcPinTrait<AdcPeri>,
{
    pub fn new(
        adc: hwa::StaticController<MutexAdc, hwa::device::AdcImpl<AdcPeri>>,
        adc_pin: AdcPin,
        pwm: PwmController<MutexPwm, PwmHwaDevice>,
        defer_channel: hwa::DeferChannel<hwa::DeferChannelMutexType>,
        thermistor_properties: &'static hwa::ThermistorProperties,
    ) -> Self {
        Self {
            adc,
            adc_pin,
            v_ratio: 1.0f32,
            pwm,
            target_temp: 0.0f32,
            current_temp: 0.0f32,
            current_resistance: 0.0f32,
            defer_channel,
            commander_channel: CommChannel::Internal,
            on: false,
            thermistor_properties,
        }
    }
    pub async fn init(&mut self) {
        hwa::debug!("Initializing ADC...");
        cfg_if::cfg_if! {
            if #[cfg(feature="without-vref-int")] {
                let vref_default = f32::from(hwa::ADC_VREF_DEFAULT_MV);
                let vref_sample = f32::from(hwa::ADC_VREF_DEFAULT_SAMPLE);
            }
            else {
                let mut bus  = self.adc.lock().await;
                cfg_if::cfg_if! {
                    if #[cfg(feature="enable_vrefint-with-delay")] {
                        let mut vref_int = bus.enable_vrefint();
                    }
                    else {
                        let mut vref_int = bus.enable_vrefint();
                        embassy_time::Timer::after(embassy_time::Duration::from_micros(hwa::ADC_START_TIME_US as u64)).await;
                    }
                }
                let vref_default = f32::from(hwa::ADC_VREF_DEFAULT_MV);
                let vref_sample = f32::from(bus.read(&mut vref_int));
            }
        }
        self.v_ratio = vref_default / (vref_sample * 1000.0f32);

        hwa::info!(
            "ADC Initialized: refint_default = {}, vref_sample = {} | v_ratio = {} volts/adc_unit",
            Real::from_f32(vref_default),
            Real::from_f32(vref_sample),
            Real::from_f32(self.v_ratio),
        );
    }

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
        let mut bus = self.adc.lock().await;
        cfg_if::cfg_if! {
            if #[cfg(feature="adc-is-async")] {
                let value = bus.read(&mut self.adc_pin).await;
            }
            else {
                let value = bus.read(&mut self.adc_pin);
            }
        }
        (self.current_temp, self.current_resistance) = self.convert_to_celcius(value.into());
        self.current_temp
    }

    #[inline]
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
    ) -> bool {
        if requested_temp > 0.0f32 {
            let tdiff = requested_temp - self.target_temp;
            self.target_temp = requested_temp;
            self.on();
            if tdiff.abs() > 0.1f32 * self.target_temp {
                self.commander_channel = channel;
                self.defer_channel
                    .send(AwaitRequested(action, channel))
                    .await;
                true
            } else {
                self.commander_channel = CommChannel::Internal;
                false
            }
        } else {
            self.target_temp = 0.0f32;
            self.commander_channel = CommChannel::Internal;
            self.off().await;
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
    pub async fn ping_subscribe(&mut self, channel: CommChannel, action: DeferAction) -> bool {
        if self.is_on() && self.current_temp > 0.0f32 {
            let tdiff = self.current_temp - self.target_temp;
            if tdiff.abs() > 0.1f32 * self.target_temp {
                self.commander_channel = channel;
                self.defer_channel
                    .send(AwaitRequested(action, channel))
                    .await;
                true
            } else {
                self.commander_channel = CommChannel::Internal;
                false
            }
        } else {
            false
        }
    }

    // Updates the latest measured temperature in Celsius degrees
    #[inline]
    pub fn set_current_temp(&mut self, current_temp: f32) {
        self.current_temp = current_temp;
    }

    // Gets the latest measured temperature in Celsius degrees
    #[inline]
    pub fn get_current_temp(&mut self) -> f32 {
        self.current_temp
    }

    // Gets the latest measured resistance in Ohms
    #[inline]
    pub fn get_current_resistance(&mut self) -> f32 {
        self.current_resistance
    }

    // Sets the applied power in scale between 0 and 100
    #[inline]
    pub async fn set_power(&mut self, power: u8) {
        self.pwm.set_power(power).await;
    }

    // Gets the applied power in scale between 0.0 and 1.0
    #[inline]
    pub async fn get_current_power(&mut self) -> f32 {
        self.pwm.get_power().await
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
        let measured_resistance: f32 = (sample_v * self.thermistor_properties.r_pullup)
            / ((4095.0f32 * self.v_ratio) - sample_v);

        hwa::debug!(
            "sample: {} : v: {} Volt measured_resistance: {} Ohm",
            sample,
            Real::from_f32(sample_v),
            Real::from_f32(measured_resistance),
        );

        let log_mr_by_r_nominal: f32 =
            micromath::F32::from(measured_resistance / self.thermistor_properties.r_nominal)
                .ln()
                .into();
        (
            (1.0 / (log_mr_by_r_nominal / self.thermistor_properties.beta + 1.0 / 298.15)) - 273.15,
            measured_resistance,
        )
    }

    // Sends and consume the deferred action
    pub async fn flush_notification(&mut self, action: DeferAction) {
        self.defer_channel
            .send(Completed(action, self.commander_channel))
            .await;
        self.commander_channel = CommChannel::Internal;
    }

    pub fn is_awaited(&self) -> bool {
        match &self.commander_channel {
            CommChannel::Internal => false,
            _ => true,
        }
    }

    // Checks if heater is enabled
    #[inline]
    pub fn is_on(&self) -> bool {
        self.on
    }
    #[inline]
    pub fn on(&mut self) {
        self.on = true;
    }
    #[inline]
    pub async fn off(&mut self) {
        self.on = false;
        self.pwm.set_power(0).await;
    }
}
