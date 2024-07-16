//! Mostly functional
use crate::hwa;
use printhor_hwa_common::{CommChannel, DeferAction, DeferChannelRef};
use printhor_hwa_common::DeferEvent::{AwaitRequested, Completed};
use crate::hwa::controllers::pwm_controller::PwmController;
#[allow(unused)]
use crate::tgeo::ArithmeticOps;

type AdcControllerRef<AdcPeri> = hwa::InterruptControllerRef<crate::hwa::device::AdcImpl<AdcPeri>>;

pub struct HeaterController<AdcPeri, AdcPin, PwmHwaDevice>
    where
        AdcPeri: crate::hwa::device::AdcTrait + 'static,
        AdcPin: crate::hwa::device::AdcPinTrait<AdcPeri>,
        PwmHwaDevice: embedded_hal_02::Pwm<Duty=u32> + 'static,
        <PwmHwaDevice as embedded_hal_02::Pwm>::Channel: Copy

{
    /// Shared Analog-Digital Converter controller to measure temperature
    adc: hwa::InterruptControllerRef<crate::hwa::device::AdcImpl<AdcPeri>>,
    adc_pin: AdcPin,
    /// Precomputed ratio of volts per ADC unit
    v_ratio: f32,
    /// Shared PWM controller to apply heating power
    pwm: PwmController<PwmHwaDevice>,
    /// The defer channel to submit status changes to
    defer_channel: DeferChannelRef,
    /// The target/expected temperature value (in Celsius)
    target_temp: f32,
    /// Cache of last temperature value measure (in Celsius)
    current_temp: f32,
    /// Cache of last resistance value measure (in Ohms)
    current_resistance: f32,
    /// The comm channel who commanded the request.
    commander_channel: CommChannel,
    /// Cache of controller state
    on: bool,
    /// The thermistor properties
    thermistor_properties: &'static hwa::ThermistorProperties,
}
#[allow(dead_code)]
impl<AdcPeri, AdcPin, PwmHwaDevice> HeaterController<AdcPeri, AdcPin, PwmHwaDevice>
where
    AdcPeri: crate::hwa::device::AdcTrait + 'static,
    AdcPin: crate::hwa::device::AdcPinTrait<AdcPeri>,
    PwmHwaDevice: embedded_hal_02::Pwm<Duty=u32> + 'static,
    <PwmHwaDevice as embedded_hal_02::Pwm>::Channel: Copy,
    crate::hwa::device::VrefInt: crate::hwa::device::AdcPinTrait<AdcPeri>
{
    pub fn new(adc: AdcControllerRef<AdcPeri>, adc_pin: AdcPin,
               pwm: PwmController<PwmHwaDevice>,
               defer_channel: DeferChannelRef,
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
                        let vref_int = bus.enable_vrefint();
                    }
                    else {
                        let mut vref_int = bus.enable_vrefint();
                        embassy_time::Timer::after(embassy_time::Duration::from_micros(hwa::ADC_START_TIME_US as u64)).await;
                    }
                }
                let vref_default = f32::from(hwa::ADC_VREF_DEFAULT_MV);
                let vref_sample = f32::from(bus.blocking_read(&mut vref_int));
            }
        }
        self.v_ratio = vref_default / (vref_sample * 1000.0f32);
        hwa::info!("ADC Initialized: refint_default = {}, vref_sample = {} | v_ratio = {} volts/adc_unit", vref_default, vref_sample, self.v_ratio);
    }

    // Read temperature in celsius degrees
    #[inline]
    pub async fn read_temp(&mut self) -> f32 {
        let mut bus  = self.adc.lock().await;
        cfg_if::cfg_if! {
            if #[cfg(feature="adc-is-async")] {
                let value = bus.read(&mut self.adc_pin).await;
            }
            else {
                let value = bus.blocking_read(&mut self.adc_pin);
            }
        }
        (self.current_temp, self.current_resistance) = self.convert_to_celcius(value.into());
        self.current_temp
    }

    #[inline]
    pub fn get_target_temp(&self) -> f32 {
        self.target_temp
    }

    /// Returns if true when command is deferred (needs to delay ACK to reach the temperature).
    /// Automatically triggers a defer event
    pub async fn set_target_temp(&mut self, channel: CommChannel, action: DeferAction, requested_temp: f32) -> bool {

        if requested_temp > 0.0f32 {
            let tdiff = requested_temp - self.target_temp;
            self.target_temp = requested_temp;
            self.on();
            if tdiff.abs() > 0.1f32 * self.target_temp {
                self.commander_channel = channel;
                self.defer_channel.send(AwaitRequested(action, channel)).await;
                true
            }
            else {
                self.commander_channel = CommChannel::Internal;
                false
            }
        }
        else {
            self.target_temp = 0.0f32;
            self.commander_channel = CommChannel::Internal;
            self.off().await;
            false
        }
    }

    /// Check temperature is OK
    /// Returns if true when command is deferred (needs to delay ACK to reach the temperature).
    /// Automatically triggers a defer event
    pub async fn ping_subscribe(&mut self, channel: CommChannel, action: DeferAction) -> bool {
        if self.is_on() && self.current_temp > 0.0f32 {
            let tdiff = self.current_temp - self.target_temp;
            if tdiff.abs() > 0.1f32 * self.target_temp {
                self.commander_channel = channel;
                self.defer_channel.send(AwaitRequested(action, channel)).await;
                true
            }
            else {
                self.commander_channel = CommChannel::Internal;
                false
            }
        }
        else {
            false
        }
    }

    // Updates the latest measured temperature in celsius degrees
    #[inline]
    pub fn set_current_temp(&mut self, current_temp: f32) {
        self.current_temp = current_temp;
    }

    // Gets the latest measured temperature in celsius degrees
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
        let measured_resistance: f32 = (sample_v * self.thermistor_properties.r_pullup) / (( 4095.0f32 * self.v_ratio) - sample_v);
        hwa::debug!("sample: {} : v: {} Volt measured_resistance: {} Ohm", sample, sample_v, measured_resistance);
        let log_mr_by_r_nominal: f32 = micromath::F32::from(measured_resistance / self.thermistor_properties.r_nominal).ln().into();
        (
            (1.0 / (log_mr_by_r_nominal / self.thermistor_properties.beta + 1.0 / 298.15)) - 273.15,
            measured_resistance,
        )
    }

    // Sends and consume the deferred action
    pub async fn flush_notification(&mut self, action: DeferAction) {
        self.defer_channel.send(Completed(action, self.commander_channel)).await;
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
