//! TODO: This feature is still in incubation

use crate::hwa;
use hwa::MutexStrategy;
use hwa::StaticController;

/// A controller for managing ADCs.
pub struct AdcController<H, P>
where
    H: MutexStrategy + Send + 'static,
    H::Resource: Send + 'static,
{
    adc: StaticController<H>,
    adc_pin: P,
}

impl<H, P> AdcController<H, P>
where
    H: MutexStrategy + Send + 'static,
    H::Resource: Send + 'static,
{
    pub fn new(adc: StaticController<H>, adc_pin: P) -> Self {
        Self { adc, adc_pin }
    }

    pub async fn init(&mut self) {
        hwa::debug!("Initializing ADC...");
        let adc_peri = self.adc.lock().await;
        hwa::info!(
            "ADC Initialized: refint_default = {}, vref_sample = {} | v_ratio = {} volts/adc_unit",
            0, //Real::from_f32(vref_default),
            0, // Real::from_f32(vref_sample),
            0, //Real::from_f32(self.v_ratio),
        );
        /*
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
                        // TODO
                        //embassy_time::Timer::after(embassy_time::Duration::from_micros(hwa::ADC_START_TIME_US as u64)).await;
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

         */
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
    pub async fn read_temp(&self) -> f32 {
        let mut mg = self.adc.lock().await;
        use core::ops::DerefMut;
        let _peri = mg.deref_mut();
        hwa::info!("");
        //let value = _peri.read(&mut self.adc_pin).await;

        cfg_if::cfg_if! {
            if #[cfg(feature="adc-is-async")] {
                let value = mg.read(&mut self.adc_pin).await;
            }
            else {

                //let value = mg.read(&mut self.adc_pin);
                let value = 0.0f32;
            }
        }
        0.0f32
        /*
        (self.current_temp, self.current_resistance) = self.convert_to_celcius(value.into());
        self.current_temp

         */
    }
}
