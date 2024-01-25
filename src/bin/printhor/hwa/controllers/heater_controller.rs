use crate::hwa;
#[allow(unused)]
use printhor_hwa_common::{ControllerMutexType, ControllerRef};
use crate::hwa::controllers::pwm_controller::PwmController;
use crate::hwa::VREF_SAMPLE;

type AdcControllerRef<AdcPeri> = ControllerRef<crate::hwa::device::AdcImpl<AdcPeri>>;

pub struct HeaterController<AdcPeri, AdcPin, PwmHwaDevice>
    where
        AdcPeri: crate::hwa::device::AdcTrait + 'static,
        AdcPin: crate::hwa::device::AdcPinTrait<AdcPeri>,
        PwmHwaDevice: embedded_hal_02::Pwm<Duty=u16> + 'static,
        <PwmHwaDevice as embedded_hal_02::Pwm>::Channel: Copy

{
    adc: ControllerRef<crate::hwa::device::AdcImpl<AdcPeri>>,
    adc_pin: AdcPin,
    vref_sample: u16,
    pwm: PwmController<PwmHwaDevice>,
    target_temp: f32,
    current_temp: f32,
    on: bool,
}

#[allow(dead_code)]
impl<AdcPeri, AdcPin, PwmHwaDevice> HeaterController<AdcPeri, AdcPin, PwmHwaDevice>
where
    AdcPeri: crate::hwa::device::AdcTrait + 'static,
    AdcPin: crate::hwa::device::AdcPinTrait<AdcPeri>,
    PwmHwaDevice: embedded_hal_02::Pwm<Duty=u16> + 'static,
    <PwmHwaDevice as embedded_hal_02::Pwm>::Channel: Copy
{
    pub fn new(adc: AdcControllerRef<AdcPeri>, adc_pin: AdcPin, pwm: PwmController<PwmHwaDevice>) -> Self {
        Self {
            adc,
            adc_pin,
            vref_sample: VREF_SAMPLE,
            pwm,
            target_temp: 0.0f32,
            current_temp: 0.0f32,
            on: false,
        }
    }
    pub async fn init(&mut self) {
        // TODO: Rework on callibration
        hwa::debug!("Initializing ADC...");
        //let mut _bus  = self.adc.lock().await;
        //let mut bus = &mut (*_bus);

        //let mut vref_int = bus.enable_vrefint();
        //embassy_time::Timer::after(Duration::from_micros(embassy_stm32::adc::VrefInt::start_time_us() as u64)).await;
        //self.vref_sample = bus.read_internal(&mut vref_int);
        self.vref_sample = VREF_SAMPLE;
        hwa::debug!("ADC Initiallized: vref_sample = {}", self.vref_sample);
    }

    #[inline]
    pub async fn read_temp(&mut self) -> f32 {
        let mut bus  = self.adc.lock().await;
        let value = bus.read(&mut self.adc_pin);
        self.convert_to_celcius(value.into())
    }

    #[inline]
    pub fn get_target_temp(&self) -> f32 {
        self.target_temp
    }

    #[inline]
    pub fn set_target_temp(&mut self, target_temp: f32) {
        self.target_temp = target_temp;

    }

    #[inline]
    pub fn set_current_temp(&mut self, current_temp: f32) {
        self.current_temp = current_temp;
    }

    #[inline]
    pub fn get_current_temp(&mut self) -> f32 {
        self.current_temp
    }

    #[inline]
    pub async fn set_power(&mut self, power: u8) {
        self.pwm.set_power(power).await;
    }

    #[inline]
    pub async fn get_current_power(&mut self) -> f32 {
        self.pwm.get_power().await
    }

    /// Measures the temperature aplying the SteinHart-Hart equation
    /// Steps:
    /// 1 - Compute the resistance of the thermistor
    /// 2 - Compute the temperature applying the β parameter Steinhart–Hart equation
    fn convert_to_celcius(&self, sample: u16) -> f32 {
        const B: f32 = 3950.0; // B value of the thermistor
        const R0: f32 = 10000.0; // Nominal NTC Value
        const R1: f32 = 9850.0;

        let sample_mv =  self.convert_to_millivolts(sample);
        let measured_resistance: f32 = R1 / ((4096.0f32 / sample_mv as f32) - 1.0f32);
        let log_mr_by_r0: f32 = micromath::F32::from(measured_resistance / R0).ln().into();
        (1.0 / (log_mr_by_r0 / B + 1.0 / 298.15)) - 273.15
    }

    fn convert_to_millivolts(&self, sample: u16) -> u16 {
        // From http://www.st.com/resource/en/datasheet/DM00071990.pdf
        // 6.3.24 Reference voltage
        const VREFINT_MV: u32 = 1210; // Internal reference voltage (average)
        hwa::trace!("sample: {}", sample);
        ((u32::from(sample) * VREFINT_MV) / u32::from(self.vref_sample)) as u16
    }

    #[inline]
    pub fn is_on(&self) -> bool {
        self.on
    }
    #[inline]
    pub fn on(&mut self) {
        self.on = true;
    }
    #[inline]
    pub fn off(&mut self) {
        self.on = false;
    }
}
