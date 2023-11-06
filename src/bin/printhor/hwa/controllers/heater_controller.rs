use crate::hwa;
use printhor_hwa_common::ControllerRef;
use crate::hwa::controllers::pwm_controller::PwmController;
use crate::hwa::VREF_SAMPLE;

use crate::hwa::devices::AdcImpl;
use crate::hwa::devices::AdcPinTrait;
use crate::hwa::devices::AdcTrait;

type AdcControllerRef<AdcPeri> = ControllerRef<AdcImpl<AdcPeri>>;

#[allow(dead_code)]
pub struct HeaterController<AdcPeri, AdcPin, PwmHwaDevice>
    where
        AdcPeri: AdcTrait + 'static,
        AdcPin: AdcPinTrait<AdcPeri>,
        PwmHwaDevice: embedded_hal::Pwm + 'static,
        <PwmHwaDevice as embedded_hal::Pwm>::Channel: Copy

{
    adc: ControllerRef<AdcImpl<AdcPeri>>,
    adc_pin: AdcPin,
    vref_sample: u16,
    pwm: PwmController<PwmHwaDevice>,
}

#[allow(dead_code)]
impl<AdcPeri, AdcPin, PwmHwaDevice> HeaterController<AdcPeri, AdcPin, PwmHwaDevice>
where
    AdcPeri: AdcTrait + 'static,
    AdcPin: AdcPinTrait<AdcPeri>,
    PwmHwaDevice: embedded_hal::Pwm + 'static,
    <PwmHwaDevice as embedded_hal::Pwm>::Channel: Copy
{
    pub fn new(adc: AdcControllerRef<AdcPeri>, adc_pin: AdcPin, pwm: PwmController<PwmHwaDevice>) -> Self {
        Self {
            adc,
            adc_pin,
            vref_sample: VREF_SAMPLE,
            pwm,
        }
    }
    pub async fn init(&mut self) {
        // TODO: Rework on callibration
        hwa::info!("Initializing ADC...");
        //let mut _bus  = self.adc.lock().await;
        //let mut bus = &mut (*_bus);

        //let mut vref_int = bus.enable_vrefint();
        //embassy_time::Timer::after(Duration::from_micros(embassy_stm32::adc::VrefInt::start_time_us() as u64)).await;
        //self.vref_sample = bus.read_internal(&mut vref_int);
        self.vref_sample = VREF_SAMPLE;
        hwa::info!("ADC Initiallized: vref_sample = {}", self.vref_sample);
    }

    pub async fn read_temp(&mut self) -> f32 {
        let mut bus  = self.adc.lock().await;
        bus.read(&mut self.adc_pin);
        0.0f32
    }
    pub fn get_target_temp(&self) -> f32 {
        0.0f32
    }

    pub async fn set_target_temp(&mut self, _target_temp: f32) {

    }

    pub fn set_current_temp(&mut self, _target_temp: f32) {
    }

    pub fn get_current_temp(&mut self) -> f32 {
        0.0f32
    }

    pub async fn set_power(&mut self, _power: f32) {
        self.pwm.set_power(_power).await;
    }

    #[inline]
    pub fn is_on(&self) -> bool {
        self.pwm.is_on()
    }
}
