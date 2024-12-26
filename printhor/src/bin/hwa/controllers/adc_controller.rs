//! TODO: This feature is still in incubation
use crate::hwa;
use hwa::math;
use hwa::traits::UnifiedAdc16;
use hwa::AsyncMutexStrategy;
use hwa::StaticAsyncController;

/// A controller for managing ADCs.
pub struct GenericAdcController<H>
where
    H: AsyncMutexStrategy + 'static,
    H::Resource: hwa::traits::UnifiedAdc16 + 'static,
{
    adc: StaticAsyncController<H>,
    adc_pin: <H::Resource as UnifiedAdc16>::SamplePin,
    default_sample: u16,
    v_ratio: f32,
}

impl<H> GenericAdcController<H>
where
    H: AsyncMutexStrategy + 'static,
    H::Resource: hwa::traits::UnifiedAdc16 + 'static,
{
    pub fn new(
        adc: StaticAsyncController<H>,
        adc_pin: <H::Resource as UnifiedAdc16>::SamplePin,
        default_sample: u16,
    ) -> Self {
        Self {
            adc,
            adc_pin,
            default_sample,
            v_ratio: 1.0f32,
        }
    }

    pub async fn init(&mut self, v_ref_default: u16) {
        hwa::debug!("Initializing ADC...");

        let mut adc_mg = self.adc.lock().await;
        let sample = adc_mg.read_vref().await.unwrap_or(self.default_sample);

        self.v_ratio = f32::from(v_ref_default) / (f32::from(sample) * 1000.0f32);
        hwa::info!(
            "ADC Initialized: ref_int_default = {:?}, vref_sample = {:?} | v_ratio = {:?} volts/adc_unit",
            v_ref_default,
            sample,
            math::Real::from_f32(self.v_ratio),
        );
    }

    pub async fn read_sample(&mut self) -> u16 {
        let mut adc_mg = self.adc.lock().await;
        adc_mg.read_adc(&mut self.adc_pin).await
    }
}
