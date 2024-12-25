use printhor_hwa_common as hwa;
use embassy_time::Instant;
use crate::board::mocked_peripherals::MockedIOPin;

pub struct MockedAdc<PERI> {
    _peri: PERI,
    // For simulation: Time since expected read was required.
    time_since: Instant,
    // For simulation: Target expected read.
    expected_read: u16,
    default_sample: u16,
}
impl<PERI> MockedAdc<PERI> {
    pub(crate) fn new(_peri: PERI, default_sample: u16) -> Self {
        Self {
            _peri,
            time_since: Instant::now(),
            expected_read: 4000,
            default_sample,
        }
    }
}

impl<PERI> hwa::traits::UnifiedAdc16 for MockedAdc<PERI> {
    type VRefPin = ();
    type SamplePin = MockedIOPin;
    async fn read_vref(&mut self) -> Result<u16, ()>
    {
        Ok(self.default_sample)
    }

    async fn read_adc(&mut self, _pin: &mut Self::SamplePin) -> u16 {
        const EXPECTED_TIME: f32 = 10.0f32;
        let elapsed = self.time_since.elapsed().as_secs() as f32;
        // Add some noise as measurement * cos( w * t)
        // for instance: temp_deviation: measurement, w: frequency
        let noise = 5.0f32 * (0.1f32 * elapsed).cos();
        let mut simulated_measure = self.expected_read as f32;
        if elapsed < EXPECTED_TIME {
            // Simulated linear ramp to reach temp
            simulated_measure *= (elapsed / EXPECTED_TIME) as f32;
        }
        (5.0f32 + simulated_measure + noise) as u16
    }
}