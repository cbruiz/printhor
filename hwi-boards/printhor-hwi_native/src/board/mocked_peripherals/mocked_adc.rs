use printhor_hwa_common as hwa;
use crate::board::mocked_peripherals::MockedIOPin;
use crate::simulation::{self, Heater, Thermistor};

/// A 12-bit ADC whose pins read the thermistors of the simulated heaters (see [crate::simulation])
pub struct MockedAdc<PERI> {
    _peri: PERI,
    default_sample: u16,
}
impl<PERI> MockedAdc<PERI> {
    pub(crate) fn new(_peri: PERI, default_sample: u16) -> Self {
        Self {
            _peri,
            default_sample,
        }
    }
}

/// The thermistor of each heater, as configured in the board contract
#[allow(unused)]
fn thermistor(heater: Heater) -> Option<Thermistor> {
    use hwa::HwiContract;
    type C = crate::board::Contract;
    match heater {
        #[cfg(feature = "with-hot-end")]
        Heater::HotEnd => Some(Thermistor {
            beta: <C as HwiContract>::HOT_END_THERM_BETA,
            nominal_resistance: <C as HwiContract>::HOT_END_THERM_NOMINAL_RESISTANCE,
            pull_up_resistance: <C as HwiContract>::HOT_END_THERM_PULL_UP_RESISTANCE,
        }),
        #[cfg(feature = "with-hot-bed")]
        Heater::HotBed => Some(Thermistor {
            beta: <C as HwiContract>::HOT_BED_THERM_BETA,
            nominal_resistance: <C as HwiContract>::HOT_BED_THERM_NOMINAL_RESISTANCE,
            pull_up_resistance: <C as HwiContract>::HOT_BED_THERM_PULL_UP_RESISTANCE,
        }),
        #[allow(unreachable_patterns)]
        _ => None,
    }
}

impl<PERI> hwa::traits::UnifiedAdc16 for MockedAdc<PERI> {
    type VRefPin = ();
    type SamplePin = MockedIOPin;
    async fn read_vref(&mut self) -> Result<u16, ()>
    {
        Ok(self.default_sample)
    }

    async fn read_adc(&mut self, pin: &mut Self::SamplePin) -> u16 {
        match simulation::thermistor_temperature(pin.id()) {
            Some((heater, temp)) => thermistor(heater).map(|t| t.sample(temp)).unwrap_or(0),
            // Nothing connected: floating input
            None => 0,
        }
    }
}
