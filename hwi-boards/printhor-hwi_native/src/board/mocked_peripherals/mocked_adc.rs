use crate::board::mocked_peripherals::InputPin;

pub(crate) struct MockedTemperatureAdc {
    #[allow(unused)]
    p: InputPin,
}
impl MockedTemperatureAdc {
    pub(crate) const fn new() -> Self {
        Self {
            p: InputPin::new(),
        }
    }
    #[allow(unused)]
    pub(crate) fn read<T>(&self, _pin: &T) -> u16 {
        1024
    }

    #[allow(unused)]
    pub(crate) async fn test(&self) {
        crate::info!("adc test ok");
    }

}