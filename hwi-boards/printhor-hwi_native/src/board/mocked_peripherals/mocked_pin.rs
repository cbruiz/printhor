use std::marker::PhantomData;
use embassy_time::{Duration, Timer};
use crate::device::AdcPinTrait;

pub struct MockedOutputPin<'a, T> {
    p: PhantomData<&'a T>
}

#[allow(unused)]
impl<'a, T> MockedOutputPin<'_, T> {
    pub(crate) const fn new() -> Self {
        Self { p: PhantomData }
    }

    pub fn set_high(&mut self) {

    }

    pub fn set_low(&mut self) {

    }

    pub fn is_set_high(&mut self) -> bool {
        false
    }

    pub fn is_set_low(&mut self) -> bool {
        false
    }
}

pub struct MockedInputPin<'a, T> {
    p: PhantomData<&'a T>
}
impl<'a, T> MockedInputPin<'a, T> {
    #[allow(unused)]
    pub(crate) const fn new() -> Self {
        Self {
            p: PhantomData,
        }
    }

    #[allow(unused)]
    pub fn is_low(&self) -> bool {
        false
    }

    #[allow(unused)]
    pub fn is_high(&self) -> bool {
        false
    }

    #[allow(unused)]
    pub async fn wait_for_any_edge<'b>(&'b mut self) {
        //println!("wait_edge");
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
impl<'a, T> AdcPinTrait<crate::board::mocked_peripherals::MockedAdc<T>> for MockedInputPin<'a, T> {

}

#[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
impl<'a, T> AdcPinTrait<crate::board::mocked_peripherals::MockedAdc<T>> for u8 {

}

#[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
impl AdcPinTrait<u8> for u8 {

}

#[cfg(any(feature = "with-hotend", feature = "with-hotbed"))]
impl<'a, T> AdcPinTrait<u8> for MockedInputPin<'a, T> {

}

#[cfg(feature = "with-hotbed")]
impl<'a, ADC, Word, PIN> embedded_hal::adc::OneShot<ADC, Word, PIN> for MockedInputPin<'a, PIN>
where PIN: embedded_hal::adc::Channel<ADC>
{

    type Error = u8;
    fn read(&mut self, _: &mut PIN) -> Result<Word, nb::Error<u8>> {
        todo!()
    }

}


#[cfg(feature = "with-spi")]
impl<T> embedded_hal::digital::v2::OutputPin for MockedOutputPin<'_, T> {
    type Error = core::convert::Infallible;
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}