use std::marker::PhantomData;
use embassy_time::{Duration, Timer};
#[cfg(feature = "with-hotbed")]
use crate::hwi::native::traits::{AdcPin, TemperatureAdcCompat};

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

#[cfg(feature = "with-hotbed")]
impl<'a, T> AdcPin for MockedInputPin<'a, T> {}

#[cfg(feature = "with-hotbed")]
impl<'a, T> TemperatureAdcCompat for MockedInputPin<'a, T> {}


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