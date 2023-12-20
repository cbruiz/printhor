//! This specialization exists because there are several ways to communicate with stepper's Trinamic UART
use embassy_stm32::usart::Error;

pub struct SingleWireSoftwareUart {


}

impl SingleWireSoftwareUart {

    pub fn new() -> Self {
        Self {

        }
    }

    pub async fn read_until_idle(&mut self, _buffer: &mut [u8]) -> Result<usize, Error>
    {
        Ok(0)
    }

    pub async fn write(&mut self, _buffer: &[u8]) -> Result<(), Error> {
        Ok(())
    }

    #[inline]
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        Ok(())
    }

}