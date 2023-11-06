
pub struct MockedSpi {
}

impl MockedSpi {
    pub fn new() -> Self {
        Self {}
    }

    #[allow(unused)]
    pub fn blocking_transfer_in_place<T>(&mut self, _buff: &mut [T]) {}

    #[allow(unused)]
    pub fn blocking_write<T>(&mut self, _buff: &[T]) {}
}

impl embedded_hal::blocking::spi::Transfer<u8> for MockedSpi {
    type Error = core::convert::Infallible;
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        Ok(words)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for MockedSpi {
    type Error = core::convert::Infallible;
    fn write<'w>(&mut self, _words: &'w [u8]) -> Result<(), Self::Error> {
        Ok(())
    }
}
