//! TODO: [Work In progress] Software serial communication (UART)

use embassy_time::Timer;
use embassy_time::Duration;

/// Serial communication error type
#[derive(Debug)]
pub enum SerialError {
    /// Framing error
    Framing,
}

pub trait IOPin {
    fn set_output(&mut self);
    fn set_input(&mut self);

    fn is_high(&mut self) -> bool;

    #[inline]
    fn is_low(&mut self) -> bool {!self.is_high()}

    fn set_high(&mut self);

    fn set_low(&mut self);
}

pub trait AsyncRead<Word> {
    /// Read error
    type Error;

    /// Reads a single word from the serial interface
    fn read(&mut self) -> impl core::future::Future<Output = Result<Word, Self::Error>> + Send;
}

pub trait AsyncWrite<Word> {
    /// Read error
    type Error;

    /// Writes a single word from the serial interface
    fn write(&mut self, word: Word) -> impl core::future::Future<Output = Result<(), Self::Error>> + Send;
}

/// Bit banging serial communication (UART)
pub struct HalfDuplexSerial<RXTX>
    where
        RXTX: IOPin,
{
    rxtx: RXTX,
    t: Duration,
}

impl<RXTX> HalfDuplexSerial<RXTX>
    where
        RXTX: IOPin,
{
    pub fn new(rxtx: RXTX) -> Self {

        Self { rxtx, t: Duration::from_hz(115200) }
    }
}

impl<RXTX> AsyncWrite<u8> for HalfDuplexSerial<RXTX>
    where
        RXTX: IOPin + Send,
{
    type Error = SerialError;

    async fn write(&mut self, byte: u8) -> Result<(), Self::Error> {

        let mut data_out = byte;
        self.rxtx.set_output();

        let mut ticker = embassy_time::Ticker::every(self.t);
        self.rxtx.set_low(); // start bit
        ticker.next().await;
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.rxtx.set_high();
            } else {
                self.rxtx.set_low();
            }
            data_out >>= 1;
            ticker.next().await;
        }
        self.rxtx.set_high(); // stop bit
        ticker.next().await;
        self.rxtx.set_low();
        self.rxtx.set_input(); // safety
        Ok(())
    }
}

impl<RXTX> AsyncRead<u8> for HalfDuplexSerial<RXTX>
    where
        RXTX: IOPin + Send
{
    type Error = SerialError;

    async fn read(&mut self) -> Result<u8, Self::Error> {

        self.rxtx.set_input();
        let mut data_in = 0;

        // wait for start bit
        while self.rxtx.is_high() {
            Timer::after_ticks(self.t.as_ticks() << 1).await;
        }
        // Read 8 bits
        for _bit in 0..8 {
            Timer::after_ticks(self.t.as_ticks()).await;
            data_in <<= 1;
            if self.rxtx.is_high() {
                data_in |= 1
            }
        }
        // wait for stop bit
        Timer::after_ticks(self.t.as_ticks()).await;
        if self.rxtx.is_high() {
            Ok(data_in)
        }
        else {
            Err(Self::Error::Framing)
        }
    }
}