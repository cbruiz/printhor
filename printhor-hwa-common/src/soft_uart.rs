#![doc = r"
This module provides a software-based serial communication (UART) implementation. 
It includes traits and structures to facilitate read/write operations over a single 
IOPin in a half-duplex mode. 

## Enums

### `UartChannel`
Represents different UART channels.
- `Ch1`: Channel 1
- `Ch2`: Channel 2
- `Ch3`: Channel 3
- `Ch4`: Channel 4

## Traits

### `IOPin`
Defines the necessary operations for an IO pin used in serial communication.
- `set_output`: Sets the pin mode to output.
- `set_input`: Sets the pin mode to input.
- `is_high`: Checks if the pin is high.
- `is_low`: Checks if the pin is low (inline default implementation).
- `set_high`: Sets the pin high.
- `set_low`: Sets the pin low.
- `set_open_drain`: Sets the pin to open drain mode.

### `MultiChannel`
Defines the method to set the UART channel.
- `set_channel`: Sets the UART channel.

### `AsyncRead`
Provides an asynchronous method to read a single word from the serial interface.
- `read`: Reads a single word.

### `AsyncWrite`
Provides an asynchronous method to write a single word to the serial interface.
- `write`: Writes a single word.

## Structs

### `HalfDuplexSerial`
Represents a software-based half-duplex UART implementation.
- `new`: Initializes a new `HalfDuplexSerial`.
- `word_transfer_duration`: Returns the duration for transferring a word.
- `set_timeout`: Sets the read/write timeout.
- `set_write_mode`: Sets the pin mode to write (asynchronously).
- `set_read_mode`: Sets the pin mode to read (asynchronously).

## Example

```rust
use printhor_hwa_common as hwa;
use hwa::soft_uart::HalfDuplexSerial;
use hwa::soft_uart::AsyncWrite;

/// A custom Pin actor implementation
struct MyPinImpl {
    // Fill me
}

impl MyPinImpl {
    const fn new() -> Self {
        Self{}
    }
}
// Implement IOPin trait for your concrete Pin actor

impl hwa::soft_uart::IOPin for MyPinImpl {
    fn set_output(&mut self) {
        // Fill me
    }

    fn set_input(&mut self) {
        // Fill me
    }

    fn is_high(&mut self) -> bool {
        // Fill me
        false
    }

    fn set_high(&mut self) {
        // Fill me
    }

    fn set_low(&mut self) {
        // Fill me
    }

    fn set_open_drain(&mut self) {
        // Fill me
    }
}

async fn usage() {
    let mut _my_serial = HalfDuplexSerial::new(
        MyPinImpl::new(), 115200
    );

    // Use it ...
    let a_byte = '?' as u8;
    let _result = _my_serial.write(a_byte).await;
}
```
"]
//! TODO: [Work In progress] Software serial communication (UART)
use crate as hwa;
use embassy_time::Duration;
use embassy_time::Timer;

/// Software UART channel
#[cfg_attr(feature = "with-log", derive(Debug))]
#[derive(Copy, Clone)]
pub enum UartChannel {
    Ch1,
    Ch2,
    Ch3,
    Ch4,
}

pub trait IOPin {
    fn set_output(&mut self);
    fn set_input(&mut self);

    fn is_high(&mut self) -> bool;

    #[inline]
    fn is_low(&mut self) -> bool {
        !self.is_high()
    }

    fn set_high(&mut self);

    fn set_low(&mut self);

    fn set_open_drain(&mut self);
}

pub trait MultiChannel {
    /// Reads a single word from the serial interface
    fn set_channel(&mut self, channel: Option<UartChannel>);
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
    fn write(
        &mut self,
        word: Word,
    ) -> impl core::future::Future<Output = Result<(), Self::Error>> + Send;
}

/// Bit banging serial communication (UART)
pub struct HalfDuplexSerial<RXTX>
where
    RXTX: IOPin,
{
    rxtx: RXTX,
    // Bit rate period (1 / bauds)
    bit_period: Duration,
    // Read timeout in milliseconds.
    timeout_ms: Option<Duration>,
}

impl<RXTX> HalfDuplexSerial<RXTX>
where
    RXTX: IOPin,
{
    pub fn new(mut rxtx: RXTX, baud_rate: u32) -> Self {
        hwa::debug!("HalfDuplexSerial init at {} baud rate", baud_rate);
        rxtx.set_output();
        rxtx.set_high();

        Self {
            rxtx,
            bit_period: Duration::from_hz(baud_rate as u64),
            timeout_ms: None,
        }
    }

    /// The time it takes to transfer a word (10 bits: 1 start bit + 8 bits + 1 stop bit)
    pub fn word_transfer_duration(&self) -> Duration {
        10 * self.bit_period
    }

    /// the timeout for R/W operation
    pub fn set_timeout(&mut self, timeout_ms: Option<Duration>) {
        self.timeout_ms = timeout_ms;
    }

    pub async fn set_write_mode(&mut self) {
        hwa::debug!("[soft_uart] set write mode()");
        self.rxtx.set_output();
        self.rxtx.set_high();
    }

    pub async fn set_read_mode(&mut self) {
        hwa::debug!("[soft_uart] set read mode()");
        self.rxtx.set_input();
    }
}

impl<RXTX> AsyncWrite<u8> for HalfDuplexSerial<RXTX>
where
    RXTX: IOPin + Send,
{
    type Error = hwa::uart::SerialError;

    async fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        let mut data_out = byte;

        hwa::trace!(
            "[soft_uart] Send word: {:08b} at {} ms period by bit",
            byte,
            self.bit_period.as_millis()
        );
        let mut _reference_instant = embassy_time::Instant::now();
        let mut ticker = embassy_time::Ticker::every(self.bit_period);
        self.rxtx.set_low(); // start bit
        hwa::trace!(
            "[soft_uart] TX: Start bit [at +{} ms] [global_time: {} ms]",
            _reference_instant.elapsed().as_millis(),
            embassy_time::Instant::now().as_millis()
        );
        ticker.next().await;
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.rxtx.set_high();
                hwa::trace!(
                    "[soft_uart] TX: W {:08b} [1] [at +{} ms]",
                    data_out,
                    _reference_instant.elapsed().as_millis()
                );
            } else {
                self.rxtx.set_low();
                hwa::trace!(
                    "[soft_uart] TX: W {:08b} [0] [at +{} ms]",
                    data_out,
                    _reference_instant.elapsed().as_millis()
                );
            }
            data_out >>= 1;
            ticker.next().await;
        }
        self.rxtx.set_high(); // stop bit
        hwa::trace!(
            "[soft_uart] TX: Stop bit [at +{} ms]",
            _reference_instant.elapsed().as_millis()
        );
        ticker.next().await;
        hwa::trace!(
            "[soft_uart] TX: End [at +{} ]",
            _reference_instant.elapsed().as_millis()
        );
        Ok(())
    }
}

impl<RXTX> AsyncRead<u8> for HalfDuplexSerial<RXTX>
where
    RXTX: IOPin + Send,
{
    type Error = hwa::uart::SerialError;

    async fn read(&mut self) -> Result<u8, Self::Error> {
        let mut data_in = 0;

        let mut _reference_instant = embassy_time::Instant::now();

        // wait for start bit
        while self.rxtx.is_high() {
            Timer::after_ticks(self.bit_period.as_ticks() >> 4).await;
            match &self.timeout_ms {
                Some(timeout_ms) => {
                    if _reference_instant.elapsed() > *timeout_ms {
                        return Err(Self::Error::Timeout);
                    }
                }
                None => {}
            }
        }
        _reference_instant = embassy_time::Instant::now();

        hwa::trace!(
            "[soft_uart] RX: Start bit detected at [at +{}] [global_time: {} ms]",
            _reference_instant.as_millis(),
            embassy_time::Instant::now().as_millis()
        );
        // Aditional wait: Align to pulse center assuming start bit is detected closely after rising edge
        Timer::after_ticks(self.bit_period.as_ticks() + (self.bit_period.as_ticks() >> 1)).await;
        let mut ticker = embassy_time::Ticker::every(self.bit_period);
        // Sample 8 bits
        for _bit in 0..8 {
            data_in >>= 1;
            if self.rxtx.is_high() {
                data_in |= 0b10000000;
                hwa::trace!(
                    "[soft_uart] RX: R {:08b} [1] [at +{} ms]",
                    data_in,
                    _reference_instant.elapsed().as_millis()
                );
            } else {
                hwa::trace!(
                    "[soft_uart] RX: R {:08b} [0] [at +{} ms]",
                    data_in,
                    _reference_instant.elapsed().as_millis()
                );
            }
            ticker.next().await;
        }
        // Expect stop bit
        if self.rxtx.is_high() {
            hwa::trace!(
                "[soft_uart] RX: Stop bit [at +{} ms]",
                _reference_instant.elapsed().as_millis()
            );
            hwa::trace!("[soft_uart] RX: Got: {:08b}", data_in);
            Ok(data_in)
        } else {
            hwa::debug!(
                "[soft_uart] RX: Stop bit missing [at +{} ms]",
                _reference_instant.elapsed().as_millis()
            );
            hwa::trace!("[soft_uart] RX: Getting so far: {:08b}", data_in);
            Err(Self::Error::Framing)
        }
    }
}

#[cfg(test)]
mod tests {
    use crate as hwa;
    use crate::soft_uart::{AsyncRead, AsyncWrite, IOPin};
    use hwa::soft_uart::HalfDuplexSerial;

    /// A custom Pin actor implementation
    struct MyPinImpl {
        state: Option<bool>,
    }

    impl MyPinImpl {
        const fn new() -> Self {
            Self { state: None }
        }
    }
    // Implement IOPin trait for your concrete Pin actor
    impl hwa::soft_uart::IOPin for MyPinImpl {
        fn set_output(&mut self) {
            // No action
        }

        fn set_input(&mut self) {
            // No action
        }

        fn is_high(&mut self) -> bool {
            self.state.unwrap() == true
        }

        fn set_high(&mut self) {
            self.state.replace(true);
        }

        fn set_low(&mut self) {
            self.state.replace(false);
        }

        fn set_open_drain(&mut self) {
            // No action
        }
    }

    #[test]
    fn pin_tests() {
        let mut pin = MyPinImpl::new();
        pin.set_output();
        pin.set_high();
        assert_eq!(pin.is_high(), true);
        assert_eq!(pin.is_low(), false);

        pin.set_low();
        assert_eq!(pin.is_high(), false);
        assert_eq!(pin.is_low(), true);

        pin.set_input();
        assert_eq!(pin.is_high(), false);
        assert_eq!(pin.is_low(), true);

        pin.set_output();
        pin.set_high();

        pin.set_open_drain();
        assert_eq!(pin.is_high(), true);
        assert_eq!(pin.is_low(), false);
    }

    #[futures_test::test]
    async fn half_duplex_serial_mock_test() {
        let mut serial = HalfDuplexSerial::new(MyPinImpl::new(), 115200);

        let _ = serial.word_transfer_duration();
        serial.set_timeout(None);
        serial.set_timeout(Some(embassy_time::Duration::from_secs(2)));

        let _ = serial.set_write_mode().await;
        // Use it ...
        let _w_res = serial.write('?' as u8).await;

        let _ = serial.set_read_mode().await;

        let _r_res = serial.read().await;
    }
}
