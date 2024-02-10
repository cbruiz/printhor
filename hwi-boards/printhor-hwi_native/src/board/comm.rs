//! This specialization exists because there are several ways to communicate with stepper's Trinamic UART

use printhor_hwa_common as hwa;
use hwa::soft_uart;
use hwa::soft_uart::{AsyncRead, MultiChannel, UartChannel};
use printhor_hwa_common::soft_uart::{AsyncWrite, SerialError};
use crate::device;

#[derive(Debug)]
pub enum Error {
    Uninit,
    Timeout,
}
#[allow(unused)]
use crate::board::MockedIOPin;

/// Software UART channel
#[derive(Debug)]
pub enum AxisChannel {
    TMCUartX,
    TMCUartY,
    TMCUartZ,
    TMCUartE,
}
/// TODO: turn into a easily configurable property
impl Into<UartChannel> for AxisChannel {
    fn into(self) -> UartChannel {
        match self {
            AxisChannel::TMCUartX => UartChannel::Ch1,
            AxisChannel::TMCUartY => UartChannel::Ch2,
            AxisChannel::TMCUartZ => UartChannel::Ch3,
            AxisChannel::TMCUartE => UartChannel::Ch4,
        }
    }
}

pub struct AnyPinWrapper(MockedIOPin);

impl soft_uart::IOPin for AnyPinWrapper
{
    #[inline]
    fn set_output(&mut self) {
    }
    #[inline]
    fn set_input(&mut self) {
    }
    #[inline]
    fn is_high(&mut self) -> bool {
        self.0.is_high()
    }
    #[inline]
    fn set_high(&mut self) { self.0.set_high() }
    #[inline]
    fn set_low(&mut self) { self.0.set_low() }
}

pub struct SingleWireSoftwareUart {
    tmc_uarts: [soft_uart::HalfDuplexSerial<AnyPinWrapper>; 4],
    selected: Option<UartChannel>,
}

impl SingleWireSoftwareUart {

    pub fn new(
        baud_rate: u32,
        mut tmc_uart_ch1_pin: device::TMCUartCh1Pin,
        mut tmc_uart_ch2_pin: device::TMCUartCh2Pin,
        mut tmc_uart_ch3_pin: device::TMCUartCh3Pin,
        mut tmc_uart_ch4_pin: device::TMCUartCh4Pin,
    ) -> Self {
        tmc_uart_ch1_pin.set_high();
        tmc_uart_ch2_pin.set_high();
        tmc_uart_ch3_pin.set_high();
        tmc_uart_ch4_pin.set_high();
        Self {
            tmc_uarts: [
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(tmc_uart_ch1_pin), baud_rate),
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(tmc_uart_ch2_pin), baud_rate),
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(tmc_uart_ch3_pin), baud_rate),
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(tmc_uart_ch4_pin), baud_rate),
                ],
            selected: None,
        }
    }

    /// "Low level" speciallization with channel semantics
    pub fn set_axis_channel(&mut self, axis_channel: Option<AxisChannel>) {
        self.set_channel(axis_channel.and_then(|ac| Some(ac.into())));
    }


    pub async fn read_until_idle(&mut self, _buffer: &mut [u8]) -> Result<usize, Error>
    {
        let uart = match self.selected.as_ref().ok_or(Error::Uninit)? {
            UartChannel::Ch1 => &mut self.tmc_uarts[0],
            UartChannel::Ch2=> &mut self.tmc_uarts[1],
            UartChannel::Ch3=> &mut self.tmc_uarts[2],
            UartChannel::Ch4=> &mut self.tmc_uarts[3],
        };

        uart.set_timeout(Some(uart.word_transfer_duration()));
        let mut read_idx: usize = 0;
        loop {
            if read_idx >= _buffer.len() {
                return Ok(read_idx)
            }
            match uart.read().await {
                Ok(b) => {
                    _buffer[read_idx] = b;
                    read_idx += 1;
                },
                Err(SerialError::Timeout) => {
                    if read_idx > 0 {
                        return Ok(read_idx)
                    }
                    else {
                        return Err(Error::Timeout)
                    }
                },
                _e => {
                    return Err(Error::Uninit)   // FIXME: Proper codes
                }
            }
        }
    }

    pub async fn write(&mut self, _buffer: &[u8]) -> Result<(), Error> {
        let uart = match self.selected.as_ref().ok_or(Error::Uninit)? {
            UartChannel::Ch1 => &mut self.tmc_uarts[0],
            UartChannel::Ch2=> &mut self.tmc_uarts[1],
            UartChannel::Ch3=> &mut self.tmc_uarts[2],
            UartChannel::Ch4=> &mut self.tmc_uarts[3],
        };
        for b in _buffer {
            uart.write(*b).await.map_err(|_e| Error::Timeout)?;
        }
        log::info!("Written {:?}", _buffer);
        Ok(())
    }

    #[inline]
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        Ok(())
    }

}

impl MultiChannel for SingleWireSoftwareUart {
    fn set_channel(&mut self, channel: Option<UartChannel>) {
        self.selected = channel;
    }
}