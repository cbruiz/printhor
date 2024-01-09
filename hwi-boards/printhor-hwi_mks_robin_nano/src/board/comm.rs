//! This specialization exists because there are several ways to communicate with stepper's Trinamic UART
use embassy_stm32::gpio::{Pin, AnyPin, Flex, Pull, Speed};
use printhor_hwa_common::soft_uart;
use printhor_hwa_common::soft_uart::{AsyncRead, MultiChannel, UartChannel};
use crate::device;

#[derive(Debug)]
pub enum Error {
    Uninit,
    Timeout,
}

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

pub struct AnyPinWrapper(Flex<'static, AnyPin>);

impl soft_uart::IOPin for AnyPinWrapper
{
    #[inline]
    fn set_output(&mut self) {
        self.0.set_as_output(Speed::VeryHigh);
    }
    #[inline]
    fn set_input(&mut self) {
        self.0.set_as_input(Pull::Down);
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
        tmc_uart_ch1_pin: device::TMCUartCh1Pin,
        tmc_uart_ch2_pin: device::TMCUartCh2Pin,
        tmc_uart_ch3_pin: device::TMCUartCh3Pin,
        tmc_uart_ch4_pin: device::TMCUartCh4Pin,
    ) -> Self {
        Self {
            tmc_uarts: [
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(Flex::new(tmc_uart_ch1_pin.degrade())), baud_rate),
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(Flex::new(tmc_uart_ch2_pin.degrade())), baud_rate),
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(Flex::new(tmc_uart_ch3_pin.degrade())), baud_rate),
                soft_uart::HalfDuplexSerial::new(AnyPinWrapper(Flex::new(tmc_uart_ch4_pin.degrade())), baud_rate),
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

        uart.set_timeout(Some(uart.word_transfer_duration() * 10));

        let _x = uart.read().await;

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

impl MultiChannel for SingleWireSoftwareUart {
    fn set_channel(&mut self, channel: Option<UartChannel>) {
        self.selected = channel;
    }
}