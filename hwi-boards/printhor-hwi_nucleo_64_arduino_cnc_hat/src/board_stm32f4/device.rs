#[allow(unused)]
use printhor_hwa_common as hwa;

pub type Watchdog = embassy_stm32::wdg::IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>;
pub type SerialPort1Tx = hwa::SerialTxWrapper<embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>>;
pub type SerialPort1Rx = super::io::uart_port1::UartPort1RxInputStream;