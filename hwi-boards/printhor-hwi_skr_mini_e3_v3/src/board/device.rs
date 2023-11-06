use embassy_stm32::wdg;
use embassy_stm32::gpio::Output;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::exti::ExtiInput;

#[cfg(feature = "with-usbserial")]
pub type USBDrv = embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB>;

#[cfg(feature = "with-usbserial")]
pub use crate::board::io::usbserial::*;

#[cfg(feature = "with-uart-port-1")]
pub(crate) type UartPort1Device = embassy_stm32::usart::Uart<'static,
    embassy_stm32::peripherals::USART2,
    embassy_stm32::peripherals::DMA2_CH2, embassy_stm32::peripherals::DMA2_CH1
>;
#[cfg(feature = "with-uart-port-1")]
pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static,
    embassy_stm32::peripherals::USART2, embassy_stm32::peripherals::DMA2_CH2
>;
#[cfg(feature = "with-uart-port-1")]
pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static,
    embassy_stm32::peripherals::USART2, embassy_stm32::peripherals::DMA2_CH1
>;
#[cfg(feature = "with-uart-port-1")]
pub type UartPort1TxControllerRef = crate::board::ControllerRef<UartPort1TxDevice>;
#[cfg(feature = "with-uart-port-1")]
pub use crate::board::io::uart_port1::UartPort1RxInputStream;

#[cfg(feature = "with-trinamic")]
pub type Uart4 = crate::board::usart::Uart<'static,
    embassy_stm32::peripherals::USART4,
    embassy_stm32::peripherals::DMA1_CH7, embassy_stm32::peripherals::DMA1_CH6
>;

#[cfg(feature = "with-trinamic")]
pub type UartTrinamic = Uart4;

#[cfg(feature = "with-spi")]
pub(crate) type Spi1 = embassy_stm32::spi::Spi<'static,
    embassy_stm32::peripherals::SPI1,
    embassy_stm32::peripherals::DMA1_CH2, embassy_stm32::peripherals::DMA1_CH1
>;

#[cfg(feature = "with-spi")]
pub type SpiCardDevice = Spi1;

#[cfg(feature = "with-spi")]
pub type Spi = Spi1;

#[cfg(feature = "with-spi")]
pub type SpiDeviceRef = crate::board::ControllerRef<Spi>;

#[cfg(feature = "with-sdcard")]
pub type SpiCardDeviceRef = crate::board::ControllerRef<Spi>;

#[cfg(feature = "with-sdcard")]
pub type SpiCardCSPin = Output<'static, embassy_stm32::peripherals::PA4>;

pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
pub trait AdcTrait = embassy_stm32::adc::Instance;
pub trait AdcPinTrait<PERI: AdcTrait> = embassy_stm32::adc::AdcPin<PERI>;
pub type AdcHotendHotbedPeripheral = embassy_stm32::peripherals::ADC1;
pub type AdcHotendHotbed = AdcImpl<AdcHotendHotbedPeripheral>;
pub type AdcHotendPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotbedPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotend = AdcHotendHotbed;
pub type AdcHotbed = AdcHotendHotbed;
pub type AdcHotendPin = embassy_stm32::peripherals::PA0;
pub type AdcHotbedPin = embassy_stm32::peripherals::PC4;

pub trait PwmTrait = embassy_stm32::timer::CaptureCompare16bitInstance;
pub type PwmImpl<TimPeri> = embassy_stm32::timer::simple_pwm::SimplePwm<'static, TimPeri>;

pub type PwmServo = SimplePwm<'static, embassy_stm32::peripherals::TIM2>;

#[cfg(feature = "with-laser")]
pub type PwmLaser = SimplePwm<'static, embassy_stm32::peripherals::TIM16>;

pub type PwmFan0Fan1HotendHotbed = SimplePwm<'static, embassy_stm32::peripherals::TIM3>;

pub type PwmFan0 = PwmFan0Fan1HotendHotbed;
pub type PwmFan1 = PwmFan0Fan1HotendHotbed;
pub type PwmHotend = PwmFan0Fan1HotendHotbed;
pub type PwmHotbed = PwmFan0Fan1HotendHotbed;

pub type PwmChannel = embassy_stm32::timer::Channel;

pub type Watchdog = wdg::IndependentWatchdog<'static,
    embassy_stm32::peripherals::IWDG
>;

#[allow(non_camel_case_types)]
pub type DISPLAY_SER_RST_OUTPUT = Output<'static, embassy_stm32::peripherals::PC1>;
#[allow(non_camel_case_types)]
pub type DISPLAY_SER_CS_OUTPUT = Output<'static, embassy_stm32::peripherals::PB0>;
#[allow(non_camel_case_types)]
pub type DISPLAY_SER_DC_OUTPUT = Output<'static, embassy_stm32::peripherals::PA4>;

#[cfg(feature = "ili9341_spi")]
pub struct DisplayDevice {
    pub interface: SpiControllerRef,
    pub rst: DISPLAY_SER_RST_OUTPUT,
    pub cs: DISPLAY_SER_CS_OUTPUT,
    pub dc: DISPLAY_SER_DC_OUTPUT,
}

#[cfg(feature = "with-probe")]
pub struct ProbePeripherals {
    pub probe_pwm: PwmServo,
    pub probe_channel: PwmChannel,
}

#[cfg(feature = "with-hotend")]
pub struct HotendPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmHotend>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::ControllerRef<AdcHotend>,
    pub temp_pin: AdcHotendPin
}

#[cfg(feature = "with-hotbed")]
pub struct HotbedPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmHotbed>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::ControllerRef<AdcHotbed>,
    pub temp_pin: AdcHotbedPin
}

#[cfg(feature = "with-fan0")]
pub struct Fan0Peripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmFan0>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-fan1")]
pub struct Fan1Peripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmFan1>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmLaser>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-motion")]
pub struct MotionPins {
    pub x_enable_pin: Output<'static, embassy_stm32::peripherals::PB14>,
    pub y_enable_pin: Output<'static, embassy_stm32::peripherals::PB11>,
    pub z_enable_pin: Output<'static, embassy_stm32::peripherals::PB1>,
    pub e_enable_pin: Output<'static, embassy_stm32::peripherals::PD1>,

    pub x_endstop_pin: ExtiInput<'static, embassy_stm32::peripherals::PC0>,
    pub y_endstop_pin: ExtiInput<'static, embassy_stm32::peripherals::PC1>,
    pub z_endstop_pin: ExtiInput<'static, embassy_stm32::peripherals::PC2>,
    pub e_endstop_pin: ExtiInput<'static, embassy_stm32::peripherals::PC15>,

    pub x_step_pin: Output<'static, embassy_stm32::peripherals::PB13>,
    pub y_step_pin: Output<'static, embassy_stm32::peripherals::PB10>,
    pub z_step_pin: Output<'static, embassy_stm32::peripherals::PB0>,
    pub e_step_pin: Output<'static, embassy_stm32::peripherals::PB3>,

    pub x_dir_pin: Output<'static, embassy_stm32::peripherals::PB12>,
    pub y_dir_pin: Output<'static, embassy_stm32::peripherals::PB2>,
    pub z_dir_pin: Output<'static, embassy_stm32::peripherals::PC5>,
    pub e_dir_pin: Output<'static, embassy_stm32::peripherals::PB4>,
}

#[cfg(feature = "with-motion")]
pub struct MotionDevice {

    #[cfg(feature = "with-trinamic")]
    pub trinamic_uart: UartTrinamic,

    pub motion_pins: MotionPins,
}


#[cfg(feature = "with-sdcard")]
pub struct CardDevice {

    pub card_spi: SpiCardDevice,
    pub card_cs: SpiCardCSPin,

}
