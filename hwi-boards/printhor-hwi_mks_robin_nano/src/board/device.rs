use embassy_stm32::wdg;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::timer::simple_pwm::SimplePwm;

#[cfg(feature = "with-serial-usb")]
pub type USBDrv = embassy_stm32::usb_otg::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>;

#[cfg(feature = "with-serial-usb")]
pub use crate::board::io::usbserial::*;

#[cfg(feature = "with-serial-port-1")]
pub(crate) type UartPort1Device = embassy_stm32::usart::Uart<'static,
    embassy_stm32::peripherals::USART1,
    embassy_stm32::peripherals::DMA2_CH7, embassy_stm32::peripherals::DMA2_CH5
>;
#[cfg(feature = "with-serial-port-1")]
pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static,
    embassy_stm32::peripherals::USART1, embassy_stm32::peripherals::DMA2_CH7
>;
#[cfg(feature = "with-serial-port-1")]
pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static,
    embassy_stm32::peripherals::USART1, embassy_stm32::peripherals::DMA2_CH5
>;
#[cfg(feature = "with-serial-port-1")]
pub type UartPort1TxControllerRef = crate::board::ControllerRef<UartPort1TxDevice>;
#[cfg(feature = "with-serial-port-1")]
pub use crate::board::io::uart_port1::UartPort1RxInputStream;

#[cfg(feature = "with-trinamic")]
pub type UartTrinamic = crate::board::comm::SingleWireSoftwareUart;

#[cfg(feature = "with-trinamic")]
pub use crate::board::comm::AxisChannel;

#[cfg(feature = "with-trinamic")]
pub type TMCUartCh1Pin = embassy_stm32::peripherals::PD5;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh2Pin = embassy_stm32::peripherals::PD7;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh3Pin = embassy_stm32::peripherals::PD4;
#[cfg(feature = "with-trinamic")]
pub type TMCUartCh4Pin = embassy_stm32::peripherals::PD9;

#[cfg(feature = "with-spi")]
pub(crate) type Spi1 = embassy_stm32::spi::Spi<'static,
    embassy_stm32::peripherals::SPI3,
    embassy_stm32::peripherals::DMA1_CH5, embassy_stm32::peripherals::DMA1_CH0
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
pub type SpiCardCSPin = Output<'static, embassy_stm32::peripherals::PC9>;

pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
pub use embassy_stm32::adc::Instance as AdcTrait;
pub use embassy_stm32::adc::AdcPin as AdcPinTrait;
pub type AdcHotendHotbedPeripheral = embassy_stm32::peripherals::ADC1;
pub type AdcHotendHotbed = AdcImpl<AdcHotendHotbedPeripheral>;
pub type AdcHotendPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotbedPeripheral = AdcHotendHotbedPeripheral;
pub type AdcHotend = AdcHotendHotbed;
pub type AdcHotbed = AdcHotendHotbed;
pub type AdcHotendPin = embassy_stm32::peripherals::PC1;
pub type AdcHotbedPin = embassy_stm32::peripherals::PC0;

pub use embassy_stm32::timer::CaptureCompare16bitInstance as PwmTrait;
pub type PwmImpl<TimPeri> = embassy_stm32::timer::simple_pwm::SimplePwm<'static, TimPeri>;

pub type PwmServo = SimplePwm<'static, embassy_stm32::peripherals::TIM1>;
pub type PwmLayerFan = SimplePwm<'static, embassy_stm32::peripherals::TIM3>;
pub type PwmHotend = SimplePwm<'static, embassy_stm32::peripherals::TIM9>;
pub type PwmHotbed = SimplePwm<'static, embassy_stm32::peripherals::TIM5>;
pub type PwmLaser = SimplePwm<'static, embassy_stm32::peripherals::TIM13>;

pub type PwmChannel = embassy_stm32::timer::Channel;


pub type Watchdog = wdg::IndependentWatchdog<'static,
    embassy_stm32::peripherals::IWDG
>;


#[cfg(feature = "with-probe")]
pub struct ProbePeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmServo>,
    pub power_channel: PwmChannel,
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

#[cfg(feature = "with-fan-layer-fan1")]
pub struct LayerFanPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmLayerFan>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmLaser>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-motion")]
pub struct MotionPins {
    pub x_enable_pin: Output<'static, embassy_stm32::peripherals::PE4>,
    pub y_enable_pin: Output<'static, embassy_stm32::peripherals::PE1>,
    pub z_enable_pin: Output<'static, embassy_stm32::peripherals::PB8>,
    pub e_enable_pin: Output<'static, embassy_stm32::peripherals::PB3>,

    pub x_endstop_pin: Input<'static, embassy_stm32::peripherals::PA15>,
    pub y_endstop_pin: Input<'static, embassy_stm32::peripherals::PD2>,
    pub z_endstop_pin: Input<'static, embassy_stm32::peripherals::PC8>,
    pub e_endstop_pin: Input<'static, embassy_stm32::peripherals::PC4>,

    pub x_step_pin: Output<'static, embassy_stm32::peripherals::PE3>,
    pub y_step_pin: Output<'static, embassy_stm32::peripherals::PE0>,
    pub z_step_pin: Output<'static, embassy_stm32::peripherals::PB5>,
    pub e_step_pin: Output<'static, embassy_stm32::peripherals::PD6>,

    pub x_dir_pin: Output<'static, embassy_stm32::peripherals::PE2>,
    pub y_dir_pin: Output<'static, embassy_stm32::peripherals::PB9>,
    pub z_dir_pin: Output<'static, embassy_stm32::peripherals::PB4>,
    pub e_dir_pin: Output<'static, embassy_stm32::peripherals::PD3>,
}

#[cfg(feature = "with-motion")]
impl MotionPins {
    #[inline]
    pub fn enable_x_stepper(&mut self) {
        self.x_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_y_stepper(&mut self) {
        self.y_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_z_stepper(&mut self) {
        self.z_enable_pin.set_low();
    }
    #[inline]
    pub fn enable_e_stepper(&mut self) {
        self.e_enable_pin.set_low();
    }
    #[inline]
    pub fn disable_x_stepper(&mut self) {
        self.x_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_y_stepper(&mut self) {
        self.y_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_z_stepper(&mut self) {
        self.z_enable_pin.set_high();
    }
    #[inline]
    pub fn disable_e_stepper(&mut self) { self.e_enable_pin.set_high(); }
    #[inline]
    pub fn disable_all_steppers(&mut self) {
        self.x_enable_pin.set_high();
        self.y_enable_pin.set_high();
        self.z_enable_pin.set_high();
        self.e_enable_pin.set_high();
    }
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
