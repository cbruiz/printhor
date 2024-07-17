use embassy_stm32::wdg;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::timer::simple_pwm::SimplePwm;

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type USBDrv = embassy_stm32::usb_otg::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>;
        pub use crate::board::io::usbserial::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-1")] {
        type UsartPort1Peri = embassy_stm32::peripherals::USART1;
        type UsartPort1TxDma = embassy_stm32::peripherals::DMA2_CH7;
        type UsartPort1RxDma = embassy_stm32::peripherals::DMA2_CH5;

        pub type UartPort1Device = embassy_stm32::usart::Uart<'static, UsartPort1Peri, UsartPort1TxDma, UsartPort1RxDma>;
        pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static, UsartPort1Peri, UsartPort1TxDma>;
        pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static, UsartPort1Peri, UsartPort1RxDma>;
        pub type UartPort1TxControllerRef = printhor_hwa_common::ControllerRef<UartPort1TxDevice>;
        pub use crate::board::io::uart_port1::UartPort1RxInputStream;
        cfg_if::cfg_if! {
            if #[cfg(feature="upstream-embassy")] {
                pub type UartPort1RingBufferedRxDevice = embassy_stm32::usart::RingBufferedUartRx<'static, UsartPort1Peri>;
            }
            else {
                pub type UartPort1RingBufferedRxDevice = embassy_stm32::usart::RingBufferedUartRx<'static, UsartPort1Peri, UsartPort1RxDma>;
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-trinamic")] {
        pub type TrinamicUart = crate::board::comm::SingleWireSoftwareUart;
        pub use crate::board::comm::AxisChannel;
        pub type TMCUartCh1Pin = embassy_stm32::peripherals::PD5;
        pub type TMCUartCh2Pin = embassy_stm32::peripherals::PD1;
        pub type TMCUartCh3Pin = embassy_stm32::peripherals::PD4;
        pub type TMCUartCh4Pin = embassy_stm32::peripherals::PD9;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        cfg_if::cfg_if! {
            if #[cfg(feature = "upstream-embassy")] {
                pub type PsOnPin = Output<'static>;
            }
            else {
                pub type PsOnPin = Output<'static, embassy_stm32::peripherals::PB2>;
            }
        }
        pub type PsOnRef = printhor_hwa_common::ControllerRef<PsOnPin>;
    }
}

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


cfg_if::cfg_if! {
    if #[cfg(feature = "with-sdcard")] {
        cfg_if::cfg_if! {
            if #[cfg(feature = "upstream-embassy")] {
                pub type SpiCardCSPin = Output<'static>;
            }
            else {
                pub type SpiCardCSPin = Output<'static, embassy_stm32::peripherals::PC9>;
            }
        }
    }
}

pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
pub use embassy_stm32::adc::Instance as AdcTrait;
pub use embassy_stm32::adc::AdcPin as AdcPinTrait;
pub use embassy_stm32::adc::VrefInt as VrefInt;
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
pub type PwmFanLayer = SimplePwm<'static, embassy_stm32::peripherals::TIM3>;
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

#[cfg(feature = "with-hot-end")]
pub struct HotendPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmHotend>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::ControllerRef<AdcHotend>,
    pub temp_pin: AdcHotendPin,
    pub thermistor_properties: &'static printhor_hwa_common::ThermistorProperties,
}

#[cfg(feature = "with-hot-bed")]
pub struct HotbedPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmHotbed>,
    pub power_channel: PwmChannel,
    pub temp_adc: printhor_hwa_common::ControllerRef<AdcHotbed>,
    pub temp_pin: AdcHotbedPin,
    pub thermistor_properties: &'static printhor_hwa_common::ThermistorProperties,
}

#[cfg(feature = "with-fan-layer")]
pub struct FanLayerPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmFanLayer>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-laser")]
pub struct LaserPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmLaser>,
    pub power_channel: PwmChannel,
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        cfg_if::cfg_if!{
            if #[cfg(feature = "upstream-embassy")] {
                pub struct MotionPins {
                    pub x_enable_pin: Output<'static>,
                    pub y_enable_pin: Output<'static>,
                    pub z_enable_pin: Output<'static>,
                    pub e_enable_pin: Output<'static>,

                    pub x_endstop_pin: Input<'static>,
                    pub y_endstop_pin: Input<'static>,
                    pub z_endstop_pin: Input<'static>,
                    pub e_endstop_pin: Input<'static>,

                    pub x_step_pin: Output<'static>,
                    pub y_step_pin: Output<'static>,
                    pub z_step_pin: Output<'static>,
                    pub e_step_pin: Output<'static>,

                    pub x_dir_pin: Output<'static>,
                    pub y_dir_pin: Output<'static>,
                    pub z_dir_pin: Output<'static>,
                    pub e_dir_pin: Output<'static>,
                }
            }
            else {
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
            }
        }
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
            #[inline]
            pub fn enable_all_steppers(&mut self) {
                self.x_enable_pin.set_low();
                self.y_enable_pin.set_low();
                self.z_enable_pin.set_low();
                self.e_enable_pin.set_low();
            }
        }
        pub struct MotionDevice {
            #[cfg(feature = "with-trinamic")]
            pub trinamic_uart: TrinamicUart,

            pub motion_pins: MotionPins,
        }
    }
}

#[cfg(feature = "with-sdcard")]
pub struct CardDevice {
    pub card_spi: SpiCardDevice,
    pub card_cs: SpiCardCSPin,
}