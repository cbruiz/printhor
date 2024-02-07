use embassy_stm32::wdg;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::timer::simple_pwm::SimplePwm;

// Common for both boards
cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type USBDrv = embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB>;
        pub use crate::board::io::usbserial::*;
    }
}

// Specific for single board
cfg_if::cfg_if! {
    // Specific to sk3_mini_e3_v2
    if #[cfg(feature="sk3_mini_e3_v2")] {

        cfg_if::cfg_if! {
            if #[cfg(feature="with-trinamic")] {
                type TrinamicUartPeri = embassy_stm32::peripherals::UART4;
                type TrinamicUartTxDma = embassy_stm32::peripherals::DMA2_CH5;
                type TrinamicUartRxDma = embassy_stm32::peripherals::DMA2_CH3;

                pub type UartTrinamic = crate::board::usart::Uart<'static, TrinamicUartPeri, TrinamicUartTxDma, TrinamicUartRxDma>;
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature="with-serial-port-1")] {
                type UsartPort1Peri = embassy_stm32::peripherals::USART2;
                type UsartPort1TxDma = embassy_stm32::peripherals::DMA1_CH7;
                type UsartPort1RxDma = embassy_stm32::peripherals::DMA1_CH6;

                pub(crate) type UartPort1Device = embassy_stm32::usart::Uart<'static, UsartPort1Peri, UsartPort1TxDma, UsartPort1RxDma>;
                pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static, UsartPort1Peri, UsartPort1TxDma>;
                pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static, UsartPort1Peri, UsartPort1RxDma>;

                pub type UartPort1TxControllerRef = crate::board::ControllerRef<UartPort1TxDevice>;
                pub use crate::board::io::uart_port1::UartPort1RxInputStream;
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature="with-spi")] {
                type SpiPeri = embassy_stm32::peripherals::SPI1;
                type SpiPeriTxDma = embassy_stm32::peripherals::DMA1_CH4;
                type SpiPeriRxDma = embassy_stm32::peripherals::DMA1_CH3;

                pub(crate) type Spi1 = embassy_stm32::spi::Spi<'static, SpiPeri, SpiPeriTxDma, SpiPeriRxDma>;
            }
        }

        // [Customization!!!] Use NeoPixel PWM as laser in this board
        cfg_if::cfg_if! {
            if #[cfg(feature="with-laser")] {
                pub type PwmLaser = SimplePwm<'static, embassy_stm32::peripherals::TIM1>;
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(any(feature="with-hotend", feature="with-hotend"))] {
                pub type AdcHotendHotbedPeripheral = embassy_stm32::peripherals::ADC1;
                pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
                pub use embassy_stm32::adc::Instance as AdcTrait;
                pub use embassy_stm32::adc::AdcPin as AdcPinTrait;

                pub type AdcHotendHotbed = AdcImpl<AdcHotendHotbedPeripheral>;
                pub type AdcHotendPeripheral = AdcHotendHotbedPeripheral;
                pub type AdcHotbedPeripheral = AdcHotendHotbedPeripheral;
                pub type AdcHotend = AdcHotendHotbed;
                pub type AdcHotbed = AdcHotendHotbed;
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature="with-hotend")] {
                pub type AdcHotendPin = embassy_stm32::peripherals::PA0;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="with-hotbed")] {
                pub type AdcHotbedPin = embassy_stm32::peripherals::PC3;
            }
        }
    }
    else if #[cfg(feature="sk3_mini_e3_v3")] {
        #[cfg(feature = "with-trinamic")]
        pub type UartTrinamic = crate::board::usart::Uart<'static,
            embassy_stm32::peripherals::USART4,
            embassy_stm32::peripherals::DMA1_CH7, embassy_stm32::peripherals::DMA1_CH6>;

        #[cfg(feature = "with-serial-port-1")]
        pub(crate) type UartPort1Device = embassy_stm32::usart::Uart<'static,
            embassy_stm32::peripherals::USART2,
            embassy_stm32::peripherals::DMA2_CH2, embassy_stm32::peripherals::DMA2_CH1>;

        #[cfg(feature = "with-serial-port-1")]
        pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static,
            embassy_stm32::peripherals::USART2, embassy_stm32::peripherals::DMA2_CH2>;

        #[cfg(feature = "with-serial-port-1")]
        pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static,
            embassy_stm32::peripherals::USART2, embassy_stm32::peripherals::DMA2_CH1>;

        #[cfg(feature = "with-serial-port-1")]
        pub type UartPort1TxControllerRef = crate::board::ControllerRef<UartPort1TxDevice>;

        #[cfg(feature = "with-serial-port-1")]
        pub use crate::board::io::uart_port1::UartPort1RxInputStream;

        #[cfg(feature = "with-spi")]
        pub(crate) type Spi1 = embassy_stm32::spi::Spi<'static,
            embassy_stm32::peripherals::SPI1,
            embassy_stm32::peripherals::DMA1_CH4, embassy_stm32::peripherals::DMA1_CH3>;

        #[cfg(feature = "with-laser")]
        pub type PwmLaser = SimplePwm<'static, embassy_stm32::peripherals::TIM16>;

        pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
        pub use embassy_stm32::adc::Instance as AdcTrait;
        pub use embassy_stm32::adc::AdcPin as AdcPinTrait;
        pub type AdcHotendHotbedPeripheral = embassy_stm32::peripherals::ADC1;
        pub type AdcHotendHotbed = AdcImpl<AdcHotendHotbedPeripheral>;
        pub type AdcHotendPeripheral = AdcHotendHotbedPeripheral;
        pub type AdcHotbedPeripheral = AdcHotendHotbedPeripheral;
        pub type AdcHotend = AdcHotendHotbed;
        pub type AdcHotbed = AdcHotendHotbed;

        pub type AdcHotendPin = embassy_stm32::peripherals::PA0;
        pub type AdcHotbedPin = embassy_stm32::peripherals::PC4;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature="with-spi", feature="with-sdcard"))] {
        pub type SpiCardDevice = Spi1;
        pub type Spi = Spi1;
        pub type SpiDeviceRef = crate::board::ControllerRef<Spi>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-sdcard")] {
        pub type SpiCardDeviceRef = crate::board::ControllerRef<Spi>;
        pub type SpiCardCSPin = Output<'static, embassy_stm32::peripherals::PA4>;
    }
}

pub use embassy_stm32::timer::CaptureCompare16bitInstance as PwmTrait;
pub type PwmImpl<TimPeri> = embassy_stm32::timer::simple_pwm::SimplePwm<'static, TimPeri>;

pub type PwmServo = SimplePwm<'static, embassy_stm32::peripherals::TIM2>;



pub type PwmFan0Fan1HotendHotbed = SimplePwm<'static, embassy_stm32::peripherals::TIM3>;

pub type PwmFanLayer = PwmFan0Fan1HotendHotbed;
pub type PwmFanExtra1 = PwmFan0Fan1HotendHotbed;
pub type PwmHotend = PwmFan0Fan1HotendHotbed;
pub type PwmHotbed = PwmFan0Fan1HotendHotbed;

pub type PwmChannel = embassy_stm32::timer::Channel;

pub type Watchdog = wdg::IndependentWatchdog<'static,
    embassy_stm32::peripherals::IWDG
>;

#[allow(non_camel_case_types)]
pub type DISPLAY_SER_RST_OUTPUT = Output<'static, embassy_stm32::peripherals::PC1>;
//pub type DISPLAY_SER_RST_OUTPUT = Output<'static>;
#[allow(non_camel_case_types)]
pub type DISPLAY_SER_CS_OUTPUT = Output<'static, embassy_stm32::peripherals::PB0>;
//pub type DISPLAY_SER_CS_OUTPUT = Output<'static>;

#[allow(non_camel_case_types)]
pub type DISPLAY_SER_DC_OUTPUT = Output<'static, embassy_stm32::peripherals::PA4>;
//pub type DISPLAY_SER_DC_OUTPUT = Output<'static>;


#[cfg(feature = "ili9341_spi")]
pub struct DisplayDevice {
    pub interface: SpiControllerRef,
    pub rst: DISPLAY_SER_RST_OUTPUT,
    pub cs: DISPLAY_SER_CS_OUTPUT,
    pub dc: DISPLAY_SER_DC_OUTPUT,
}

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

#[cfg(feature = "with-fan-layer")]
pub struct FanLayerPeripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmFanLayer>,
    pub power_channel: PwmChannel,
}

#[cfg(feature = "with-fan-extra-1")]
pub struct FanExtra1Peripherals {
    pub power_pwm: printhor_hwa_common::ControllerRef<PwmFanExtra1>,
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

    pub x_endstop_pin: Input<'static, embassy_stm32::peripherals::PC0>,
    pub y_endstop_pin: Input<'static, embassy_stm32::peripherals::PC1>,
    pub z_endstop_pin: Input<'static, embassy_stm32::peripherals::PC2>,
    pub e_endstop_pin: Input<'static, embassy_stm32::peripherals::PC15>,

    pub x_step_pin: Output<'static, embassy_stm32::peripherals::PB13>,
    pub y_step_pin: Output<'static, embassy_stm32::peripherals::PB10>,
    pub z_step_pin: Output<'static, embassy_stm32::peripherals::PB0>,
    pub e_step_pin: Output<'static, embassy_stm32::peripherals::PB3>,

    pub x_dir_pin: Output<'static, embassy_stm32::peripherals::PB12>,
    pub y_dir_pin: Output<'static, embassy_stm32::peripherals::PB2>,
    pub z_dir_pin: Output<'static, embassy_stm32::peripherals::PC5>,
    pub e_dir_pin: Output<'static, embassy_stm32::peripherals::PB4>,
}
/*
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
*/

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
