//! Device definitions for SKR Mini E3 V3 (STM32G0B1RE)
#[allow(unused)]
use printhor_hwa_common as hwa;

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type USBDrv = embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB>;
        pub use super::io::usb_serial::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-1")] {
        pub type UartPort1Device = embassy_stm32::usart::Uart<'static, embassy_stm32::mode::Async>;
        pub type UartPort1TxDevice = embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>;
        pub type UartPort1RxDevice = embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>;
        pub type UartPort1RingBufferedRxDevice = embassy_stm32::usart::RingBufferedUartRx<'static>;

        pub type UartPort1TxControllerRef = hwa::StandardControllerRef<hwa::SerialAsyncWrapper<UartPort1TxDevice>>;
        pub use crate::board::io::uart_port1::UartPort1RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-2")] {

        pub type UartPort2Device = embassy_stm32::usart::Uart<'static, embassy_stm32::mode::Async>;
        pub type UartPort2TxDevice = embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>;
        pub type UartPort2RxDevice = embassy_stm32::usart::UartRx<'static, embassy_stm32::mode::Async>;
        pub type UartPort2RingBufferedRxDevice = embassy_stm32::usart::RingBufferedUartRx<'static>;

        pub type UartPort2TxControllerRef = hwa::StandardControllerRef<hwa::SerialAsyncWrapper<UartPort2TxDevice>>;
        pub use crate::board::io::uart_port2::UartPort2RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-trinamic")] {
        pub type TrinamicUart = embassy_stm32::usart::Uart<'static, embassy_stm32::mode::Async>;

        pub type TrinamicUartDevice = TrinamicUartWrapper;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-spi")] {
        pub type Spi1 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
    }
}

#[cfg(feature = "with-laser")]
pub type PwmLaser =
    embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM16>;

cfg_if::cfg_if! {
    if #[cfg(any(feature="with-hot-end", feature="with-hot-end"))] {
        pub type AdcImpl<PERI> = embassy_stm32::adc::Adc<'static, PERI>;
        pub use embassy_stm32::adc::Instance as AdcTrait;
        pub use embassy_stm32::adc::AdcChannel as AdcPinTrait;
        pub use embassy_stm32::adc::VrefInt as VrefInt;
        pub type AdcHotendHotbedPeripheral = embassy_stm32::peripherals::ADC1;

    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-hot-bed")] {
        pub type AdcHotendHotbed = AdcImpl<AdcHotendHotbedPeripheral>;
        pub type AdcHotbedPeripheral = AdcHotendHotbedPeripheral;
        pub type AdcHotbedPin = embassy_stm32::peripherals::PC4;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature="with-hot-end")] {
        pub type AdcHotendPeripheral = AdcHotendHotbedPeripheral;
        pub type AdcHotend = AdcHotendHotbed;
        pub type AdcHotbed = AdcHotendHotbed;
        pub type AdcHotendPin = embassy_stm32::peripherals::PA0;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-ps-on")] {
        pub type PsOnPin = embassy_stm32::gpio::Output<'static>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature="with-spi", feature = "with-sd-card"))] {
        pub type Spi = Spi1;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub type SpiCardCSPin = embassy_stm32::gpio::Output<'static>;
    }
}
#[allow(unused)]
pub use embassy_stm32::timer::GeneralInstance4Channel as PwmTrait;
use hwa::CoordSel;
#[cfg(feature = "with-trinamic")]
use crate::board_stm32g0::io::TrinamicUartWrapper;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-probe",
    feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))]
pub type PwmImpl<TimPeri> = embassy_stm32::timer::simple_pwm::SimplePwm<'static, TimPeri>;

#[cfg(any(feature = "with-probe"))]
pub type PwmServo =
    embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM2>;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer", feature = "with-fan-extra-1"))]
pub type PwmFan0Fan1HotendHotbed =
    embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM3>;

#[cfg(any(feature ="with-fan-layer"))]
pub type PwmFanLayer = PwmFan0Fan1HotendHotbed;
#[cfg(any(feature = "with-fan-extra-1"))]
pub type PwmFanExtra1 = PwmFan0Fan1HotendHotbed;
#[cfg(any(feature = "with-hot-end"))]
pub type PwmHotend = PwmFan0Fan1HotendHotbed;
#[cfg(any(feature = "with-hot-bed"))]
pub type PwmHotbed = PwmFan0Fan1HotendHotbed;

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-probe",
    feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))]
pub type PwmChannel = embassy_stm32::timer::Channel;

pub type Watchdog =
    embassy_stm32::wdg::IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>;


cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub struct MotionPins {
            #[cfg(feature = "with-x-axis")]
            pub x_enable_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-y-axis")]
            pub y_enable_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-z-axis")]
            pub z_enable_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-e-axis")]
            pub e_enable_pin: embassy_stm32::gpio::Output<'static>,

            #[cfg(feature = "with-x-axis")]
            pub x_endstop_pin: embassy_stm32::gpio::Input<'static>,
            #[cfg(feature = "with-y-axis")]
            pub y_endstop_pin: embassy_stm32::gpio::Input<'static>,
            #[cfg(feature = "with-z-axis")]
            pub z_endstop_pin: embassy_stm32::gpio::Input<'static>,
            #[cfg(feature = "with-e-axis")]
            pub e_endstop_pin: embassy_stm32::gpio::Input<'static>,

            #[cfg(feature = "with-x-axis")]
            pub x_step_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-y-axis")]
            pub y_step_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-z-axis")]
            pub z_step_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-e-axis")]
            pub e_step_pin: embassy_stm32::gpio::Output<'static>,

            #[cfg(feature = "with-x-axis")]
            pub x_dir_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-y-axis")]
            pub y_dir_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-z-axis")]
            pub z_dir_pin: embassy_stm32::gpio::Output<'static>,
            #[cfg(feature = "with-e-axis")]
            pub e_dir_pin: embassy_stm32::gpio::Output<'static>,
        }

        impl hwa::traits::MotionPinsTrait for MotionPins {
            fn set_enabled(&mut self, _channels: CoordSel, _enabled: bool) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(CoordSel::X) {
                    if _enabled {
                        let _ = self.x_enable_pin.set_low();
                    }
                    else {
                        let _ = self.x_enable_pin.set_high();
                    }
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(CoordSel::Y) {
                    if _enabled {
                        let _ = self.y_enable_pin.set_low();
                    }
                    else {
                        let _ = self.y_enable_pin.set_high();
                    }
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(CoordSel::Z) {
                    if _enabled {
                        let _ = self.z_enable_pin.set_low();
                    }
                    else {
                        let _ = self.z_enable_pin.set_high();
                    }
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(CoordSel::E) {
                    if _enabled {
                        let _ = self.e_enable_pin.set_low();
                    }
                    else {
                        let _ = self.e_enable_pin.set_high();
                    }
                }
            }

            fn set_forward_direction(&mut self, _channels: CoordSel, _mask: CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(CoordSel::X) {
                    if _mask.contains(CoordSel::X) {
                        let _ = self.x_dir_pin.set_high();
                    }
                    else {
                        let _ = self.x_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(CoordSel::Y) {
                    if _mask.contains(CoordSel::Y)  {
                        let _ = self.y_dir_pin.set_high();
                    }
                    else {
                        let _ = self.y_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(CoordSel::Z) {
                    if _mask.contains(CoordSel::Z)  {
                        let _ = self.z_dir_pin.set_high();
                    }
                    else {
                        let _ = self.z_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(CoordSel::E) {
                    if _mask.contains(CoordSel::E)  {
                        let _ = self.e_dir_pin.set_high();
                    }
                    else {
                        let _ = self.e_dir_pin.set_low();
                    }
                }
            }

            fn step_toggle(&mut self, _channels: CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(CoordSel::X) {
                    let _ = self.x_step_pin.toggle();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(CoordSel::Y) {
                    let _ = self.y_step_pin.toggle();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(CoordSel::Z) {
                    let _ = self.z_step_pin.toggle();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(CoordSel::E) {
                    let _ = self.e_step_pin.toggle();
                }
            }

            fn step_high(&mut self, _channels: CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(CoordSel::X) {
                    let _ = self.x_step_pin.set_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(CoordSel::Y) {
                    let _ = self.y_step_pin.set_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(CoordSel::Z) {
                    let _ = self.z_step_pin.set_high();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(CoordSel::E) {
                    let _ = self.e_step_pin.set_high();
                }
            }

            fn step_low(&mut self, _channels: CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(CoordSel::X) {
                    let _ = self.x_step_pin.set_low();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(CoordSel::Y) {
                    let _ = self.y_step_pin.set_low();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(CoordSel::Z) {
                    let _ = self.z_step_pin.set_low();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(CoordSel::E) {
                    let _ = self.e_step_pin.set_low();
                }
            }

            fn endstop_triggered(&mut self, _channels: CoordSel) -> bool {
                #[allow(unused_mut)]
                let mut triggered = false;
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    triggered |= self.x_endstop_pin.is_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    triggered |= self.y_endstop_pin.is_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    triggered |= self.z_endstop_pin.is_high();
                }
                triggered
            }

        }
    }
}
