//! device definitions for {BOARD} ({MCU})

#[allow(unused)]
use printhor_hwa_common as hwa;

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type USBDrv = embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB_OTG_FS>;
        pub use crate::board::io::serial_usb::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-1")] {
        pub type SerialPort1Tx = hwa::SerialTxWrapper<embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>>;
        pub type SerialPort1Rx = super::io::serial_port_1::SerialPort1RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-2")] {

        pub type SerialPort2Tx = hwa::SerialTxWrapper<embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>>;
        pub type SerialPort2Rx = super::io::serial_port_2::SerialPort2RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-trinamic")] {
        pub type TrinamicUart = crate::board::comm::trinamic::SingleWireSoftwareUart;
        //pub type TMCUartCh1Pin = embassy_stm32::peripherals::PD5;
        //pub type TMCUartCh2Pin = embassy_stm32::peripherals::PD1;
        //pub type TMCUartCh3Pin = embassy_stm32::peripherals::PD4;
        //pub type TMCUartCh4Pin = embassy_stm32::peripherals::PD9;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-spi")] {
        pub type Spi1 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
        //pub type Spi2 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
        pub type Spi3 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        //pub type SpiLCDCSPin = embassy_stm32::gpio::Output<'static>;
        pub type SpiCardCSPin = embassy_stm32::gpio::Output<'static>;
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

pub type Watchdog =
    embassy_stm32::wdg::IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>;


cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub struct StepActuator {
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

        impl hwa::traits::StepActuatorTrait for StepActuator {
            fn set_enabled(&mut self, _channels: hwa::CoordSel, _enabled: bool) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    if _enabled {
                        let _ = self.x_enable_pin.set_low();
                    }
                    else {
                        let _ = self.x_enable_pin.set_high();
                    }
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    if _enabled {
                        let _ = self.y_enable_pin.set_low();
                    }
                    else {
                        let _ = self.y_enable_pin.set_high();
                    }
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    if _enabled {
                        let _ = self.z_enable_pin.set_low();
                    }
                    else {
                        let _ = self.z_enable_pin.set_high();
                    }
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    if _enabled {
                        let _ = self.e_enable_pin.set_low();
                    }
                    else {
                        let _ = self.e_enable_pin.set_high();
                    }
                }
            }

            fn set_forward_direction(&mut self, _channels: hwa::CoordSel, _mask: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _mask.contains(hwa::CoordSel::X) {
                    if _channels.contains(hwa::CoordSel::X) {
                        let _ = self.x_dir_pin.set_high();
                    }
                    else {
                        let _ = self.x_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-y-axis")]
                if _mask.contains(hwa::CoordSel::Y) {
                    if _channels.contains(hwa::CoordSel::Y)  {
                        let _ = self.y_dir_pin.set_high();
                    }
                    else {
                        let _ = self.y_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-z-axis")]
                if _mask.contains(hwa::CoordSel::Z) {
                    if _channels.contains(hwa::CoordSel::Z)  {
                        let _ = self.z_dir_pin.set_high();
                    }
                    else {
                        let _ = self.z_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-e-axis")]
                if _mask.contains(hwa::CoordSel::E) {
                    if _channels.contains(hwa::CoordSel::E)  {
                        let _ = self.e_dir_pin.set_high();
                    }
                    else {
                        let _ = self.e_dir_pin.set_low();
                    }
                }
            }

            fn step_toggle(&mut self, _channels: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    let _ = self.x_step_pin.toggle();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    let _ = self.y_step_pin.toggle();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    let _ = self.z_step_pin.toggle();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    let _ = self.e_step_pin.toggle();
                }
            }

            fn step_high(&mut self, _channels: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    let _ = self.x_step_pin.set_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    let _ = self.y_step_pin.set_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    let _ = self.z_step_pin.set_high();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    let _ = self.e_step_pin.set_high();
                }
            }

            fn step_low(&mut self, _channels: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    let _ = self.x_step_pin.set_low();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    let _ = self.y_step_pin.set_low();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    let _ = self.z_step_pin.set_low();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    let _ = self.e_step_pin.set_low();
                }
            }

            fn endstop_triggered(&mut self, _channels: hwa::CoordSel) -> bool {
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