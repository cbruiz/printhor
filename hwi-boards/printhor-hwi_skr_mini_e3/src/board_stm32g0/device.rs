//! Device definitions for SKR Mini E3 V3 (STM32G0B1RE)
#[allow(unused)]
use printhor_hwa_common as hwa;

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type SerialUsbDriver = embassy_stm32::usb::Driver<'static, embassy_stm32::peripherals::USB>;
        pub use super::io::serial_usb::*;
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
        pub type TrinamicUart = embassy_stm32::usart::Uart<'static, embassy_stm32::mode::Async>;

        pub type TrinamicUartDevice = TrinamicUartWrapper;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-spi")] {
        pub type Spi1 = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async, embassy_stm32::spi::mode::Master>;
    }
}

#[cfg(feature = "with-laser")]
pub type PwmLaser =
    embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM16>;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        /// ADC1 shared by the hot-end (TH0: PA0) and hot-bed (THB: PC4) thermistors
        pub type HotEndHotBedAdc = super::io::adc::AdcWrapper<embassy_stm32::peripherals::ADC1>;
        pub type HotEndHotBedAdcPin = embassy_stm32::adc::AnyAdcChannel<'static, embassy_stm32::peripherals::ADC1>;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-end")] {
        pub type HotEndAdcPin = HotEndHotBedAdcPin;
        pub type HotEndPwmChannel = PwmChannel;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-hot-bed")] {
        pub type HotBedAdcPin = HotEndHotBedAdcPin;
        pub type HotBedPwmChannel = PwmChannel;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-layer")] {
        pub type FanLayerPwmChannel = PwmChannel;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-fan-extra-1")] {
        pub type FanExtra1PwmChannel = PwmChannel;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-eeprom")] {
        /// On-board AT24C32 (4KB) EEPROM on I2C1 (PB6: SCL, PB7: SDA)
        pub type EepromI2c = embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>;
        pub type Eeprom = hwa::eeprom::At24c32<EepromI2c>;
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
#[cfg(feature = "with-motion")]
use hwa::CoordSel;
#[cfg(feature = "with-trinamic")]
use crate::board_stm32g0::io::TrinamicUartWrapper;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        pub type PwmServo = embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM2>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-probe")] {
        pub type PwmProbe = PwmServo;
        pub type PwmProbeChannel = PwmChannel;
    }
}


/// TIM3 drives FAN0 (PC6, CH1), FAN1 (PC7, CH2), HE0 (PC8, CH3) and the bed (PC9, CH4)
#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-fan-layer", feature = "with-fan-extra-1"))]
pub type PwmFan0Fan1HotendHotbed =
    embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM3>;


#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed", feature = "with-probe",
    feature = "with-fan-layer", feature = "with-fan-extra-1", feature = "with-laser"))]
pub type PwmChannel = embassy_stm32::timer::Channel;

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
                if _mask.contains(CoordSel::X) {
                    if _channels.contains(CoordSel::X) {
                        let _ = self.x_dir_pin.set_high();
                    }
                    else {
                        let _ = self.x_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-y-axis")]
                if _mask.contains(CoordSel::Y) {
                    if _channels.contains(CoordSel::Y)  {
                        let _ = self.y_dir_pin.set_high();
                    }
                    else {
                        let _ = self.y_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-z-axis")]
                if _mask.contains(CoordSel::Z) {
                    if _channels.contains(CoordSel::Z)  {
                        let _ = self.z_dir_pin.set_high();
                    }
                    else {
                        let _ = self.z_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-e-axis")]
                if _mask.contains(CoordSel::E) {
                    // Inverted: forward (extrusion) is DIR low with the stock extruder motor
                    if _channels.contains(CoordSel::E)  {
                        let _ = self.e_dir_pin.set_low();
                    }
                    else {
                        let _ = self.e_dir_pin.set_high();
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
