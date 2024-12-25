#[allow(unused)]
use printhor_hwa_common as hwa;

pub type Watchdog = embassy_stm32::wdg::IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1Tx = hwa::SerialTxWrapper<embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>>;
        pub type SerialPort1Rx = super::io::uart_port1::UartPort1RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-spi")] {
        pub type Spi = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub struct MotionPins {
            pub all_enable_pin: embassy_stm32::gpio::Output<'static>, // D8

            pub x_endstop_pin: embassy_stm32::gpio::Input<'static>, // D9
            pub y_endstop_pin: embassy_stm32::gpio::Input<'static>, // D10
            pub z_endstop_pin: embassy_stm32::gpio::Input<'static>, // D11

            pub x_step_pin: embassy_stm32::gpio::Output<'static>, // D2
            pub y_step_pin: embassy_stm32::gpio::Output<'static>, // D3
            pub z_step_pin: embassy_stm32::gpio::Output<'static>, // D4

            pub x_dir_pin: embassy_stm32::gpio::Output<'static>, // D5
            pub y_dir_pin: embassy_stm32::gpio::Output<'static>, // D6
            pub z_dir_pin: embassy_stm32::gpio::Output<'static>, // D7
        }

        impl hwa::traits::MotionPinsTrait for MotionPins {
            fn set_enabled(&mut self, _channels: hwa::StepperChannel, _enabled: bool) {
                if _enabled {
                    self.all_enable_pin.set_low();
                }
                else {
                    self.all_enable_pin.set_high();
                }
            }
            fn set_forward_direction(&mut self, _channels: hwa::StepperChannel, _mask: hwa::StepperChannel) {
                #[cfg(feature = "with-x-axis")]
                if _mask.contains(hwa::StepperChannel::X) {
                    if _channels.contains(hwa::StepperChannel::X) {
                        self.x_dir_pin.set_high();
                    }
                    else {
                        self.x_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-y-axis")]
                if _mask.contains(hwa::StepperChannel::Y) {
                    if _channels.contains(hwa::StepperChannel::Y) {
                        self.y_dir_pin.set_high();
                    }
                    else {
                        self.y_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-z-axis")]
                if _mask.contains(hwa::StepperChannel::Z) {
                    if _channels.contains(hwa::StepperChannel::Z) {
                        self.z_dir_pin.set_high();
                    }
                    else {
                        self.z_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-e-axis")]
                if _mask.contains(hwa::StepperChannel::E) {
                    unreachable!()
                }
            }
            fn step_toggle(&mut self, _channels: hwa::StepperChannel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::StepperChannel::X) {
                    self.x_step_pin.toggle();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::StepperChannel::Y) {
                    self.y_step_pin.toggle();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::StepperChannel::Z) {
                    self.z_step_pin.toggle();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::StepperChannel::E) {
                    unreachable!()
                }
            }
            fn step_high(&mut self, _channels: hwa::StepperChannel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::StepperChannel::X) {
                    self.x_step_pin.set_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::StepperChannel::Y) {
                    self.y_step_pin.set_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::StepperChannel::Z) {
                    self.z_step_pin.set_high();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::StepperChannel::E) {
                    unreachable!()
                }
            }
            fn step_low(&mut self, _channels: hwa::StepperChannel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::StepperChannel::X) {
                    self.x_step_pin.set_low();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::StepperChannel::Y) {
                    self.y_step_pin.set_low();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::StepperChannel::Z) {
                    self.z_step_pin.set_low();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::StepperChannel::E) {
                    unreachable!()
                }
            }
            fn endstop_triggered(&mut self, _channels: hwa::StepperChannel) -> bool {
                #[allow(unused_mut)]
                let mut triggered = false;
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::StepperChannel::X) {
                    triggered |= self.x_endstop_pin.is_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::StepperChannel::Y) {
                    triggered |= self.y_endstop_pin.is_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::StepperChannel::Z) {
                    triggered |= self.z_endstop_pin.is_high();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::StepperChannel::E) {
                    unreachable!()
                }
                triggered
            }
        }
    }
}

cfg_if::cfg_if!{
    if #[cfg(feature = "with-laser")] {
        pub type PwmLaser = embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM1>;
        pub type LaserPwmChannel = embassy_stm32::timer::Channel;
    }
}
