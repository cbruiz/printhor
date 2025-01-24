#[allow(unused)]
use printhor_hwa_common as hwa;

pub type Watchdog = embassy_stm32::wdg::IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG>;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        pub type SerialPort1Tx = hwa::SerialTxWrapper<embassy_stm32::usart::BufferedUartTx<'static>>;
        pub type SerialPort1Rx = super::io::uart_port1::UartPort1RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-spi")] {
        pub type Spi = embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-i2c")] {
        pub type I2c = super::io::MotionI2c;
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
            fn set_enabled(&mut self, _channels: hwa::CoordSel, _enabled: bool) {
                if _enabled {
                    self.all_enable_pin.set_low();
                }
                else {
                    self.all_enable_pin.set_high();
                }
            }
            fn set_forward_direction(&mut self, _channels: hwa::CoordSel, _mask: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _mask.contains(hwa::CoordSel::X) {
                    if _channels.contains(hwa::CoordSel::X) {
                        self.x_dir_pin.set_high();
                    }
                    else {
                        self.x_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-y-axis")]
                if _mask.contains(hwa::CoordSel::Y) {
                    if _channels.contains(hwa::CoordSel::Y) {
                        self.y_dir_pin.set_high();
                    }
                    else {
                        self.y_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-z-axis")]
                if _mask.contains(hwa::CoordSel::Z) {
                    if _channels.contains(hwa::CoordSel::Z) {
                        self.z_dir_pin.set_high();
                    }
                    else {
                        self.z_dir_pin.set_low();
                    }
                }
                #[cfg(feature = "with-e-axis")]
                if _mask.contains(hwa::CoordSel::E) {
                    unreachable!()
                }
            }
            fn step_toggle(&mut self, _channels: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    self.x_step_pin.toggle();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    self.y_step_pin.toggle();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    self.z_step_pin.toggle();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    unreachable!()
                }
            }
            fn step_high(&mut self, _channels: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    self.x_step_pin.set_high();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    self.y_step_pin.set_high();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    self.z_step_pin.set_high();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    unreachable!()
                }
            }
            fn step_low(&mut self, _channels: hwa::CoordSel) {
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    self.x_step_pin.set_low();
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    self.y_step_pin.set_low();
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    self.z_step_pin.set_low();
                }
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    unreachable!()
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
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    unreachable!()
                }
                triggered
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(feature = "with-motion", feature = "with-motion-broadcast"))] {
        pub type MotionSender = super::io::MotionI2c;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-ps-on")] {
        pub type PsOnPin = embassy_stm32::gpio::Output<'static>;
    }
}

cfg_if::cfg_if!{
    if #[cfg(feature = "with-fan-layer")] {
        pub type PwmFanLayer = embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM2>;
        pub type PwmFanChannel = embassy_stm32::timer::Channel;
    }
}

cfg_if::cfg_if!{
    if #[cfg(feature = "with-laser")] {
        pub type PwmLaser = embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM1>;
        pub type PwmLaserChannel = embassy_stm32::timer::Channel;
    }
}

cfg_if::cfg_if!{
    if #[cfg(feature = "with-probe")] {
        pub type PwmProbe = embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM3>;
        pub type PwmProbeChannel = embassy_stm32::timer::Channel;
    }
}

cfg_if::cfg_if!{
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        pub type HotEndHotBedAdc = super::io::adc::AdcWrapper<embassy_stm32::peripherals::ADC1, embassy_stm32::peripherals::DMA1_CH1>;
        pub type HotEndHotBedAdcPin = embassy_stm32::adc::AnyAdcChannel<embassy_stm32::peripherals::ADC1>;
        pub type HotEndHotBedPwm = embassy_stm32::timer::simple_pwm::SimplePwm<'static, embassy_stm32::peripherals::TIM15>;
    }
}

cfg_if::cfg_if!{
    if #[cfg(feature = "with-hot-end")] {
        //pub type HotEndAdc = HotEndHotBedAdc;
        //pub type HotEndPwm = HotEndHotBedPwm;
        pub type HotEndAdcPin = HotEndHotBedAdcPin; //embassy_stm32::peripherals::PC2;
        pub type HotEndPwmChannel = embassy_stm32::timer::Channel;
    }
}

cfg_if::cfg_if!{
    if #[cfg(feature = "with-hot-bed")] {
        //pub type HotBedAdc = HotEndHotBedAdc;
        //pub type HotBedPwm = HotEndHotBedPwm;
        pub type HotBedAdcPin = HotEndHotBedAdcPin; //embassy_stm32::peripherals::PC3;
        pub type HotBedPwmChannel = embassy_stm32::timer::Channel;
    }
}
