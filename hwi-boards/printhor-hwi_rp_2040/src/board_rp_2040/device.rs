//! device definitions for {BOARD} ({MCU})
use printhor_hwa_common as hwa;
use hwa::HwiContract;


cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type SerialUsbDriver = embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>;
        pub use super::io::serial_usb::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-1")] {
        pub type SerialPort1Tx = embassy_rp::uart::UartTx<'static, embassy_rp::peripherals::UART0, embassy_rp::uart::Async>;
        pub type SerialPort1Rx = super::io::serial_port_1::SerialPort1RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-2")] {
        pub type SerialPort2Tx = embassy_rp::uart::UartTx<'static, embassy_rp::peripherals::UART1, embassy_rp::uart::Async>;
        pub type SerialPort2Rx = super::io::serial_port_2::SerialPort2RxInputStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-trinamic")] {
        pub type TrinamicUart = compile_error!("Provide me");
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-spi")] {
        pub type Spi1 = compile_error!("Provide me");
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub type SpiCardCSPin = compile_error!("Provide me");
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-ps-on")] {
        pub type PsOnPin = embassy_rp::gpio::Output<'static>;
    }
}

/// Wraps the gap between stm32 and rp8020 Watchdog API specs
pub struct WatchdogAdapter {
    inner: embassy_rp::watchdog::Watchdog,
}

impl WatchdogAdapter {
    pub fn new(peri: embassy_rp::peripherals::WATCHDOG) -> Self {
        Self {
            inner: embassy_rp::watchdog::Watchdog::new(peri),
        }
    }
    #[inline(always)]
    pub fn unleash(&mut self) {
        self.inner.start(embassy_time::Duration::from_micros(
            crate::Contract::WATCHDOG_TIMEOUT_US as u64,
        ))
    }
    #[inline(always)]
    pub fn pet(&mut self) {
        self.inner.feed();
    }
}

pub type Watchdog = WatchdogAdapter;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub struct StepActuator {
            #[cfg(any(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis", feature = "with-e-axis"))]
            pub all_enable_pin: embassy_rp::gpio::Output<'static>,
            #[cfg(feature = "with-x-axis")]
            pub x_endstop_pin: embassy_rp::gpio::Input<'static>,
            #[cfg(feature = "with-y-axis")]
            pub y_endstop_pin: embassy_rp::gpio::Input<'static>,
            #[cfg(feature = "with-z-axis")]
            pub z_endstop_pin: embassy_rp::gpio::Input<'static>,
            #[cfg(feature = "with-e-axis")]
            pub e_endstop_pin: embassy_rp::gpio::Input<'static>,

            #[cfg(feature = "with-x-axis")]
            pub x_step_pin: embassy_rp::gpio::Output<'static>,
            #[cfg(feature = "with-y-axis")]
            pub y_step_pin: embassy_rp::gpio::Output<'static>,
            #[cfg(feature = "with-z-axis")]
            pub z_step_pin: embassy_rp::gpio::Output<'static>,
            #[cfg(feature = "with-e-axis")]
            pub e_step_pin: embassy_rp::gpio::Output<'static>,

            #[cfg(feature = "with-x-axis")]
            pub x_dir_pin: embassy_rp::gpio::Output<'static>,
            #[cfg(feature = "with-y-axis")]
            pub y_dir_pin: embassy_rp::gpio::Output<'static>,
            #[cfg(feature = "with-z-axis")]
            pub z_dir_pin: embassy_rp::gpio::Output<'static>,
            #[cfg(feature = "with-e-axis")]
            pub e_dir_pin: embassy_rp::gpio::Output<'static>,
        }

        impl hwa::traits::StepActuatorTrait for StepActuator {
            fn set_enabled(&mut self, _channels: hwa::CoordSel, _enabled: bool) {
                #[allow(unused_mut)]
                let mut hit_channel = hwa::CoordSel::empty();
                
                #[cfg(feature = "with-e-axis")]
                if _channels.contains(hwa::CoordSel::E) {
                    hit_channel |= hwa::CoordSel::E;
                }
                #[cfg(feature = "with-x-axis")]
                if _channels.contains(hwa::CoordSel::X) {
                    hit_channel |= hwa::CoordSel::X;
                }
                #[cfg(feature = "with-y-axis")]
                if _channels.contains(hwa::CoordSel::Y) {
                    hit_channel |= hwa::CoordSel::Y;
                }
                #[cfg(feature = "with-z-axis")]
                if _channels.contains(hwa::CoordSel::Z) {
                    hit_channel |= hwa::CoordSel::Z;
                }
                
                if !hit_channel.is_empty() {
                    if _enabled {
                        let _ = self.all_enable_pin.set_low();
                    }
                    else {
                        let _ = self.all_enable_pin.set_high();
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
