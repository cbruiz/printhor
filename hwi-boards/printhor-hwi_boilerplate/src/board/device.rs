//! device definitions for {BOARD} ({MCU})
#[allow(unused)]
use printhor_hwa_common as hwa;

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-usb")] {
        pub type SerialUsbDriver = compile_error!("Provide me");
        pub use super::io::usbserial::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-1")] {
        pub type SerialPort1Tx = compile_error!("Provide me");
        pub type SerialPort1Rx = compile_error!("Provide me");
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-serial-port-2")] {
        pub type SerialPort2Tx = compile_error!("Provide me");
        pub type SerialPort2Rx = compile_error!("Provide me");
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
        pub type PsOnPin = compile_error!("Provide me");
    }
}

pub type Watchdog = compile_error!("Provide me");

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub struct StepActuator {
            #[cfg(feature = "with-x-axis")]
            pub x_enable_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-y-axis")]
            pub y_enable_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-z-axis")]
            pub z_enable_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-e-axis")]
            pub e_enable_pin: compile_error!("Provide me"),

            #[cfg(feature = "with-x-axis")]
            pub x_endstop_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-y-axis")]
            pub y_endstop_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-z-axis")]
            pub z_endstop_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-e-axis")]
            pub e_endstop_pin: compile_error!("Provide me"),

            #[cfg(feature = "with-x-axis")]
            pub x_step_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-y-axis")]
            pub y_step_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-z-axis")]
            pub z_step_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-e-axis")]
            pub e_step_pin: compile_error!("Provide me"),

            #[cfg(feature = "with-x-axis")]
            pub x_dir_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-y-axis")]
            pub y_dir_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-z-axis")]
            pub z_dir_pin: compile_error!("Provide me"),
            #[cfg(feature = "with-e-axis")]
            pub e_dir_pin: compile_error!("Provide me"),
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
