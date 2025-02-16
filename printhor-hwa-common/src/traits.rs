#[allow(unused)]
use crate as hwa;

cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "with-serial-usb",
        feature = "with-serial-port-1",
        feature = "with-serial-port-2",
        feature = "with-sd-card",
    ))] {
        pub use async_gcode::ByteStream as GCodeByteStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub trait MotionPinsTrait {
            fn enable_all_steppers(&mut self) {
                self.set_enabled(hwa::CoordSel::all_axis(), true)
            }
            #[cfg(feature = "with-x-axis")]
            fn enable_x_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::X, true)
            }
            #[cfg(feature = "with-y-axis")]
            fn enable_y_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::Y, true)
            }
            #[cfg(feature = "with-z-axis")]
            fn enable_z_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::Z, true)
            }
            #[cfg(feature = "with-e-axis")]
            fn enable_e_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::E, true)
            }
            #[cfg(feature = "with-x-axis")]
            fn disable_x_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::X, false)
            }
            #[cfg(feature = "with-y-axis")]
            fn disable_y_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::Y, false)
            }
            #[cfg(feature = "with-z-axis")]
            fn disable_z_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::Z, false)
            }
            #[cfg(feature = "with-e-axis")]
            fn disable_e_stepper(&mut self) {
                self.set_enabled(hwa::CoordSel::E, false)
            }
            fn disable_all_steppers(&mut self) {
                self.set_enabled(hwa::CoordSel::all_axis(), false)
            }

            fn disable(&mut self, _channels: hwa::CoordSel)
            {
                self.set_enabled(hwa::CoordSel::all_axis(), false)
            }

            fn set_enabled(&mut self, _channels: hwa::CoordSel, _enabled: bool);
            fn set_forward_direction(&mut self, _channels: hwa::CoordSel, _mask: hwa::CoordSel);
            fn step_toggle(&mut self, _channels: hwa::CoordSel);
            fn step_high(&mut self, _channels: hwa::CoordSel);
            fn step_low(&mut self, _channels: hwa::CoordSel);
            fn endstop_triggered(&mut self, _channels: hwa::CoordSel) -> bool;
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-trinamic")] {
        pub trait TrinamicUartTrait {
            fn read_until_idle(&mut self, buffer: &mut [u8]) -> impl core::future::Future<Output=Result<usize, hwa::uart::SerialError>>;
            fn write(&mut self,buffer: &[u8]) -> impl core::future::Future<Output=Result<(), hwa::uart::SerialError>>;
            fn flush(&mut self) -> impl core::future::Future<Output=Result<(), hwa::uart::SerialError>>;
        }


    }
}


cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "with-hot-end", feature = "with-hot-bed",
        feature = "with-fan-layer", feature="with-fan-extra-1",
        feature = "with-laser", feature = "with-probe"
    ))] {
        pub use embedded_hal_0::Pwm;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {

        /// A work in progress unified trait for `ADC`s
        pub trait UnifiedAdc16 {

            type VRefPin;
            type SamplePin;

            fn read_vref(&mut self) -> impl core::future::Future<Output = Result<u16, ()>> {
                async {
                    Err(())
                }
            }

            fn read_adc(&mut self, pin: &mut Self::SamplePin) -> impl core::future::Future<Output = u16>;
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub use embedded_sdmmc::BlockDevice as SDBlockDevice;

        pub trait AsyncSDBlockDevice: embedded_sdmmc::BlockDevice {
            fn do_retain(&self) -> impl core::future::Future<Output = Result<(), ()>>;
            fn do_release(&self) -> Result<(), ()>;
        }
    }
}
