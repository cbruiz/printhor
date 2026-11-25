//! The printhor traits (Work In Progress).
//! This module expects to define or re-export all the HWI devices interfaces shall satisfy
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
        pub trait StepActuatorTrait {
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

            fn disable(&mut self, channels: hwa::CoordSel)
            {
                self.set_enabled(channels, false)
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
            /// Selects the TMC stepper to address given coordinate axis.
            ///
            /// Normally, single uart is shared for all them. However, if more than one is needed, this
            /// method selects the one active
            fn select_stepper_of_axis(&mut self, _channel: hwa::CoordSel) -> Result<(), ()> {
                Ok(())
            }
            /// Gets the TMC Adddress of given coordinate axis
            fn get_tmc_address(&self, _channel: hwa::CoordSel) -> Result<u8, ()>;
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
            /// What [Self::do_lock] yields and [Self::do_hold] keeps (e.g. the guard of the SPI bus)
            type Lock;
            /// Retains the device (e.g. its SPI bus) in two steps: since embedded-sdmmc 0.10 the
            /// device is only reachable inside `VolumeManager::device(|d| ..)`, so this future
            /// must not borrow `self` (it is awaited after that borrow ends), and the lock it
            /// yields is then kept by the device itself with [Self::do_hold].
            fn do_lock(&self) -> impl core::future::Future<Output = Option<Self::Lock>> + use<Self>;
            /// Keeps the lock taken by [Self::do_lock] until [Self::do_release]
            fn do_hold(&self, lock: Self::Lock) -> Result<(), ()>;
            fn do_release(&self) -> Result<(), ()>;
            /// Forgets the card state, so that the next access initializes the card again (a card
            /// inserted, replaced or not ready when the firmware started). No-op by default.
            fn reset_card(&mut self) {}
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-eeprom")] {
        /// Errors of an [AsyncEeprom] device
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
        pub enum EepromError {
            /// The access goes beyond the capacity of the device
            OutOfBounds,
            /// The device did not answer (or answered with an error)
            Bus,
            /// The device did not finish an internal write cycle in time
            Timeout,
        }

        /// A non-volatile byte-addressable memory (EEPROM or alike), used to persist the settings
        /// (M500/M501).
        ///
        /// Boards with such a memory implement this trait (see [crate::eeprom::I2cEeprom24x] for
        /// the common I2C 24Cxx chips) and export it through
        /// [crate::HwiContract::EepromMutexStrategy]. Every operation is async: the bus transfers
        /// and the internal write cycles never block the executor.
        pub trait AsyncEeprom {
            /// The capacity of the memory in bytes
            fn capacity(&self) -> usize;

            /// Reads `buffer.len()` bytes starting at `offset`
            fn read(&mut self, offset: usize, buffer: &mut [u8]) -> impl core::future::Future<Output = Result<(), EepromError>>;

            /// Writes `data` starting at `offset`. Implementations should skip the unchanged
            /// regions to reduce the wear and the latency.
            fn write(&mut self, offset: usize, data: &[u8]) -> impl core::future::Future<Output = Result<(), EepromError>>;
        }
    }
}

#[cfg(test)]
mod coverage_tests;
