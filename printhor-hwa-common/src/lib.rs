//! Common Hardware Abstraction types and traits
#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

pub type InterruptControllerMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

cfg_if::cfg_if! {
    if #[cfg(feature = "executor-interrupt")] {
        pub type ControllerMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    }
    else {
        pub type ControllerMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    }
}

mod event_bus;
pub use event_bus::*;

mod defer_channel;
pub use defer_channel::*;

mod shared_controller;
pub use shared_controller::*;

pub use printhor_hwa_common_macros::*;
pub use printhor_hwa_utils::*;

mod context;
pub use context::*;

mod asynch;
pub use asynch::*;
use bitflags::bitflags;

use strum::EnumCount;

mod persistent_state;
pub use persistent_state::PersistentState;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        mod thermistor;
        pub use thermistor::*;
    }
}

pub mod soft_uart;

/// Represents a logical channel where the request(s) come from
#[derive(strum::EnumCount, Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum CommChannel {
    /// Communication through Serial USB
    #[cfg(feature = "with-serial-usb")]
    SerialUsb,
    /// Communication through Serial Port 1
    #[cfg(feature = "with-serial-port-1")]
    SerialPort1,
    /// Communication through Serial Port 2
    #[cfg(feature = "with-serial-port-2")]
    SerialPort2,
    /// This variant is mandatory. Used by SD prints, integration tests or macro/automation.
    /// It means the source communication channel comes from programmatic and/or internal routines.
    Internal,
}

impl Default for CommChannel {
    fn default() -> Self {
        CommChannel::Internal
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-usb")] {
        #[allow(unused)]
        pub const SERIAL_USB_OFFSET: usize = 1;
    }
    else {
        #[allow(unused)]
        pub const SERIAL_USB_OFFSET: usize = 0;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        #[allow(unused)]
        pub const SERIAL_PORT1_OFFSET: usize = SERIAL_USB_OFFSET + 1;
    }
    else {
        #[allow(unused)]
        pub const SERIAL_PORT1_OFFSET: usize = SERIAL_USB_OFFSET;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-2")] {
        #[allow(unused)]
        pub const SERIAL_PORT2_OFFSET: usize = SERIAL_PORT1_OFFSET + 1;
        #[allow(unused)]
        pub const INTERNAL_PORT_OFFSET: usize = SERIAL_PORT2_OFFSET + 1;
    }
    else {
        #[allow(unused)]
        pub const SERIAL_PORT2_OFFSET: usize = SERIAL_PORT1_OFFSET;
        #[allow(unused)]
        pub const INTERNAL_PORT_OFFSET: usize = SERIAL_PORT2_OFFSET;
    }
}
impl CommChannel {
    ///
    /// Retrieves the communication channel instance associated with the specified index.
    ///
    /// # Arguments
    ///
    /// * `_idx` - An index that corresponds to a specific communication channel.
    ///
    /// # Returns
    ///
    /// Returns a `CommChannel` variant that matches the specified index.
    /// If the index does not correspond to any known communication channel,
    /// `CommChannel::Internal` is returned as a default.
    ///
    /// # Example
    ///
    /// ```
    /// use printhor_hwa_common as hwa;
    /// use hwa::CommChannel;
    ///
    /// let idx = 0;
    /// let comm_channel = CommChannel::index(idx);
    ///
    /// match comm_channel {
    ///     #[cfg(feature = "with-serial-usb")]
    ///     CommChannel::SerialUsb => println!("Index corresponds to SerialUsb"),
    ///     #[cfg(feature = "with-serial-port-1")]
    ///     CommChannel::SerialPort1 => println!("Index corresponds to SerialPort1"),
    ///     #[cfg(feature = "with-serial-port-2")]
    ///     CommChannel::SerialPort2 => println!("Index corresponds to SerialPort2"),
    ///     CommChannel::Internal => println!("Index does not match any external channel, defaulting to Internal"),
    /// }
    /// ```
    ///
    pub const fn index(_idx: usize) -> CommChannel {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-serial-usb")] {
                if _idx == 0 {
                    return CommChannel::SerialUsb
                }
            }
            else if #[cfg(feature = "with-serial-port-1")] {
                if _idx == SERIAL_USB_OFFSET {
                    return CommChannel::SerialPort1
                }
            }
            else if #[cfg(feature = "with-serial-port-2")] {
                if _idx == SERIAL_PORT1_OFFSET {
                    return CommChannel::SerialPort2
                }
            }
        }
        CommChannel::Internal
    }

    /// Retrieves the communication channel based on an index value.
    ///
    /// This method is used to obtain a `CommChannel` variant based on the provided
    /// index value. The index values are mapped to specific communication channels
    /// depending on the features enabled in the build configuration.
    ///
    /// # Arguments
    ///
    /// * `_idx` - An index value representing the communication channel to retrieve.
    ///
    /// # Returns
    ///
    /// This method returns a `CommChannel` variant corresponding to the provided index
    /// value. If the index value does not match any available communication channels,
    /// the `CommChannel::Internal` variant is returned.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::CommChannel;
    ///
    /// // Assuming the appropriate feature flags are set:
    /// #[cfg(feature = "with-serial-usb")]
    /// let channel = CommChannel::index(0);
    /// #[cfg(feature = "with-serial-usb")]
    /// assert_eq!(channel, CommChannel::SerialUsb); // Example assuming `with-serial-usb` feature is enabled
    ///
    /// let internal_channel = CommChannel::index(hwa::SERIAL_PORT2_OFFSET + 1);
    /// assert_eq!(internal_channel, CommChannel::Internal);
    /// ```
    pub const fn index_of(channel: CommChannel) -> usize {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            CommChannel::SerialUsb => SERIAL_USB_OFFSET - 1,
            #[cfg(feature = "with-serial-port-1")]
            CommChannel::SerialPort1 => SERIAL_PORT1_OFFSET - 1,
            #[cfg(feature = "with-serial-port-2")]
            CommChannel::SerialPort2 => SERIAL_PORT1_OFFSET,
            CommChannel::Internal => Self::COUNT - 1,
        }
    }

    /// Provides the number of communication channels available.
    ///
    /// The `count` method is useful to retrieve the total number of
    /// communication channels defined by the `CommChannel` enum.
    /// This count includes all variants of the enum based on the features
    /// enabled in the build configuration.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::CommChannel;
    ///
    /// let total_channels = CommChannel::count();
    /// println!("Total number of communication channels: {}", total_channels);
    /// ```
    ///
    /// # Returns
    ///
    /// This method returns a constant `usize` value representing the total
    /// number of communication channels available.
    pub const fn count() -> usize {
        Self::COUNT
    }
}

// A dummy empty structure to model whether a device is not necessary.
// This is zero-cost in memory but useful to maintain a common method signature across boards
#[allow(unused)]
pub struct NoDevice;
impl NoDevice {
    #[allow(unused)]
    pub const fn new() -> Self {
        Self {}
    }
}

bitflags! {
    /// A bitmask structure representing different stepper channels.
    ///
    /// This structure uses bitflags to define constant values for each
    /// stepper channel. Each flag represents a specific axis or state.
    ///
    /// Flags:
    /// - `X`: Represents the X-axis stepper channel. This flag is only available if the `with-x-axis` feature is enabled.
    /// - `Y`: Represents the Y-axis stepper channel. This flag is only available if the `with-y-axis` feature is enabled.
    /// - `Z`: Represents the Z-axis stepper channel. This flag is only available if the `with-z-axis` feature is enabled.
    /// - `E`: Represents the E-axis stepper channel. This flag is only available if the `with-e-axis` feature is enabled.
    /// - `UNSET`: Represents an unset or undefined channel. This flag is always available.
    ///
    /// Example usage:
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::StepperChannel;
    ///
    /// // Assuming the appropriate feature flags are set...
    /// let mut channels = StepperChannel::empty();
    ///
    /// // Check if a specific channel is set:
    /// #[cfg(feature = "with-x-axis")]
    /// if channels.contains(StepperChannel::X) {
    ///     // Do something with the X channel
    /// }
    ///
    /// // Set another channel:
    /// #[cfg(feature = "with-z-axis")]
    /// channels.insert(StepperChannel::Z);
    ///
    /// // Remove a channel:
    /// #[cfg(feature = "with-y-axis")]
    /// channels.remove(StepperChannel::Y);
    ///
    /// // Check if no channel is set:
    /// if channels.contains(StepperChannel::UNSET) {
    ///     // Handle unset channel
    /// }
    /// ```
    #[derive(Clone, Copy, PartialEq, Debug)]
    pub struct StepperChannel: u8 {
        /// Represents the X-axis stepper channel. This flag is only available if the `with-x-axis` feature is enabled.
        #[cfg(feature = "with-x-axis")]
        const X    = 0b00000001;
        /// Represents the Y-axis stepper channel. This flag is only available if the `with-y-axis` feature is enabled.
        #[cfg(feature = "with-y-axis")]
        const Y    = 0b00000010;
        /// Represents the Z-axis stepper channel. This flag is only available if the `with-z-axis` feature is enabled.
        #[cfg(feature = "with-z-axis")]
        const Z    = 0b00000100;
        /// Represents the E-axis stepper channel. This flag is only available if the `with-e-axis` feature is enabled.
        #[cfg(feature = "with-e-axis")]
        const E    = 0b00001000;
        /// Represents an unset or undefined channel. This flag is always available.
        const UNSET  = 0b10000000;
    }
}
