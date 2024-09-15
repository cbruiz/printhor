//! Common Hardware Abstraction types and traits
#![allow(async_fn_in_trait)]
#![no_std]
cfg_if::cfg_if! {
    if #[cfg(feature = "with-log")] {
        pub use log::*;
    }
    else if #[cfg(feature = "with-defmt")] {
        pub use defmt::{info, debug, trace, warn, error};
    }
}

pub type InterruptControllerMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

cfg_if::cfg_if! {
    if #[cfg(feature = "executor-interrupt")] {
        pub type ControllerMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    }
    else {
        pub type ControllerMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;
    }
}

mod tracked_static_cell;

pub use tracked_static_cell::TrackedStaticCell;
mod event_bus;
pub use event_bus::*;

mod defer_channel;
pub use defer_channel::*;

mod shared_controller;
pub use shared_controller::*;

mod context;
pub use context::*;

pub use tracked_static_cell::COUNTER;

mod asynch;
pub use asynch::*;
use bitflags::bitflags;


use strum::EnumCount;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        mod thermistor;
        pub use thermistor::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-display")] {
        mod display;
        pub use display::DisplayScreenUI;
    }
}

pub mod soft_uart;
pub mod traits;

/// Represents a logical channel where the request(s) come from
#[derive(strum::EnumCount, Clone, Copy, Debug)]
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
        const SERIAL_USB_OFFSET: usize = 1;
    }
    else {
        #[allow(unused)]
        const SERIAL_USB_OFFSET: usize = 0;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-serial-port-1")] {
        #[allow(unused)]
        const SERIAL_PORT1_OFFSET: usize = SERIAL_USB_OFFSET + 1;
    }
    else {
        #[allow(unused)]
        const SERIAL_PORT1_OFFSET: usize = SERIAL_USB_OFFSET;
    }
}
impl CommChannel {
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

    pub const fn index_of(channel: CommChannel) -> usize {
        match channel {
            #[cfg(feature = "with-serial-usb")]
            CommChannel::SerialUsb => {
                SERIAL_USB_OFFSET - 1
            }
            #[cfg(feature = "with-serial-port-1")]
            CommChannel::SerialPort1 => {
                SERIAL_PORT1_OFFSET - 1
            }
            #[cfg(feature = "with-serial-port-2")]
            CommChannel::SerialPort2 => {
                SERIAL_PORT1_OFFSET
            }
            CommChannel::Internal => {
                Self::COUNT - 1
            }
        }
    }

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
    pub const fn new() -> Self { Self {} }
}


bitflags! {
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub struct StepperChannel: u8 {
        #[cfg(feature = "with-x-axis")]
        const X    = 0b00000001;
        #[cfg(feature = "with-y-axis")]
        const Y    = 0b00000010;
        #[cfg(feature = "with-z-axis")]
        const Z    = 0b00000100;
        #[cfg(feature = "with-e-axis")]
        const E    = 0b00001000;
        const UNSET  = 0b10000000;
    }
}
