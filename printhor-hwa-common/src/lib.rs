//! Common Hardware Abstraction types and traits
#![no_std]
cfg_if::cfg_if! {
    if #[cfg(feature = "with-log")] {
        pub use log::*;
    }
    else if #[cfg(feature = "with-defmt")] {
        pub use defmt::*;
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

use strum::EnumCount;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-ui")] {
        mod display;
        pub use display::DisplayScreenUI;
    }
}

pub mod soft_uart;

/// Represents a logical channel where the request(s) come from
#[derive(strum::EnumCount, Clone, Copy, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum CommChannel {
    #[cfg(feature = "with-serial-usb")]
    SerialUsb,
    #[cfg(feature = "with-serial-port-1")]
    SerialPort1,
    #[cfg(feature = "with-serial-port-2")]
    SerialPort2,
    /// This variant is mandatory. Used by SD prints, integration tests or marco/automation
    Internal,
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
