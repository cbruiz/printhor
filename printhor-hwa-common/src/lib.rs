//! Common Hardware Abstraction types and traits
#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;
extern crate core;

#[cfg(feature = "with-motion")]
pub mod kinematics;
pub mod math;

pub use printhor_hwa_common_macros::*;
pub use printhor_hwa_utils::*;
use strum::EnumCount;

mod event_bus;
pub use event_bus::*;

mod defer_channel;
pub use defer_channel::*;

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion-broadcast")] {
        mod motion_broadcast;
        pub use motion_broadcast::*;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-serial-usb", feature = "with-serial-port-1", feature = "with-serial-port-2"))] {
        mod async_utils;
        pub use async_utils::*;
    }
}

mod persistent_state;
pub use persistent_state::PersistentState;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        mod thermistor;
        pub use thermistor::*;
    }
}

mod contract;
mod event_bus_channel;
pub mod soft_uart;
pub mod traits;

pub use contract::{HwiContext, HwiContract};

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub mod sd_card;
    }
}

pub use crate::math::CoordSel;
pub use event_bus_channel::*;

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
    /// This variant is mandatory. Used by integration tests or macro/automation.
    /// It means the source communication channel comes from programmatic and/or internal routines.
    Internal,
    /// This variant is mandatory. Used by SD prints. Output is discarded.
    Sink,
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
    ///     CommChannel::Sink => println!("Index does not match any external channel, defaulting to Internal"),
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
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-serial-port-1")] {
                if _idx == SERIAL_USB_OFFSET {
                    return CommChannel::SerialPort1
                }
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-serial-port-2")] {
                if _idx == SERIAL_PORT1_OFFSET {
                    return CommChannel::SerialPort2
                }
            }
        }
        if _idx == Self::COUNT - 2 {
            CommChannel::Internal
        } else {
            CommChannel::Sink
        }
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
            CommChannel::Internal => Self::COUNT - 2,
            CommChannel::Sink => Self::COUNT - 1,
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
pub struct NoDevice;
impl NoDevice {
    #[allow(unused)]
    pub const fn new() -> Self {
        Self {}
    }
}

/// Covers a device directly imported from HWI
pub struct HwiResource<D> {
    inner: D,
}

impl<D> HwiResource<D> {
    pub const fn new(inner: D) -> Self {
        Self { inner }
    }
}

impl<D> RawHwiResource for HwiResource<D> {
    type Resource = D;

    fn take(self) -> D {
        self.inner
    }
}

impl<D> core::ops::Deref for HwiResource<D> {
    type Target = D;
    fn deref(&self) -> &<Self as core::ops::Deref>::Target {
        &self.inner
    }
}

impl<D> Clone for HwiResource<D>
where
    D: Clone,
{
    fn clone(&self) -> Self {
        HwiResource::new(self.inner.clone())
    }
}

impl<D> Copy for HwiResource<D> where D: Copy {}

#[cfg(test)]
mod test {
    #[allow(unused)]
    use crate as hwa;

    #[cfg(all(
        feature = "with-serial-usb",
        feature = "with-serial-port-1",
        feature = "with-serial-port-2"
    ))]
    #[test]
    fn test_some_objects() {
        let _nd = hwa::NoDevice::new();
        let ch1 = hwa::CoordSel::X;
        hwa::info!("{:?}", ch1);
        let mut ch2 = ch1.clone();
        assert_eq!(ch1, ch2);
        let ch3 = ch2;
        assert_eq!(ch2, ch3);
        ch2.set(hwa::CoordSel::Y, true);
        hwa::info!("{:?}", ch2);
        assert_ne!(ch2, ch3);

        assert_eq!(hwa::CommChannel::count(), 5);

        let c0 = hwa::CommChannel::SerialUsb;
        let c1 = hwa::CommChannel::SerialPort1;
        let c2 = hwa::CommChannel::SerialPort2;
        let c3 = hwa::CommChannel::Internal;

        assert_eq!(c0, hwa::CommChannel::index(0));
        assert_eq!(c1, hwa::CommChannel::index(1));
        assert_eq!(c2, hwa::CommChannel::index(2));
        assert_eq!(c3, hwa::CommChannel::index(3));

        assert_eq!(0, hwa::CommChannel::index_of(c0));
        assert_eq!(1, hwa::CommChannel::index_of(c1));
        assert_eq!(2, hwa::CommChannel::index_of(c2));
        assert_eq!(3, hwa::CommChannel::index_of(c3));
    }
}
