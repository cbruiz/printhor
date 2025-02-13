//! # Defer Channel Module
//!
//! This module provides the necessary abstractions and implementations to handle deferrable events within a concurrent system.
//!
//! The core components of this module include:
//!
//! - `DeferAction`: Enum representing various actions that can be deferred.
//! - `DeferEvent`: Enum representing different states of defer actions (e.g., awaiting or completed).
//! - `DeferChannelRef`: A struct that encapsulates a static reference to a `DeferChannelChannelType`, promoting safe concurrency practices.
//!
//! ## Overview
//!
//! The purpose of this module is to manage actions that can be deferred and later executed, tracked, or monitored.
//! It is particularly useful in systems that require precise control over the execution flow,
//! such as motion control systems or environments where tasks need to be queued and executed in an orderly manner.
//! Using this API, you can define different actions, wrap them in events, and handle these events in a concurrent setup.
//!
//! ## Key Components
//!
//! - `DeferAction`: Actions that can be deferred, such as `Homing`, `RapidMove`, `LinearMove`, `Dwell`, `HotEndTemperature`, and `HotbedTemperature`.
//! - `DeferEvent`: Representations of different states of defer actions (awaiting execution or completed).
//! - `DeferChannelRef`: A reference to a defer channel that handles communication in a thread-safe manner.
//!
//! ## Example Usage
//!
//! ```rust
//!
//! // Example usage of DeferEvent
//! #[cfg(feature = "with-motion")]
//! {
//!     use printhor_hwa_common as hwa;
//!     use hwa::{GenericDeferChannel, DeferAction, DeferEvent, CommChannel};
//!     use hwa::make_static_ref;
//!     // Define the mutex type
//!     type MutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
//!
//!     let event1 = DeferEvent::AwaitRequested(
//!         DeferAction::Homing, CommChannel::default()
//!     );
//!     let event2 = DeferEvent::Completed(
//!         DeferAction::RapidMove, CommChannel::default()
//!     );
//!
//!     let defer_channel: hwa::GenericDeferChannel<MutexType> = GenericDeferChannel::new(
//!         make_static_ref!(
//!             "DeferChannel",
//!             hwa::DeferChannelChannelType<MutexType>,
//!             hwa::DeferChannelChannelType::new()
//!         )
//!     );
//!
//!     let sender = defer_channel.sender();
//!
//!     // [...]
//!     async fn do_send(sender: hwa::DeferChannelChannelType<MutexType>, event: DeferEvent) {
//!         let _  = sender.send(event).await;
//!     }
//
//! }
//! ```
//!
//! ## Purpose
//!
//! This module is designed for applications that require efficient task management and concurrency control.
//! It is aimed at developers who need to defer actions and handle these deferred actions reliably.
//! By using static references and safe concurrency practices, this module ensures that deferred actions are managed
//! in a predictable and efficient manner.

use crate as hwa;
use embassy_sync::channel::Channel;
use hwa::CommChannel;

//#region "Defer Action"

///
/// `DeferAction` represents the possible actions that can be deferred in the system.
///
/// # Variants
///
/// * `Homing` - Homing action for the motion feature.
/// * `RapidMove` - Rapid movement action for the motion feature.
/// * `LinearMove` - Linear movement action for the motion feature.
/// * `Dwell` - Dwell action, for pausing or waiting, within the motion feature.
/// * `HotEndTemperature` - Action to set or monitor hot-end temperature.
/// * `HotBedTemperature` - Action to set or monitor hot-bed temperature.
///
/// # Examples
///
/// See [DeferEvent]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum DeferAction {
    /// Homing action for the motion feature.
    #[cfg(feature = "with-motion")]
    Homing,

    /// Rapid movement action for the motion feature.
    #[cfg(feature = "with-motion")]
    RapidMove,

    /// Linear movement action for the motion feature.
    #[cfg(feature = "with-motion")]
    LinearMove,

    /// Dwell action, for pausing or waiting, within the motion feature.
    #[cfg(feature = "with-motion")]
    Dwell,

    /// SetPosition action for the motion feature.
    #[cfg(feature = "with-motion")]
    SetPosition,

    /// Action to set or monitor hot-end temperature.
    #[cfg(feature = "with-hot-end")]
    HotEndTemperature,

    /// Action to set or monitor hotbed temperature.
    #[cfg(feature = "with-hot-bed")]
    HotBedTemperature,
}

//#endregion

//#region "Defer Event"

///! These are the Events that can be deferred
///
/// # Variants
///
/// * `AwaitRequested` - For requesting a Deferred action notification.
/// * `Completed` - Indication that the Deferred action has been performed.
///
/// # Examples
///
/// ```rust
/// use printhor_hwa_common as hwa;
/// use hwa::{DeferEvent, DeferAction, CommChannel};
/// // Example usage of DeferEvent
/// #[cfg(feature = "with-motion")]
/// let _event1 = DeferEvent::AwaitRequested(DeferAction::Homing, CommChannel::default());
/// #[cfg(feature = "with-motion")]
/// let _event2 = DeferEvent::Completed(DeferAction::RapidMove, CommChannel::default());
/// ```
pub enum DeferEvent {
    /// A request for defer action
    AwaitRequested(DeferAction, CommChannel, u32),

    /// The acknowledgment of a defer action
    Completed(DeferAction, CommChannel, u32),
}

//#endregion

//#region "Defer Channel"

/// The queue size of the defer channel
#[const_env::from_env("DEFER_CHANNEL_SIZE")]
pub const DEFER_CHANNEL_SIZE: usize = 4;

pub type DeferChannelChannelType<M> = Channel<M, DeferEvent, DEFER_CHANNEL_SIZE>;

/// This struct provides a reference to a `DeferChannelChannelType`.
///
/// `DeferChannel` allows for safe access to a static `DeferChannelChannelType` instance,
/// promoting the use of channels in a concurrent environment.
///
/// # Fields
///
/// * `inner` - A static reference to a `DeferChannelChannelType` instance.
///
/// # Examples
///
/// Creating a new `DeferChannel`:
///
/// ```rust
/// use printhor_hwa_common as hwa;
///
/// type MutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
///
/// let defer_channel: hwa::GenericDeferChannel<MutexType> = hwa::GenericDeferChannel::new(
///     hwa::make_static_ref!(
///         "DeferChannel",
///         hwa::DeferChannelChannelType<MutexType>,
///         hwa::DeferChannelChannelType::new()
///     )
/// );
///
/// ```
pub struct GenericDeferChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    channel: &'static DeferChannelChannelType<M>,
}

impl<M> GenericDeferChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    /// Creates a new `DeferChannelRef`.
    ///
    /// # Arguments
    ///
    /// * `channel` - A static reference to a `DeferChannelChannelType` instance.
    ///
    /// # Returns
    ///
    /// A new instance of `DeferChannelRef`.
    ///
    /// # Example
    ///
    /// See [GenericDeferChannel]
    pub const fn new(channel: &'static DeferChannelChannelType<M>) -> Self {
        GenericDeferChannel { channel }
    }
}
impl<M> core::ops::Deref for GenericDeferChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    type Target = DeferChannelChannelType<M>;

    fn deref(&self) -> &Self::Target {
        self.channel
    }
}

impl<M> Clone for GenericDeferChannel<M>
where
    M: hwa::AsyncRawMutex + 'static,
{
    fn clone(&self) -> Self {
        GenericDeferChannel::new(self.channel)
    }
}

//#endregion

#[cfg(test)]
#[cfg(feature = "with-motion")]
pub mod tests {

    use crate as hwa;
    use std::sync::RwLock;

    type MutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    static DEFER_CHANNEL: RwLock<Option<hwa::GenericDeferChannel<MutexType>>> = RwLock::new(None);

    fn initialize() {
        let mut global = DEFER_CHANNEL.write().unwrap();

        if global.is_none() {
            global.replace(hwa::GenericDeferChannel::new(hwa::make_static_ref!(
                "DeferChannel",
                hwa::DeferChannelChannelType<MutexType>,
                hwa::DeferChannelChannelType::new()
            )));
        }
    }

    #[cfg(feature = "with-motion")]
    #[futures_test::test]
    async fn test_event_bus() {
        initialize();
        let mg = DEFER_CHANNEL.read().unwrap();
        let defer_channel = mg.as_ref().unwrap();

        // Can clone
        let _cloned_defer_channel = defer_channel.clone();

        let sender = defer_channel.sender();
        let _ = sender
            .send(hwa::DeferEvent::AwaitRequested(
                hwa::DeferAction::Homing,
                hwa::CommChannel::default(),
                1,
            ))
            .await;
    }
}
