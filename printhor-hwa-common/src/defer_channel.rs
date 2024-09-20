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
//! use printhor_hwa_common::{DeferEvent, DeferAction, CommChannel};
//!
//! // Example usage of DeferEvent
//! let event1 = DeferEvent::AwaitRequested(DeferAction::Homing, CommChannel::default());
//! let event2 = DeferEvent::Completed(DeferAction::RapidMove, CommChannel::default());
//!
//! use printhor_hwa_common as hwa;
//! const MAX_STATIC_MEMORY: usize = 1024;
//!
//! let defer_channel: hwa::DeferChannelRef = {
//!     static MDC: hwa::TrackedStaticCell<hwa::DeferChannelChannelType> = hwa::TrackedStaticCell::new();
//!     hwa::DeferChannelRef::new( 
//!         MDC.init::<{ MAX_STATIC_MEMORY }>("defer_channel", hwa::DeferChannelChannelType::new()) 
//!     ) 
//! };
//! ```
//!
//! ## Purpose
//!
//! This module is designed for applications that require efficient task management and concurrency control. 
//! It is aimed at developers who need to defer actions and handle these deferred actions reliably. 
//! By using static references and safe concurrency practices, this module ensures that deferred actions are managed 
//! in a predictable and efficient manner.
use embassy_sync::channel::Channel;
use crate::{CommChannel, TrackedStaticCell};

///
/// `DeferEvent` represents the possible events that can be deferred in the system.
///
/// # Variants
///
/// * `AwaitRequested(DeferAction, CommChannel)` - Indicates that a defer action has been requested and is awaiting execution.
/// * `Completed(DeferAction, CommChannel)` - Indicates that a defer action has been completed.
///
/// # Examples
///
/// ```rust
/// use printhor_hwa_common::{DeferEvent, DeferAction, CommChannel};
///
/// // Example usage of DeferEvent
/// let event1 = DeferEvent::AwaitRequested(DeferAction::Homing, CommChannel::default());
/// let event2 = DeferEvent::Completed(DeferAction::RapidMove, CommChannel::default());
/// ```
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum DeferAction {
    #[cfg(feature = "with-motion")]
    /// Homing action for the motion feature.
    Homing,
    #[cfg(feature = "with-motion")]
    /// Rapid movement action for the motion feature.
    RapidMove,
    #[cfg(feature = "with-motion")]
    /// Linear movement action for the motion feature.
    LinearMove,
    #[cfg(feature = "with-motion")]
    /// Dwell action, possibly for pausing or waiting, within the motion feature.
    Dwell,
    #[cfg(feature = "with-hot-end")]
    /// Action to set or monitor hot-end temperature.
    HotEndTemperature,
    #[cfg(feature = "with-hot-bed")]
    /// Action to set or monitor hotbed temperature.
    HotbedTemperature,
}

///! These are the Events that can be deferred
#[allow(unused)]
pub enum DeferEvent {
    AwaitRequested(DeferAction, CommChannel),
    Completed(DeferAction, CommChannel),
}

pub type DeferChannelChannelType = Channel<crate::ControllerMutexType, DeferEvent, 40>;

/// This struct provides a reference to a `DeferChannelChannelType`.
///
/// `DeferChannelRef` allows for safe access to a static `DeferChannelChannelType` instance,
/// promoting the use of channels in a concurrent environment.
///
/// # Fields
///
/// * `inner` - A static reference to a `DeferChannelChannelType` instance.
///
/// # Examples
///
/// Creating a new `DeferChannelRef`:
///
/// ```
/// use printhor_hwa_common as hwa;
/// const MAX_STATIC_MEMORY: usize = 1024;
///
/// let defer_channel: hwa::DeferChannelRef = {
///     static MDC: hwa::TrackedStaticCell<hwa::DeferChannelChannelType> = hwa::TrackedStaticCell::new();
///     hwa::DeferChannelRef::new( 
///         MDC.init::<{ MAX_STATIC_MEMORY }>("defer_channel", hwa::DeferChannelChannelType::new()) 
///     ) 
/// };
/// ```
#[derive(Clone)]
pub struct DeferChannelRef {
    inner: &'static DeferChannelChannelType,
}

impl DeferChannelRef {
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
    /// See [DeferChannelRef]
    pub const fn new(channel: &'static DeferChannelChannelType) -> Self {
        DeferChannelRef { inner: channel }
    }
}

//#[cfg(feature = "native")]
//unsafe impl Send for DeferChannelRef {}

impl core::ops::Deref for DeferChannelRef {
    type Target = DeferChannelChannelType;

    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

pub fn init_defer_channel<const MAX_SIZE: usize>() -> DeferChannelRef {
    static CHANNEL: TrackedStaticCell<DeferChannelChannelType> = TrackedStaticCell::new();

    DeferChannelRef {
        inner: CHANNEL.init::<MAX_SIZE>("defer_channel", DeferChannelChannelType::new()),
    }
}
