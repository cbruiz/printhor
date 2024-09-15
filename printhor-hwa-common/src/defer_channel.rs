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
