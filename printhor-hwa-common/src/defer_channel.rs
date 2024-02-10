use embassy_sync::channel::Channel;
use crate::{CommChannel, TrackedStaticCell};

type ChannelMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub enum DeferAction {
    #[cfg(feature = "with-motion")]
    Homing,
    #[cfg(feature = "with-motion")]
    RapidMove,
    #[cfg(feature = "with-motion")]
    LinearMove,
    #[cfg(feature = "with-motion")]
    Dwell,
    #[cfg(feature = "with-hot-end")]
    HotendTemperature,
    #[cfg(feature = "with-hot-bed")]
    HotbedTemperature,
}

///! These are the Events that can be deferred
#[allow(unused)]
pub enum DeferEvent {
    AwaitRequested(DeferAction, CommChannel),
    Completed(DeferAction, CommChannel),
}

type ChannelType = Channel<ChannelMutexType, DeferEvent, 4>;

#[derive(Clone)]
pub struct DeferChannelRef {
    pub(crate) inner: &'static ChannelType,
}

//#[cfg(feature = "native")]
unsafe impl Send for DeferChannelRef {}

impl core::ops::Deref for DeferChannelRef {
    type Target = ChannelType;

    fn deref(&self) -> &Self::Target {
        self.inner
    }
}

pub fn init_defer_channel() -> DeferChannelRef {
    static CHANNEL: TrackedStaticCell<ChannelType> = TrackedStaticCell::new();

    DeferChannelRef {
        inner: CHANNEL.init("defer_channel", ChannelType::new()),
    }
}