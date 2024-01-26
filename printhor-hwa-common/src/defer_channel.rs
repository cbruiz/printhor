use embassy_sync::channel::Channel;
use crate::TrackedStaticCell;

type ChannelMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub enum DeferType {
    AwaitRequested,
    Completed,
}

///! These are the Events that can be deferred
#[allow(unused)]
pub enum DeferEvent {
    #[cfg(feature = "with-motion")]
    Homing(DeferType),
    #[cfg(feature = "with-motion")]
    RapidMove(DeferType),
    #[cfg(feature = "with-motion")]
    LinearMove(DeferType),
    #[cfg(feature = "with-motion")]
    Dwell(DeferType),
    #[cfg(feature = "with-hotend")]
    HotendTemperature(DeferType),
    #[cfg(feature = "with-hotbed")]
    HotbedTemperature(DeferType),
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