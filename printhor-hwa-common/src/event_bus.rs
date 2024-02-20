//! The state manager controller.
//!
//! This module provides a global asynchronous event bus.
use core::ops::{BitAnd, BitOr, BitXor};
use crate::{TrackedStaticCell, ControllerMutex, ControllerRef};
use bitflags::bitflags;

pub type PubSubType = embassy_sync::pubsub::PubSubChannel<crate::ControllerMutexType, EventFlags, 1, 6, 1>;
pub type PublisherType = embassy_sync::pubsub::Publisher<'static, crate::ControllerMutexType, EventFlags, 1, 6, 1>;
pub type SubscriberType<'a> = embassy_sync::pubsub::Subscriber<'a, crate::ControllerMutexType, EventFlags, 1, 6, 1>;

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct EventFlags: u32 {
        const SYS_ALARM        = 0b1000000000000000000000000000000;
        const SYS_BOOTING      = 0b0100000000000000000000000000000;
        const SYS_BOOT_FAILURE = 0b0010000000000000000000000000000;
        const SYS_READY        = 0b0001000000000000000000000000000;
        const ATX_ON           = 0b0000100000000000000000000000000;
        const HOMMING          = 0b0000010000000000000000000000000;
        const MOVING           = 0b0000001000000000000000000000000;
        const MOV_QUEUE_EMPTY  = 0b0000000000000000000001000000000;
        const JOB_FILE_SEL     = 0b0000000000000000000000100000000;
        const JOB_PRINTING     = 0b0000000000000000000000010000000;
        const JOB_PAUSED       = 0b0000000000000000000000001000000;
        const JOB_COMPLETED    = 0b0000000000000000000000000100000;
        const HOTBED_TEMP_OK   = 0b0000000000000000000000000010000;
        const HOTEND_TEMP_OK   = 0b0000000000000000000000000001000;
        const X_MIN_ON         = 0b0000000000000000000000000000100;
        const Y_MIN_ON         = 0b0000000000000000000000000000010;
        const Z_MIN_ON         = 0b0000000000000000000000000000001;
    }
}


pub struct EventBus {
    bus: &'static PubSubType,
    publisher: PublisherType,
    status: EventFlags,
}

impl EventBus {
    pub(crate) fn publish_event(&mut self, event: EventStatus) {
        let incoming_bits = event.flags.bitand(event.mask);
        let changed_bits = incoming_bits.bitxor(self.status).bitand(event.mask);
        if !changed_bits.is_empty() {
            //crate::trace!("There is a change");
            let new_status = self.status.bitand(changed_bits.complement()).bitor(incoming_bits.bitand(changed_bits));
            self.status = new_status;
            self.publisher.publish_immediate(new_status);
        }
        else {
            //crate::trace!("No change, no update");
        }
    }

    pub(crate) fn get_status(&self) -> EventFlags {
        self.status
    }

}

pub struct EventBusRef {
    instance: ControllerRef<EventBus, crate::ControllerMutexType>,
}

pub struct EventBusSubscriber<'a> {
    inner: SubscriberType<'a>,
    last_status: EventFlags,
}

impl EventBusRef {
    pub fn new(c: ControllerRef<EventBus>) -> Self {
        Self {
            instance: c,
        }
    }

    #[inline]
    pub async fn publish_event(&self, event: EventStatus) {
        let mut req = self.instance.lock().await;
        req.publish_event(event);
    }

    #[inline]
    pub async fn get_status(&self) -> EventFlags {
        let req = self.instance.lock().await;
        req.get_status()
    }

    #[inline]
    pub async fn has_flags(&self, flags: EventFlags) -> bool {
        let req = self.instance.lock().await;
        req.get_status().bitand(flags).eq(&flags)
    }

    pub async fn subscriber(&self) -> EventBusSubscriber {
        let req = self.instance.lock().await;
        let inner = req.bus.subscriber().expect("Exceeded");
        EventBusSubscriber{
            inner,
            last_status: req.get_status(),
        }
    }
}

impl EventBusSubscriber<'_> {
    #[allow(unused)]
    #[inline]
    pub async fn get_status(&mut self) -> EventFlags {
        let mut status = self.last_status;
        loop {
            match self.inner.try_next_message_pure() {
                None => {
                    self.last_status = status;
                    break;
                },
                Some(s) => {
                    status = s;
                }
            }
        }
        status
    }

    /// // Use [Self::ft_wait_for] instead
    #[deprecated]
    pub async fn wait_for(&mut self, what: EventStatus) {
        let wanted = what.flags.bitand(what.mask);
        if let Some(msg) = self.inner.try_next_message_pure() {
            //crate::trace!("last_status = {:?}", msg);
            self.last_status = msg;
        }
        loop {
            let relevant_bits = self.last_status.bitand(what.mask);
            if wanted.eq(&relevant_bits) {
                return;
            }
            self.last_status = self.inner.next_message_pure().await;
        }
    }

    /// Waits until desired event(s) occur or SYS_ALARM is triggered
    /// Returns:
    /// * Ok when event occur.
    /// * Err when SYS_ALARM is set
    /// Special cases:
    /// * If SYS_ALARM is addressed (mentioned) in the expected event, No error is returned and simply will wait for desired condition
    pub async fn ft_wait_for(&mut self, what: EventStatus) -> Result<(),()> {
        let wanted = what.flags.bitand(what.mask);
        if let Some(msg) = self.inner.try_next_message_pure() {
            //crate::trace!("last_status = {:?}", msg);
            self.last_status = msg;
        }
        loop {
            // Check SYS_ALARM condition unless SYS_ALARM is explicitly mentioned
            if !what.mask.contains(EventFlags::SYS_ALARM) && self.last_status.contains(EventFlags::SYS_ALARM) {
                return Err(())
            }
            let relevant_bits = self.last_status.bitand(what.mask);
            if wanted.eq(&relevant_bits) {
                return Ok(());
            }
            self.last_status = self.inner.next_message_pure().await;
        }
    }

    #[inline]
    pub async fn ft_wait_until(&mut self, flags: EventFlags)  -> Result<(),()> {
        self.ft_wait_for(EventStatus::containing(flags)).await
    }

    // Use [Self::ft_wait_until] instead
    #[inline]
    #[deprecated]
    #[allow(deprecated)]
    pub async fn wait_until(&mut self, flags: EventFlags) {
        self.wait_for(EventStatus::containing(flags)).await
    }

    #[inline]
    pub async fn ft_wait_while(&mut self, flags: EventFlags) -> Result<(),()> {
        self.ft_wait_for(EventStatus::not_containing(flags)).await
    }

    // Use [Self::ft_wait_while] instead
    #[inline]
    #[deprecated]
    #[allow(deprecated)]
    pub async fn wait_while(&mut self, flags: EventFlags) {
        self.wait_for(EventStatus::not_containing(flags)).await
    }
}

impl Clone for EventBusRef {
    fn clone(&self) -> Self {

        let pusher = self.instance.clone();

        EventBusRef {
            instance: pusher
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EventStatus {
    pub flags: EventFlags,
    pub mask: EventFlags,
}

impl EventStatus {
    #[allow(unused)]
    pub const fn new() -> Self {
        Self {
            flags: EventFlags::empty(),
            mask: EventFlags::empty(),
        }
    }
    #[inline]
    pub const fn containing(flags: EventFlags) -> Self {
        Self {
            flags,
            mask: flags,
        }
    }
    #[inline]
    pub const fn not_containing(flags: EventFlags) -> Self {
        Self {
            flags: flags.complement(),
            mask: flags,
        }
    }
    #[allow(unused)]
    #[inline]
    pub fn and_containing(&self, flags: EventFlags) -> Self {
        Self {
            flags: self.flags.bitor(flags),
            mask: self.mask.bitor(flags),
        }
    }
    #[allow(unused)]
    #[inline]
    pub fn and_not_containing(&self, flags: EventFlags) -> Self {
        Self {
            flags: self.flags.bitand(flags.complement()),
            mask: self.mask.bitor(flags),
        }
    }
}

pub fn init_event_bus<const MAX_SIZE: usize>() -> EventBusRef {
    static EVT_BUS: TrackedStaticCell<PubSubType> = TrackedStaticCell::new();
    static EVT_CTRL_BUS: TrackedStaticCell<ControllerMutex<EventBus>> = TrackedStaticCell::new();

    let bus = EVT_BUS.init::<MAX_SIZE>("EventBusChannel", PubSubType::new());
    let publisher: PublisherType = bus.publisher().expect("publisher exausted");

    EventBusRef::new(
        ControllerRef::new(
            EVT_CTRL_BUS.init::<MAX_SIZE>("EventBus", ControllerMutex::new(
                EventBus {
                    bus,
                    publisher,
                    status: EventFlags::empty(),
                }
            ))
        )
    )
}
