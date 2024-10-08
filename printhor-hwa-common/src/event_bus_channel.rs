//#region "Event Bus Channel"

use crate::event_bus::{
    EventBusPubSubType, EventBusPublisherType, EventBusSubscriberType, EventFlags, EventStatus,
};
use core::ops::{BitAnd, BitOr, BitXor};

/// Represents the *internal* channel controller of the event bus.
/// It's responsible for managing and
/// publishing events within the system. The [crate::GenericEventBus]
/// structure holds a reference to this one, a publisher
/// for sending events, and the current status flags.
///
/// This structure ensures thread-safe operations through
/// asynchronous event handling and allows the system to
/// broadcast state changes and other relevant events globally.
///
/// # Traits
/// * `H` - The MutexStrategy of the internal resource (the pub-sub channel)
/// * `M` - The MutexType of the pub-sub channel.
///
/// # Fields
///
/// * `channel` - A static reference to the Pub/Sub event bus channel.
/// * `publisher` - A publisher that can send events immediately to the bus.
/// * `status` - Flags representing the current status of the system,
///              managed and updated by the event bus.
///
/// # Examples
///
/// See [crate::GenericEventBus]
pub struct EventBusChannelController<M>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    channel: &'static EventBusPubSubType<M>,
    publisher: EventBusPublisherType<M>,
    status: EventFlags,
}

impl<M> EventBusChannelController<M>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    pub fn new(channel: &'static EventBusPubSubType<M>) -> Self {
        let publisher = channel.publisher().unwrap();
        EventBusChannelController {
            channel,
            publisher,
            status: EventFlags::empty(),
        }
    }

    /// Publishes an event to the event bus, updating the status flags if there are changes.
    ///
    /// The method takes an `EventStatus` struct which contains the event flags and a mask.
    /// It computes the incoming bits, identifies changes against the current status, and
    /// updates the status flags accordingly. If there are changes, it publishes the new status
    /// immediately using the publisher.
    ///
    /// # Arguments
    ///
    /// * `event` - An `EventStatus` instance containing the event flags and mask.
    ///
    /// # Examples
    ///
    /// Basic usage:
    ///
    /// TBD
    ///
    pub fn publish_event(&mut self, event: EventStatus) {
        let incoming_bits = event.flags.bitand(event.mask);
        let changed_bits = incoming_bits.bitxor(self.status).bitand(event.mask);
        if !changed_bits.is_empty() {
            //crate::trace!("There is a change");
            let new_status = self
                .status
                .bitand(changed_bits.complement())
                .bitor(incoming_bits.bitand(changed_bits));
            self.status = new_status;
            self.publisher.publish_immediate(new_status);
        } else {
            //crate::trace!("No change, no update");
        }
    }

    pub fn get_status(&self) -> EventFlags {
        self.status
    }

    pub fn subscriber(
        &mut self,
    ) -> Result<EventBusSubscriberType<'static, M>, embassy_sync::pubsub::Error> {
        self.channel.subscriber()
    }
}

//#endregion

//#region "Event Bus Subscriber"

pub struct EventBusSubscriber<'a, M>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    inner: EventBusSubscriberType<'a, M>,
    last_status: EventFlags,
}

impl<M> EventBusSubscriber<'static, M>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    pub(crate) const fn new(
        inner: EventBusSubscriberType<'static, M>,
        last_status: EventFlags,
    ) -> Self {
        Self { inner, last_status }
    }

    pub async fn get_status(&mut self) -> EventFlags {
        let mut status = self.last_status;
        loop {
            match self.inner.try_next_message_pure() {
                None => {
                    self.last_status = status;
                    break;
                }
                Some(s) => {
                    status = s;
                }
            }
        }
        status
    }

    /// Waits until desired event(s) occur or SYS_ALARM is triggered. ft stands for fault tolerant.
    ///
    /// # Returns
    ///
    /// * `Ok(())` when the desired event(s) occur.
    /// * `Err(())` when SYS_ALARM is set, unless SYS_ALARM is explicitly mentioned in the expected event.
    ///
    /// # Arguments
    ///
    /// * `what` - An `EventStatus` instance containing the event flags and mask indicating the desired event(s).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::{GenericEventBus, EventFlags, EventStatus};
    ///
    /// type ChannelControllerMutexType = hwa::SyncSendMutex;
    /// type PubSubMutexType = hwa::SyncSendMutex;
    /// type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
    /// type EventBusMutexStrategyType = hwa::Holdable<ChannelControllerMutexType, ResourceType>;
    ///
    /// // See [EventBus]
    /// // let event_bus: EventBusController<BusMutexType, ChannelMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     let mut subscriber = event_bus.subscriber().await;
    ///     // [...]
    ///     match subscriber.ft_wait_for(EventStatus::containing(EventFlags::SYS_READY)).await {
    ///         Ok(_) => {
    ///             // Desired event occurred, handle success
    ///         },
    ///         Err(_) => {
    ///             // SYS_ALARM was set, handle error
    ///         },
    ///     }
    /// }
    /// ```
    pub async fn ft_wait_for(&mut self, what: EventStatus) -> Result<(), ()> {
        if what.flags.is_empty() {
            Ok(())
        } else {
            #[cfg(any(feature = "with-log", feature = "with-defmt"))]
            let t0 = embassy_time::Instant::now();
            let wanted = what.flags.bitand(what.mask);

            #[cfg(any(feature = "with-log", feature = "with-defmt"))]
            crate::debug!("ft_wait_for [{:?}]...", what);
            if let Some(msg) = self.inner.try_next_message_pure() {
                //crate::trace!("last_status = {:?}", msg);
                self.last_status = msg;
            }
            loop {
                // Check SYS_ALARM condition unless SYS_ALARM is explicitly mentioned
                if !what.mask.contains(EventFlags::SYS_ALARM)
                    && self.last_status.contains(EventFlags::SYS_ALARM)
                {
                    #[cfg(any(feature = "with-log", feature = "with-defmt"))]
                    crate::debug!(
                        "ft_wait_for [{:?}] -> ERR (took: {} us)",
                        what,
                        t0.elapsed().as_micros()
                    );
                    return Err(());
                }
                let relevant_bits = self.last_status.bitand(what.mask);
                if wanted.eq(&relevant_bits) {
                    #[cfg(any(feature = "with-log", feature = "with-defmt"))]
                    crate::debug!(
                        "ft_wait_for [{:?}] -> DONE (took: {} us)",
                        what,
                        t0.elapsed().as_micros()
                    );
                    return Ok(());
                }
                self.last_status = self.inner.next_message_pure().await;
            }
        }
    }

    /// Waits until the desired flags are set or SYS_ALARM is triggered. ft stands for fault tolerant.
    ///
    /// # Returns
    ///
    /// * `Ok(())` when the desired flags are set.
    /// * `Err(())` when SYS_ALARM is set, unless SYS_ALARM is explicitly mentioned in the expected flags.
    ///
    /// # Arguments
    ///
    /// * `flags` - An `EventFlags` instance indicating the desired flags.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::{GenericEventBus, EventFlags, EventStatus};
    ///
    /// type ChannelControllerMutexType = hwa::SyncSendMutex;
    /// type PubSubMutexType = hwa::SyncSendMutex;
    /// type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
    /// type EventBusMutexStrategyType = hwa::Holdable<ChannelControllerMutexType, ResourceType>;
    ///
    /// // See [EventBus]
    /// // let event_bus: EventBus<EventBusMutexStrategyType, PubSubMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     let mut subscriber = event_bus.subscriber().await;
    ///     // [...]
    ///     match subscriber.ft_wait_until(EventFlags::SYS_BOOTING).await {
    ///         Ok(_) => {
    ///             // Desired event occurred, handle success
    ///         },
    ///         Err(_) => {
    ///             // SYS_ALARM was set, handle error
    ///         },
    ///     }
    /// }
    /// ```
    pub async fn ft_wait_until(&mut self, flags: EventFlags) -> Result<(), ()> {
        self.ft_wait_for(EventStatus::containing(flags)).await
    }

    /// Waits until the specified flags are reset.
    ///
    /// # Returns
    ///
    /// * `Ok(())` if the specified flags are reset.
    /// * `Err(())` if SYS_ALARM is set, unless SYS_ALARM is explicitly mentioned in the expected flags.
    ///
    /// # Arguments
    ///
    /// * `flags` - An `EventFlags` instance indicating the flags to wait until they are reset.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::{GenericEventBus, EventFlags, EventStatus};
    ///
    /// type ChannelControllerMutexType = hwa::SyncSendMutex;
    /// type PubSubMutexType = hwa::SyncSendMutex;
    /// type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
    /// type EventBusMutexStrategyType = hwa::Holdable<ChannelControllerMutexType, ResourceType>;
    ///
    /// // See [EventBus]
    /// // let event_bus: EventBus<EventBusMutexStrategyType, PubSubMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     let mut subscriber = event_bus.subscriber().await;
    ///     // [...]
    ///     match subscriber.ft_wait_until_reset(EventFlags::SYS_BOOTING).await {
    ///         Ok(_) => {
    ///             // Specified flag(s) is/are reset, handle success
    ///         },
    ///         Err(_) => {
    ///             // SYS_ALARM was set, handle error
    ///         },
    ///     }
    /// }
    /// ```
    pub async fn ft_wait_until_reset(&mut self, flags: EventFlags) -> Result<(), ()> {
        self.ft_wait_for(EventStatus::not_containing(flags)).await
    }

    /// Waits while the specified flags are set.
    ///
    /// # Returns
    ///
    /// * `Ok(())` if the specified flags are set.
    /// * `Err(())` if SYS_ALARM is set, unless SYS_ALARM is explicitly mentioned in the expected flags.
    ///
    /// # Arguments
    ///
    /// * `flags` - An `EventFlags` instance indicating the flags to wait while they are set.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::{GenericEventBus, EventFlags, EventStatus};
    ///
    /// type ChannelControllerMutexType = hwa::NoopMutex;
    /// type PubSubMutexType = hwa::NoopMutex;
    /// type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
    /// type EventBusMutexStrategyType = hwa::NotHoldable<ChannelControllerMutexType, ResourceType>;
    ///
    /// // See [EventBus]
    /// // let event_bus: EventBus<EventBusMutexStrategyType, PubSubMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     let mut subscriber = event_bus.subscriber().await;
    ///     // [...]
    ///     match subscriber.ft_wait_while(EventFlags::SYS_BOOTING).await {
    ///         Ok(_) => {
    ///             // Specified flag(s) is/are reset, handle success
    ///         },
    ///         Err(_) => {
    ///             // SYS_ALARM was set, handle error
    ///         },
    ///     }
    /// }
    /// ```
    pub async fn ft_wait_while(&mut self, flags: EventFlags) -> Result<(), ()> {
        self.ft_wait_for(EventStatus::not_containing(flags)).await
    }
}

//#endregion
