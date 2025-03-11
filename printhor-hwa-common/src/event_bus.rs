//! The state manager controller.
//!
//! This module provides a global asynchronous event bus.
use crate as hwa;
use core::fmt::Formatter;
use core::ops::{BitAnd, BitOr};
use hwa::event_bus_channel::{EventBusChannelController, EventBusSubscriber};
use printhor_hwa_utils::*;

#[const_env::from_env("EVENT_BUS_NUM_SUBSCRIBERS")]
const EVENT_BUS_NUM_SUBSCRIBERS: usize = 6;

pub type EventBusPubSubType<M> =
    embassy_sync::pubsub::PubSubChannel<M, EventFlags, 1, EVENT_BUS_NUM_SUBSCRIBERS, 1>;
pub type EventBusPublisherType<M> =
    embassy_sync::pubsub::Publisher<'static, M, EventFlags, 1, EVENT_BUS_NUM_SUBSCRIBERS, 1>;
pub type EventBusSubscriberType<'a, M> =
    embassy_sync::pubsub::Subscriber<'a, M, EventFlags, 1, EVENT_BUS_NUM_SUBSCRIBERS, 1>;

//#region "Event Flags"

bitflags::bitflags! {
    /// Structure representing a collection of event flags.
    ///
    /// `EventFlags` is a bitflags structure used to define different states and conditions
    /// within the system. Each event flag represents a specific state or condition, and
    /// multiple flags can be combined using bitwise operations to track composite states.
    ///
    /// # Variants
    ///
    /// - `NOTHING`: Special flag meaning no expectations.
    /// - `SYS_ALARM`: System alarm flag.
    /// - `SYS_BOOTING`: System is booting.
    /// - `SYS_BOOT_FAILURE`: System boot failure.
    /// - `SYS_READY`: System is ready.
    /// - `ATX_ON`: ATX power is on.
    /// - `HOMING`: Homing operation in progress.
    /// - `MOVING`: System is moving.
    /// - `MOV_QUEUE_EMPTY`: Movement queue is empty.
    /// - `MOV_QUEUE_FULL`: Movement queue is full.
    /// - `JOB_PRINTING`: Job printing in progress.
    /// - `JOB_PAUSED`: Job is paused.
    /// - `JOB_COMPLETED`: Job completed.
    /// - `DRY_RUN`: Do nothing. Just interpret.
    /// - `HOT_BED_TEMP_OK`: HotBed temperature is okay.
    /// - `HOT_END_TEMP_OK`: HotEnd temperature is okay.
    ///
    /// # Examples
    ///
    /// Basic usage:
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::EventFlags;
    ///
    /// let mut flags = EventFlags::empty();
    ///
    /// // Set the SYS_READY and ATX_ON flags
    /// flags.insert(EventFlags::SYS_READY | EventFlags::ATX_ON);
    ///
    /// // Check if a specific flag is set
    /// if flags.contains(EventFlags::SYS_READY) {
    ///     // Do something if the system is ready
    /// }
    ///
    /// // Remove a flag
    /// flags.remove(EventFlags::ATX_ON);
    /// ```
    #[derive(Clone, Copy, PartialEq)]
    pub struct EventFlags: u16 {
        /// Special flag meaning no expectations
        const NOTHING          =  0b0000000000000000;
        /// System alarm flag
        const SYS_ALARM        =  0b1000000000000000;
        /// System is booting
        const SYS_BOOTING      =  0b0100000000000000;
        /// System boot failure
        const SYS_BOOT_FAILURE =  0b0010000000000000;
        /// System is ready
        const SYS_READY        =  0b0001000000000000;
        /// ATX power is on
        const ATX_ON           =  0b0000100000000000;
        /// Homing operation in progress
        const HOMING          =   0b0000010000000000;
        /// System is moving
        const MOVING           =  0b0000001000000000;
        /// Movement queue is empty
        const MOV_QUEUE_EMPTY  =  0b0000000100000000;
        /// Movement queue is full
        const MOV_QUEUE_FULL   =  0b0000000010000000;
        /// Job printing in progress
        const JOB_PRINTING     =  0b0000000001000000;
        /// Job is paused
        const JOB_PAUSED       =  0b0000000000100000;
        /// Job completed
        const JOB_COMPLETED    =  0b0000000000010000;
        /// Dry Run (do nothing. Just interpret)
        const DRY_RUN          =  0b0000000000001000;
        /// HotBed temperature is okay
        const HOT_BED_TEMP_OK   = 0b0000000000000100;
        /// HotEnd temperature is okay
        const HOT_END_TEMP_OK   = 0b0000000000000010;
    }
}

#[cfg(feature = "with-defmt")]
impl defmt::Format for EventFlags {
    fn format(&self, fmt: defmt::Formatter) {
        let mut first = true;
        for (flag_name, _) in self.iter_names() {
            if !first {
                defmt::write!(fmt, " ");
            } else {
                first = false;
            }
            defmt::write!(fmt, "{}", flag_name)
        }
    }
}

impl core::fmt::Debug for EventFlags {
    fn fmt(&self, fmt: &mut Formatter<'_>) -> core::fmt::Result {
        let mut first = true;
        for (flag_name, _flag_bits) in self.iter_names() {
            if !first {
                core::write!(fmt, " ")?;
            } else {
                first = false;
            }
            core::write!(fmt, "{}", flag_name)?;
        }
        Ok(())
    }
}
//#endregion

//#region "Event Status"

/// Represents the status of an event with associated flags and mask.
///
/// The `EventStatus` structure holds two primary fields:
/// - `flags`: Contains the status of relevant bit values (set or unset) for the bits set in mask.
/// - `mask`: Defines which bits in the `flags` field are relevant for event matching.
#[derive(Clone, Copy)]
pub struct EventStatus {
    /// Contains the status of relevant bit values (set or unset) for the bits set in mask.
    pub flags: EventFlags,
    /// Defines which bits in the `flags` field are relevant for event matching.
    pub mask: EventFlags,
}

impl EventStatus {
    /// Updates the `flags` and `mask` of the current `EventStatus` by complementing and
    /// bitwise ANDing with the provided `flags` for `flags`, and bitwise ORing with the
    /// provided `flags` for `mask`.
    ///
    /// This method applies a bitwise AND between the current `flags` and the complement of
    /// the provided `flags`, and applies a bitwise OR between the current `mask` and the
    /// provided `flags`. This allows selective modification of the current event state and
    /// its relevance mask.
    ///
    /// # Parameters
    ///
    /// - `flags`: The `EventFlags` value to apply for the updates.
    ///
    /// # Returns
    ///
    /// A new `EventStatus` with updated `flags` and `mask` based on the provided `flags`.
    ///
    /// # Examples
    ///
    /// ```
    /// use printhor_hwa_common as hwa;
    /// use hwa::EventFlags;
    /// use hwa::EventStatus;
    ///
    /// // Initial EventStatus with no flags set
    /// let mut event_status = EventStatus::new();
    ///
    /// // EventStatus updated to exclude certain flags and add to the mask
    /// let updated_event_status = event_status.and_not_containing(EventFlags::SYS_BOOTING);
    /// ```
    #[allow(unused)]
    #[inline]
    pub const fn new() -> Self {
        Self {
            flags: EventFlags::empty(),
            mask: EventFlags::empty(),
        }
    }

    /// Returns an `EventStatus` instance with all bits set in the `flags` and `mask` fields.
    ///
    /// This method initializes the `flags` field with the value provided by
    /// the `flags` parameter and sets the `mask` field with the same value.
    /// It essentially creates an `EventStatus` where all bits in `flags` are
    /// considered relevant for event checks.
    ///
    /// # Parameters
    ///
    /// - `flags`: An `EventFlags` value that represents the current state or
    /// condition of the event, used to initialize both the `flags` and `mask` fields.
    ///
    /// # Returns
    ///
    /// An `EventStatus` instance where `flags` is set to the provided `EventFlags` value
    /// and `mask` is set to the same value, indicating that all bits in `flags` are relevant.
    ///
    /// # Examples
    ///
    /// ```
    /// use printhor_hwa_common as hwa;
    /// use hwa::EventFlags;
    /// use hwa::EventStatus;
    ///
    /// // An event containging SYS_BOOTING flag
    /// let event_status = EventStatus::containing(EventFlags::SYS_BOOTING);
    /// ```
    #[inline]
    pub const fn containing(flags: EventFlags) -> Self {
        Self { flags, mask: flags }
    }

    /// Returns an `EventStatus` instance with the complement of the provided `flags` set in the `flags` field
    /// and the original `flags` set in the `mask` field.
    ///
    /// This method initializes the `flags` field with the complement of the value provided by
    /// the `flags` parameter and sets the `mask` field with the original value.
    /// It creates an `EventStatus` where all bits not in the provided `flags` are considered relevant
    /// for event checks.
    ///
    /// # Parameters
    ///
    /// - `flags`: An `EventFlags` value that represents the current state or
    /// condition of the event, used to initialize the `flags` and `mask` fields.
    ///
    /// # Returns
    ///
    /// An `EventStatus` instance where `flags` is set to the complement of the provided `EventFlags` value
    /// and `mask` is set to the original value, indicating that all bits not in `flags` are relevant.
    ///
    /// # Examples
    ///
    /// ```
    /// use printhor_hwa_common as hwa;
    /// use hwa::EventFlags;
    /// use hwa::EventStatus;
    ///
    /// // An event not containing the SYS_BOOTING flag
    /// let event_status = EventStatus::not_containing(EventFlags::SYS_BOOTING);
    /// ```
    #[inline]
    pub const fn not_containing(flags: EventFlags) -> Self {
        Self {
            flags: flags.complement(),
            mask: flags,
        }
    }

    /// Returns a new `EventStatus` instance with the complement of the provided `flags` bitwise AND
    /// with the current `flags` field, and the provided `flags` bitwise OR with the current `mask` field.
    ///
    /// This method computes a new `flags` field by taking the bitwise AND of the current `flags` and the
    /// complement of the provided `flags`. The `mask` field is updated by taking the bitwise OR of the
    /// current `mask` and the provided `flags`.
    ///
    /// # Parameters
    ///
    /// - `flags`: An `EventFlags` value that will be used to update the `flags` and `mask` fields.
    ///
    /// # Returns
    ///
    /// An `EventStatus` instance where the `flags` field is the result of the current `flags` AND the complement
    /// of the provided `EventFlags`, and the `mask` field is the result of the current `mask` OR the provided
    /// `EventFlags`.
    ///
    /// # Examples
    ///
    /// ```
    /// use printhor_hwa_common as hwa;
    /// use hwa::EventFlags;
    /// use hwa::EventStatus;
    ///
    /// let current_status = EventStatus::containing(EventFlags::SYS_BOOTING)
    ///     .and_not_containing(EventFlags::SYS_READY);
    /// ```
    #[allow(unused)]
    #[inline]
    pub fn and_containing(&self, flags: EventFlags) -> Self {
        Self {
            flags: self.flags.bitor(flags),
            mask: self.mask.bitor(flags),
        }
    }

    /// Returns a new `EventStatus` instance with the provided `flags` removed from the current `flags` field,
    /// and the provided `flags` bitwise OR with the current `mask` field.
    ///
    /// This method computes a new `flags` field by taking the bitwise AND of the current `flags` and the
    /// complement of the provided `flags`. The `mask` field is updated by taking the bitwise OR of the
    /// current `mask` and the provided `flags`.
    ///
    /// # Parameters
    ///
    /// - `flags`: An `EventFlags` value that represents the current state or condition of the event,
    /// which will be used to update the `flags` and `mask` fields.
    ///
    /// # Returns
    ///
    /// An `EventStatus` instance where the `flags` field is the result of the current `flags` AND the
    /// complement of the provided `EventFlags`, and the `mask` field is the result of the current `mask` OR
    /// the provided `EventFlags`.
    ///
    /// # Examples
    ///
    /// ```
    /// use printhor_hwa_common as hwa;
    /// use hwa::EventFlags;
    /// use hwa::EventStatus;
    ///
    /// let current_status = EventStatus::containing(EventFlags::SYS_BOOTING)
    ///     .and_not_containing(EventFlags::SYS_READY);
    /// ```
    #[allow(unused)]
    #[inline]
    pub fn and_not_containing(&self, flags: EventFlags) -> Self {
        Self {
            flags: self.flags.bitand(flags.complement()),
            mask: self.mask.bitor(flags),
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-defmt")] {
        impl defmt::Format for EventStatus {
            fn format(&self, fmt: defmt::Formatter) {
                let mut first = true;
                for (flag_name, flag_bits) in self.mask.iter_names() {
                    if !first {
                        defmt::write!(fmt, " & ");
                    }
                    else {
                        first = false;
                    }
                    if self.flags.contains(flag_bits) {
                        defmt::write!(fmt, "{}", flag_name);
                    }
                    else {
                        defmt::write!(fmt, "!{}", flag_name);
                    }
                }
            }
        }
    }
}
impl core::fmt::Debug for EventStatus {
    fn fmt(&self, fmt: &mut Formatter<'_>) -> core::fmt::Result {
        let mut first = true;
        for (flag_name, flag_bits) in self.mask.iter_names() {
            if !first {
                core::write!(fmt, " & ")?;
            } else {
                first = false;
            }
            if self.flags.contains(flag_bits) {
                core::write!(fmt, "{}", flag_name)?;
            } else {
                core::write!(fmt, "!{}", flag_name)?;
            }
        }
        Ok(())
    }
}

//#endregion

//#region "Event Bus"

/// A struct representing a reference to the `Event Bus`.
///
/// The [GenericEventBus] provides various methods to interact with the internal pub-sub channel,
/// including publishing events, getting the current status, checking specific flags,
/// and subscribing to event notifications.
///
/// # Generics
///
/// * `H` is the MutexStrategy (an instance of the trait [AsyncMutexStrategy]) to choose the locking strategy.
/// * `M` is the MutexType (an instance of the trait [hwa::AsyncRawMutex]) of the inner pub-sub mechanism.
///
/// # Examples
///
/// ```rust
/// use printhor_hwa_common as hwa;
///
/// type ChannelControllerMutexType = hwa::AsyncCsMutexType;
/// type PubSubMutexType = hwa::AsyncCsMutexType;
/// type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
/// type MutexStrategyType = hwa::AsyncStandardStrategy<ChannelControllerMutexType, ResourceType>;
///
/// let _controller = hwa::GenericEventBus::new(
///     hwa::make_static_async_controller!(
///         "EventBus",
///         MutexStrategyType,
///         hwa::EventBusChannelController::new(
///             hwa::make_static_ref!(
///                 "EventBusChannel",
///                 hwa::EventBusPubSubType<PubSubMutexType>,
///                 hwa::EventBusPubSubType::new(),
///             )
///         ),
///     )
/// );
/// ```
///
/// # Fields
///
/// * `instance` - A reference to the controller wrapping the `GenericEventBus`.
pub struct GenericEventBus<H, M>
where
    H: AsyncMutexStrategy<Resource = EventBusChannelController<M>> + 'static,
    M: hwa::AsyncRawMutex + 'static,
{
    channel_controller: StaticAsyncController<H>,
}

impl<H, M> Clone for GenericEventBus<H, M>
where
    H: AsyncMutexStrategy<Resource = EventBusChannelController<M>> + 'static,
    M: hwa::AsyncRawMutex + 'static,
{
    fn clone(&self) -> Self {
        GenericEventBus::new(self.channel_controller.clone())
    }
}

impl<H, M> GenericEventBus<H, M>
where
    H: AsyncMutexStrategy<Resource = EventBusChannelController<M>> + 'static,
    M: hwa::AsyncRawMutex + 'static,
{
    pub const fn new(channel_controller: StaticAsyncController<H>) -> Self {
        Self { channel_controller }
    }

    /// Publishes an event to the event bus asynchronously, updating the status flags if there are changes.
    ///
    /// The method takes an `EventStatus` struct which contains the event flags and a mask. It computes
    /// the incoming bits, identifies changes against the current status, and updates the status flags
    /// accordingly. If there are changes, it publishes the new status immediately using the publisher.
    ///
    /// # Arguments
    ///
    /// * `event` - An `EventStatus` instance containing the event flags and mask.
    ///
    /// # Examples
    ///
    /// Basic usage:
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::{GenericEventBus, EventFlags, EventStatus};
    ///
    /// // A Holdable mutex requires a Sync MutexType
    /// type ChannelControllerMutexType = hwa::AsyncCsMutexType;
    /// type PubSubMutexType = hwa::AsyncCsMutexType;
    /// type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
    /// type EventBusMutexStrategyType = hwa::AsyncHoldableStrategy<ChannelControllerMutexType, ResourceType>;
    ///
    /// async fn assumed_initialization(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     event_bus.publish_event(EventStatus::not_containing(EventFlags::SYS_READY)).await
    /// }
    /// // [...]
    /// async fn event_routine(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     // Update the event bus notifying that System is NOW ready
    ///     event_bus.publish_event(EventStatus::containing(EventFlags::SYS_READY)).await;
    /// }
    /// ```
    ///
    pub async fn publish_event(&self, event: EventStatus) {
        let mut req = self.channel_controller.lock().await;
        req.publish_event(event);
    }

    pub async fn get_status(&self) -> EventFlags {
        let req = self.channel_controller.lock().await;
        req.get_status()
    }

    /// Checks if the specified flags are set in the current status.
    ///
    /// This method locks the event bus instance, retrieves the current status,
    /// and checks if all bits specified in the `flags` argument are set.
    ///
    /// # Arguments
    ///
    /// * `flags` - An `EventFlags` instance containing the flags to check.
    ///
    /// # Returns
    ///
    /// `true` if all specified flags are set; `false` otherwise.
    ///
    /// # Examples
    ///
    /// Basic usage:
    ///
    /// ```rust
    /// use printhor_hwa_common as hwa;
    /// use hwa::{GenericEventBus, EventFlags, EventStatus};    ///
    ///
    /// type ChannelControllerMutexType = hwa::AsyncNoopMutexType;
    /// type PubSubMutexType = hwa::AsyncNoopMutexType;
    /// type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
    /// type EventBusMutexStrategyType = hwa::AsyncStandardStrategy<ChannelControllerMutexType, ResourceType>;
    ///
    ///
    /// async fn assumed_initialization(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     event_bus.publish_event(EventStatus::not_containing(EventFlags::SYS_READY)).await
    /// }
    ///
    /// async fn checking_routine(event_bus: GenericEventBus<EventBusMutexStrategyType, PubSubMutexType>) {
    ///     if event_bus.has_flags(EventFlags::SYS_READY).await {
    ///         // System is ready
    ///         // ...
    ///     }
    ///     else {
    ///         // System is not *CURRENTLY* ready
    ///         // ...
    ///     }
    /// }
    ///
    /// ```
    pub async fn has_flags(&self, flags: EventFlags) -> bool {
        let req = self.channel_controller.lock().await;
        req.get_status().bitand(flags).eq(&flags)
    }

    pub async fn subscriber(&self) -> EventBusSubscriber<'static, M> {
        let mut req = self.channel_controller.lock().await;
        let inner = req.subscriber().expect("Exceeded");
        EventBusSubscriber::new(inner, req.get_status())
    }
}

#[cfg(test)]
mod test {

    use crate as hwa;
    use core::future;
    use core::future::Future;
    use hwa::{EventFlags, EventStatus};
    use std::sync::RwLock;
    use std::task::Poll;
    
    fn init_logging() {
        let env = env_logger::Env::new().default_filter_or("info");
        use std::io::Write;
        let _ = env_logger::builder()
            .parse_env(env)
            .format(|buf, record| writeln!(buf, "{}: {}", record.level(), record.args()))
            .try_init();
    }

    #[test]
    fn test_flags() {
        init_logging();
        let flags = EventFlags::all();
        hwa::info!("all flags: {:?}", flags);
        let event_status = EventStatus::not_containing(EventFlags::SYS_BOOTING);
        hwa::info!("event status: {:?}", event_status);
    }

    type ChannelControllerMutexType = hwa::AsyncCsMutexType;
    type PubSubMutexType = hwa::AsyncCsMutexType;
    type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
    type MutexStrategyType = hwa::AsyncHoldableStrategy<ChannelControllerMutexType, ResourceType>;

    #[futures_test::test]
    async fn test_manual() {
        use crate as printhor_hwa_utils;

        let controller = hwa::GenericEventBus::new(hwa::make_static_async_controller!(
            "EventBus",
            MutexStrategyType,
            hwa::EventBusChannelController::new(hwa::make_static_ref!(
                "EventBusChannel",
                hwa::EventBusPubSubType<PubSubMutexType>,
                hwa::EventBusPubSubType::new(),
            )),
        ));

        let cloned_controller = controller.clone();

        controller
            .publish_event(EventStatus::containing(EventFlags::SYS_ALARM))
            .await;
        let st1 = controller.get_status().await;
        let st2 = cloned_controller.get_status().await;
        assert_eq!(st1, st2, "State is common across EventBus cloned instance");
    }

    static EVENT_BUS: RwLock<Option<hwa::GenericEventBus<MutexStrategyType, PubSubMutexType>>> =
        RwLock::new(None);

    fn initialize() {
        let env = env_logger::Env::new().default_filter_or("info");
        use std::io::Write;
        let _ = env_logger::builder()
            .parse_env(env)
            .format(|buf, record| writeln!(buf, "{}: {}", record.level(), record.args()))
            .try_init();
        let mut global = EVENT_BUS.write().unwrap();

        if global.is_none() {
            global.replace(hwa::GenericEventBus::new(
                hwa::make_static_async_controller!(
                    "EventBus",
                    MutexStrategyType,
                    hwa::EventBusChannelController::new(hwa::make_static_ref!(
                        "EventBusChannel",
                        hwa::EventBusPubSubType<PubSubMutexType>,
                        hwa::EventBusPubSubType::new(),
                    )),
                ),
            ));
        }
    }

    #[futures_test::test]
    async fn test_event_bus() {
        init_logging();
        initialize();
        let mg = EVENT_BUS.read().unwrap();
        let event_bus = mg.as_ref().unwrap();

        let event_status = EventStatus::new()
            .and_containing(EventFlags::SYS_BOOTING)
            .and_not_containing(EventFlags::SYS_BOOT_FAILURE);
        hwa::info!("{:?}", event_status);
        hwa::info!("{:?}", event_status);
        event_bus.publish_event(event_status).await;

        assert!(
            event_bus.has_flags(EventFlags::SYS_BOOTING).await,
            "StateChecks [1]: Initially, flag SYS_BOOTING is set"
        );
        assert!(
            !event_bus.has_flags(EventFlags::SYS_BOOT_FAILURE).await,
            "StateChecks [1]: Initially, flag SYS_BOOT_FAILURE is not set"
        );
        let mut subscriber = event_bus.subscriber().await;

        let the_flags = subscriber.get_status().await;
        assert_eq!(
            the_flags,
            EventFlags::SYS_BOOTING,
            "StateChecks [1]: flag SYS_BOOTING read from a lately created subscriber"
        );

        let the_flags = subscriber.get_status().await;
        assert_eq!(
            the_flags,
            EventFlags::SYS_BOOTING,
            "StateChecks [1]: Again, flag SYS_BOOTING read from a lately created subscriber (idempotence test)"
        );

        future::poll_fn(|cx| {
            {
                let pinned_subscriber_future1 =
                    core::pin::pin!(subscriber.ft_wait_while(EventFlags::SYS_BOOTING));
                let p1 = pinned_subscriber_future1.poll(cx);
                assert_eq!(
                    p1,
                    Poll::Pending,
                    "StateChecks [1]: Subscriber would wait WHILE SYS_BOOTING"
                );
            }

            {
                let pinned_subscriber_future2 =
                    core::pin::pin!(subscriber.ft_wait_until_reset(EventFlags::SYS_BOOTING));
                let p2 = pinned_subscriber_future2.poll(cx);
                assert_eq!(
                    p2,
                    Poll::Pending,
                    "StateChecks [1]: Subscriber would wait UNTIL SYS_BOOTING is reset"
                );
            }
            {
                let pinned_subscriber_future3 =
                    core::pin::pin!(subscriber.ft_wait_until(EventFlags::SYS_READY));
                let p3 = pinned_subscriber_future3.poll(cx);
                assert_eq!(
                    p3,
                    Poll::Pending,
                    "StateChecks [1]: Subscriber would wait UNTIL SYS_READY"
                );
                Poll::Ready(())
            }
        })
        .await;

        future::poll_fn(|cx| {
            let status = EventStatus::not_containing(EventFlags::SYS_BOOTING);
            let mut pinned_subscriber_future = core::pin::pin!(subscriber.ft_wait_for(status));
            let p1 = pinned_subscriber_future.as_mut().poll(cx);
            assert_eq!(
                p1,
                Poll::Pending,
                "StateChecks [1]: Subscriber would wait FOR !SYS_BOOTING"
            );

            {
                let mut pinned_publisher_future = core::pin::pin!(event_bus.publish_event(status));
                let p2 = pinned_publisher_future.as_mut().poll(cx);
                assert_ne!(
                    p2,
                    Poll::Pending,
                    "StateChecks [2]: Publisher allows to publish immediately !SYS_BOOTING"
                );
            }

            let p3 = pinned_subscriber_future.as_mut().poll(cx);
            assert_ne!(
                p3,
                Poll::Pending,
                "StateChecks [3]: Subscriber now does NOT wait FOR !SYS_BOOTING"
            );

            Poll::Ready(())
        })
        .await;
    }
}
