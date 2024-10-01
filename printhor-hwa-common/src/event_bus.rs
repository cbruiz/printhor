//! The state manager controller.
//!
//! This module provides a global asynchronous event bus.

use core::fmt::Formatter;
use core::ops::{BitAnd, BitOr, BitXor};
use printhor_hwa_utils::StaticController;

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

/// Represents the EventBus responsible for managing and
/// publishing events within the system. The `EventBus`
/// structure holds a reference to the event bus, a publisher
/// for sending events, and the current status flags.
///
/// This structure ensures thread-safe operations through
/// asynchronous event handling and allows the system to
/// broadcast state changes and other relevant events globally.
///
/// # Fields
///
/// * `bus` - A static reference to the Pub/Sub event bus channel.
/// * `publisher` - A publisher that can send events immediately to the bus.
/// * `status` - Flags representing the current status of the system,
///              managed and updated by the event bus.
///
/// # Examples
///
/// See [EventBusController]
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

    pub(crate) fn get_status(&self) -> EventFlags {
        self.status
    }
}

//#endregion

//#region "Event Bus Channel Controller"

/// A struct representing a reference to the `EventBus`.
///
/// The `EventBusRef` provides various methods to interact with the event bus,
/// including publishing events, getting the current status, checking specific flags,
/// and subscribing to event notifications.
///
/// # Examples
///
/// ```rust
/// use printhor_hwa_common as hwa;
/// use hwa::{EventBusController, EventBusChannelController, EventBusPubSubType};
/// use hwa::{make_static_controller, make_static_ref};
///
/// type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
/// type ChannelMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
///
/// let _event_bus = EventBusController::new(
///     make_static_controller!(
///         "EventBusChannelController",
///         BusMutexType,
///         EventBusChannelController<ChannelMutexType>,
///         EventBusChannelController::new(
///             make_static_ref!(
///                 "EventBusChannel",
///                 EventBusPubSubType<ChannelMutexType>,
///                 EventBusPubSubType::new()
///             )
///         )
///     )
/// );
/// ```
///
/// # Fields
///
/// * `instance` - A reference to the controller wrapping the `EventBus`.
pub struct EventBusController<M, CM>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    CM: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    channel_controller: StaticController<M, EventBusChannelController<CM>>,
}

impl<M, CM> Clone for EventBusController<M, CM>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    CM: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    fn clone(&self) -> Self {
        EventBusController::new(self.channel_controller.clone())
    }
}

impl<M, CM> EventBusController<M, CM>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    CM: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    pub const fn new(
        channel_controller: StaticController<M, EventBusChannelController<CM>>,
    ) -> Self {
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
    /// use hwa::{EventBusController, EventFlags, EventStatus};
    ///
    /// type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    /// type ChannelMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    ///
    /// async fn assumed_initialization(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
    ///     event_bus.publish_event(EventStatus::not_containing(EventFlags::SYS_READY)).await
    /// }
    /// // [...]
    /// async fn event_routine(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
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
    /// use hwa::{EventBusController, EventFlags, EventStatus};
    ///
    /// type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    /// type ChannelMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;
    ///
    /// async fn assumed_initialization(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
    ///     event_bus.publish_event(EventStatus::not_containing(EventFlags::SYS_READY)).await
    /// }
    ///
    /// async fn checking_routine(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
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

    pub async fn subscriber(&self) -> EventBusSubscriber<CM> {
        let req = self.channel_controller.lock().await;
        let inner = req.channel.subscriber().expect("Exceeded");
        EventBusSubscriber {
            inner,
            last_status: req.get_status(),
        }
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

impl<M> EventBusSubscriber<'_, M>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    #[allow(unused)]
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
    /// use hwa::{EventBusController, EventFlags, EventStatus};
    ///
    /// type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    /// type ChannelMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;
    ///
    /// // See [EventBusController]
    /// // let event_bus: EventBusController<BusMutexType, ChannelMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
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
    /// use hwa::{EventBusController, EventFlags, EventStatus};
    ///
    /// type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    /// type ChannelMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;
    ///
    /// // See [EventBusController]
    /// // let event_bus: EventBusController<BusMutexType, ChannelMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
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
    /// use hwa::{EventBusController, EventFlags, EventStatus};
    ///
    /// type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    /// type ChannelMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;
    ///
    /// // See [EventBusController]
    /// // let event_bus: EventBusController<BusMutexType, ChannelMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
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
    /// use hwa::{EventBusController, EventFlags, EventStatus};
    ///
    /// type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    /// type ChannelMutexType = embassy_sync::blocking_mutex::raw::NoopRawMutex;
    ///
    /// // See [EventBusController]
    /// // let event_bus: EventBusController<BusMutexType, ChannelMutexType> = {
    /// // [...]
    /// // };
    ///
    /// async fn any_sample_task(event_bus: EventBusController<BusMutexType, ChannelMutexType>) {
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
/*
impl Clone for EventBusRef {
    fn clone(&self) -> Self {
        let pusher = self.instance.clone();

        EventBusRef { instance: pusher }
    }
}
*/

//#endregion

/// A [core::marker::Sync] and [core::marker::Send] mutex type. Thread-safe even between cores.
/// A shortcut for [embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex]
pub type SyncSendMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// A NOT [core::marker::Sync] but [core::marker::Send] mutex type. Safe in single thread async runtime only.
/// A shortcut for [embassy_sync::blocking_mutex::raw::NoopRawMutex]
pub type NoopMutex = embassy_sync::blocking_mutex::raw::NoopRawMutex;

#[cfg(test)]
mod test {
    use crate as hwa;
    use crate::{EventBusChannelController, EventBusController, EventBusPubSubType};
    use hwa::{EventFlags, EventStatus};
    use printhor_hwa_common_macros::{make_static_controller, make_static_ref};
    use std::sync::RwLock;

    type ControllerMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    type ChannelMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    static EVENT_BUS: RwLock<Option<EventBusController<ControllerMutexType, ChannelMutexType>>> =
        RwLock::new(None);
    fn initialize() {
        let mut global = EVENT_BUS.write().unwrap();

        if global.is_none() {
            global.replace(EventBusController::new(make_static_controller!(
                "EventBusChannelController",
                ControllerMutexType,
                EventBusChannelController<ChannelMutexType>,
                EventBusChannelController::new(make_static_ref!(
                    "EventBusChannel",
                    EventBusPubSubType<ChannelMutexType>,
                    EventBusPubSubType::new()
                ))
            )));
        }
    }

    #[futures_test::test]
    async fn test_event_bus() {
        initialize();
        let mg = EVENT_BUS.read().unwrap();
        let event_bus = mg.as_ref().unwrap();

        let _event_status = EventStatus::new()
            .and_containing(EventFlags::SYS_ALARM)
            .and_not_containing(EventFlags::SYS_BOOT_FAILURE);
        printhor_hwa_utils::info!("{:?}", _event_status);
        printhor_hwa_utils::info!("{:?}", _event_status);
        event_bus.publish_event(_event_status).await;
    }
}
#[cfg(test)]
mod test2 {
    #[test]
    fn runme() {
        use crate as hwa;
        use hwa::{make_static_controller, make_static_ref};
        use hwa::{EventBusChannelController, EventBusController, EventBusPubSubType};

        type BusMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
        type ChannelMutexType = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

        let _event_bus = EventBusController::new(make_static_controller!(
            "EventBusChannelController",
            BusMutexType,
            EventBusChannelController<ChannelMutexType>,
            EventBusChannelController::new(make_static_ref!(
                "EventBusChannel",
                EventBusPubSubType<ChannelMutexType>,
                EventBusPubSubType::new()
            ))
        ));
    }
}
