//! The state manager controller.
//!
//! This module provides a global asynchronous event bus.

use core::fmt::Formatter;
use core::ops::{BitAnd, BitOr, BitXor};
use crate::{TrackedStaticCell, ControllerMutex, ControllerRef, InterruptControllerMutexType};

pub type EventBusPubSubType = embassy_sync::pubsub::PubSubChannel<InterruptControllerMutexType, EventFlags, 1, 6, 1>;
pub type EventBusPublisherType = embassy_sync::pubsub::Publisher<'static, InterruptControllerMutexType, EventFlags, 1, 6, 1>;
pub type EventBusSubscriberType<'a> = embassy_sync::pubsub::Subscriber<'a, InterruptControllerMutexType, EventFlags, 1, 6, 1>;

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
        const NOTHING          = 0b0000000000000000;
        /// System alarm flag
        const SYS_ALARM        = 0b1000000000000000;
        /// System is booting
        const SYS_BOOTING      = 0b0100000000000000;
        /// System boot failure
        const SYS_BOOT_FAILURE = 0b0010000000000000;
        /// System is ready
        const SYS_READY        = 0b0001000000000000;
        /// ATX power is on
        const ATX_ON           = 0b0000100000000000;
        /// Homing operation in progress
        const HOMING          = 0b0000010000000000;
        /// System is moving
        const MOVING           = 0b0000001000000000;
        /// Movement queue is empty
        const MOV_QUEUE_EMPTY  = 0b0000000100000000;
        /// Movement queue is full
        const MOV_QUEUE_FULL   = 0b0000000010000000;
        /// Job printing in progress
        const JOB_PRINTING     = 0b0000000001000000;
        /// Job is paused
        const JOB_PAUSED       = 0b0000000000100000;
        /// Job completed
        const JOB_COMPLETED    = 0b0000000000010000;
        /// HotBed temperature is okay
        const HOT_BED_TEMP_OK   = 0b0000000000001000;
        /// HotEnd temperature is okay
        const HOT_END_TEMP_OK   = 0b0000000000000100;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature="with-defmt")] {
        impl defmt::Format for EventFlags {
            fn format(&self, fmt: defmt::Formatter) {
                let mut first = true;
                for (flag_name, _) in self.iter_names() {
                    if !first {
                        defmt::write!(fmt, " ");
                    }
                    else {
                        first = false;
                    }
                    defmt::write!(fmt, "{}", flag_name)
                }
            }
        }

    }

}
impl core::fmt::Debug for EventFlags {
    fn fmt(&self, fmt: &mut Formatter<'_>) -> core::fmt::Result {
        let mut first = true;
        for (flag_name, _flag_bits) in self.iter_names() {
            if !first {
                core::write!(fmt, " ")?;
            }
            else {
                first = false;
            }
            core::write!(fmt, "{}", flag_name)?;
        }
        Ok(())
    }

}

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
        Self {
            flags,
            mask: flags,
        }
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
            }
            else {
                first = false;
            }
            if self.flags.contains(flag_bits) {
                core::write!(fmt, "{}", flag_name)?;
            }
            else {
                core::write!(fmt, "!{}", flag_name)?;
            }
        }
        Ok(())
    }
}

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
/// See [EventBusRef]
pub struct EventBus {
    bus: &'static EventBusPubSubType,
    publisher: EventBusPublisherType,
    status: EventFlags,
}

impl EventBus {
    pub const fn new(bus: &'static EventBusPubSubType, publisher: EventBusPublisherType, status: EventFlags) -> Self {
        EventBus { bus, publisher, status }
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
/// const MAX_STATIC_MEMORY: usize = 1024;
///
/// let event_bus = {
///     static EVT_BUS: hwa::TrackedStaticCell<hwa::EventBusPubSubType> = hwa::TrackedStaticCell::new();
///     static EVT_CTRL_BUS: hwa::TrackedStaticCell<hwa::ControllerMutex<hwa::InterruptControllerMutexType, hwa::EventBus>> = hwa::TrackedStaticCell::new();
///     let bus = EVT_BUS.init::<MAX_STATIC_MEMORY>("EventBusChannel", hwa::EventBusPubSubType::new());
///     let publisher: hwa::EventBusPublisherType = bus.publisher().expect("publisher exausted");
///     hwa::EventBusRef::new(
///             hwa::ControllerRef::new(
///                 EVT_CTRL_BUS.init::<MAX_STATIC_MEMORY>("EventBus", hwa::ControllerMutex::new(
///                     hwa::EventBus::new( bus, publisher, hwa::EventFlags::empty())
///                 ))
///             )
///     )
/// };
/// ```
///
/// # Fields
///
/// * `instance` - A reference to the controller wrapping the `EventBus`.
pub struct EventBusRef {
    instance: ControllerRef<InterruptControllerMutexType, EventBus>,
}

pub struct EventBusSubscriber<'a> {
    inner: EventBusSubscriberType<'a>,
    last_status: EventFlags,
}

impl EventBusRef {
    pub fn new(c: ControllerRef<InterruptControllerMutexType, EventBus>) -> Self {
        Self {
            instance: c,
        }
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
    /// use hwa::{EventBusRef, EventFlags, EventStatus};
    ///
    /// async fn assumed_initialization(event_bus: EventBusRef) {
    ///     event_bus.publish_event(EventStatus::not_containing(EventFlags::SYS_READY)).await
    /// }
    ///
    /// async fn event_routine(event_bus: EventBusRef) {
    ///     // Update the event bus notifying that System is NOW ready 
    ///     event_bus.publish_event(EventStatus::containing(EventFlags::SYS_READY)).await;
    /// }
    /// ```
    ///
    pub async fn publish_event(&self, event: EventStatus) {
        let mut req = self.instance.lock().await;
        req.publish_event(event);
    }

    pub async fn get_status(&self) -> EventFlags {
        let req = self.instance.lock().await;
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
    /// use hwa::{EventBusRef, EventFlags, EventStatus}; // replace with your actual crate and module
    /// 
    /// async fn assumed_initialization(event_bus: EventBusRef) {
    ///     event_bus.publish_event(EventStatus::not_containing(EventFlags::SYS_READY)).await
    /// }
    /// 
    /// async fn checking_routine(event_bus: EventBusRef) {
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
    /// use hwa::{EventBusRef, EventFlags, EventStatus};
    ///
    /// async fn example_usage(event_bus: &mut hwa::EventBusSubscriber<'_>) {
    ///     match event_bus.ft_wait_for(EventStatus::containing(EventFlags::SYS_READY)).await {
    ///         Ok(()) => {
    ///             // Desired event occurred, handle success
    ///         }
    ///         Err(()) => {
    ///             // SYS_ALARM was set, handle error
    ///         }
    ///     }
    /// }
    /// ```
    ///
    pub async fn ft_wait_for(&mut self, what: EventStatus) -> Result<(), ()> {
        if what.flags.is_empty() {
            Ok(())
        }
        else {
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
                if !what.mask.contains(EventFlags::SYS_ALARM) && self.last_status.contains(EventFlags::SYS_ALARM) {
                    #[cfg(any(feature = "with-log", feature = "with-defmt"))]
                    crate::debug!("ft_wait_for [{:?}] -> ERR (took: {} us)", what, t0.elapsed().as_micros());
                    return Err(());
                }
                let relevant_bits = self.last_status.bitand(what.mask);
                if wanted.eq(&relevant_bits) {
                    #[cfg(any(feature = "with-log", feature = "with-defmt"))]
                    crate::debug!("ft_wait_for [{:?}] -> DONE (took: {} us)", what, t0.elapsed().as_micros());
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
    /// use hwa::{EventBusRef, EventFlags, EventStatus};
    ///
    /// async fn example_usage(event_bus: &mut hwa::EventBusSubscriber<'_>) {
    ///     match event_bus.ft_wait_until(EventFlags::SYS_BOOTING).await {
    ///         Ok(()) => {
    ///             // Desired flags are set, handle success
    ///         }
    ///         Err(()) => {
    ///             // SYS_ALARM was set, handle error
    ///         }
    ///     }
    /// }
    /// ```
    pub async fn ft_wait_until(&mut self, flags: EventFlags)  -> Result<(),()> {
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
    /// use hwa::{EventBusRef, EventFlags, EventStatus};
    ///
    /// async fn example_usage(event_bus: &mut hwa::EventBusSubscriber<'_>) {
    ///     match event_bus.ft_wait_until_reset(EventFlags::SYS_BOOTING).await {
    ///         Ok(()) => {
    ///             // Specified flags are reset, handle success
    ///         }
    ///         Err(()) => {
    ///             // SYS_ALARM was set, handle error
    ///         }
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
    /// use hwa::{EventBusRef, EventFlags, EventStatus};
    ///
    /// async fn example_usage(event_bus: &mut hwa::EventBusSubscriber<'_>) {
    ///     match event_bus.ft_wait_while(EventFlags::SYS_BOOTING).await {
    ///         Ok(()) => {
    ///             // Specified flags are set, handle success
    ///         }
    ///         Err(()) => {
    ///             // SYS_ALARM was set, handle error
    ///         }
    ///     }
    /// }
    /// ```
    pub async fn ft_wait_while(&mut self, flags: EventFlags) -> Result<(),()> {
        self.ft_wait_for(EventStatus::not_containing(flags)).await
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

/// Initializes the global EventBus with the given `MAX_SIZE`.
///
/// # Parameters
/// - `MAX_SIZE`: The maximum size of the event bus, which defines the capacity for event storage and handling.
///
/// # Returns
/// - `EventBusRef`: A reference to the globally initialized EventBus.
///
/// # Panics
/// This function will panic if the static cell initialization fails or if the publisher is exhausted.
///
/// # Note
/// This function can only be called once within the program. Subsequent calls will panic.
pub fn init_event_bus<const MAX_SIZE: usize>() -> EventBusRef {
    static EVT_BUS: TrackedStaticCell<EventBusPubSubType> = TrackedStaticCell::new();
    static EVT_CTRL_BUS: TrackedStaticCell<ControllerMutex<InterruptControllerMutexType, EventBus>> = TrackedStaticCell::new();

    let bus = EVT_BUS.init::<MAX_SIZE>("EventBusChannel", EventBusPubSubType::new());
    let publisher: EventBusPublisherType = bus.publisher().expect("publisher exausted");

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