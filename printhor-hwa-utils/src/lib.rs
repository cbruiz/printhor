//! A module for managing static resources.
//!
//! This module defines:
//!
//! * A simple monitor to track the amount of memory used for controllers and machinery
//! using static cells [stack_allocation_increment], [stack_allocation_get].
//! The memory usage is tracked by incrementing a global counter.
//! Max memory is set by constant [MAX_STATIC_ALLOC_BYTES] that can be overridden with the proper
//! environment variable.
//!
//! * The [StaticSyncController] interface, with the proper alternatives based on expected behavior
//! * The [StaticAsyncController] interface, with the proper alternatives based on expected behavior
//!
//! * A convenient exports of [trace], [debug], [info], [warn], [error] log macros redirected
//! to the feature-based log implementation (`defmt` or `log` crates)
//!
//! # Example
//! ```rust
//! use printhor_hwa_utils as hwa;
//! use hwa::SyncMutexStrategy;
//! use core::cell::RefCell;
//!
//! struct DummyDevice {
//!     value: u32,
//! }
//! impl DummyDevice {
//!
//!     const fn new() -> Self { Self {value: 0} }
//!
//!     fn increment(&mut self) {
//!         self.value += 1
//!     }
//!     fn get_count(&self) -> u32 {
//!         self.value
//!     }
//! }
//!
//! type SyncMutexType = hwa::SyncNoopMutexType;
//! static SYNC_MUTEX_INSTANCE: hwa::StaticCell<hwa::SyncMutex<SyncMutexType, DummyDevice>> = hwa::StaticCell::new();
//! let controller: hwa::StaticSyncController<hwa::SyncStandardStrategy<SyncMutexType, DummyDevice>> =
//!     hwa::StaticSyncController::new(
//!         hwa::SyncStandardStrategy::new(
//!             SYNC_MUTEX_INSTANCE. init(
//!                 hwa::SyncMutex::new(
//!                     RefCell::new(DummyDevice::new())
//!                 )
//!             )
//!         )
//!     );
//!
//! // can be cloned
//! let c = controller.clone();
//! let _v1 = c.apply(|_my| {
//!     _my.get_count()
//! });
//! c.apply_mut(|_my| {
//!     _my.increment()
//! });
//! let _v2 = c.apply(|_my| {
//!     _my.get_count()
//! });
//! let _v3 = c.apply(|_my| {
//!     _my.get_count()
//! });
//! assert_eq!(_v1, 0);
//! assert_eq!(_v2, 1);
//! assert_eq!(_v3, 1);
//!
//! // Async controller:
//!
//! type AsyncMutexType = hwa::AsyncNoopMutexType;
//! static ASYNC_MUTEX_INSTANCE: hwa::StaticCell<hwa::AsyncMutex<AsyncMutexType, DummyDevice>> = hwa::StaticCell::new();
//! let controller: hwa::StaticAsyncController<hwa::AsyncHoldableStrategy<AsyncMutexType, DummyDevice>> =
//!     hwa::StaticAsyncController::new(
//!         hwa::AsyncHoldableStrategy::new(
//!             ASYNC_MUTEX_INSTANCE.init(
//!                 hwa::AsyncMutex::new(DummyDevice::new())
//!             )
//!         )
//!     );
//! ```
//!
//! # Features
//! - `with-log`: Enables logging of allocation details when enabled.
//! - `with-defmt`: Enables logging of allocation details when enabled.
//! - otherwise, `no_log` internal backend is used (Will completely discard the expression)
#![cfg_attr(not(feature = "std"), no_std)]

mod mutex_sync;
pub use mutex_sync::*;
mod mutex_async;
pub use mutex_async::*;

use core::ops::Deref;
use portable_atomic::{AtomicUsize, Ordering};
pub use static_cell::StaticCell;

//#region "Utilities"

/// Global counter to keep track of memory allocation.

static COUNTER: AtomicUsize = AtomicUsize::new(0);

/// Increments the stack reservation counter.
pub fn stack_allocation_increment(nbytes: usize) -> Result<usize, usize> {
    let _ = COUNTER.fetch_add(nbytes, Ordering::Relaxed);
    Ok(nbytes)
}

pub fn stack_allocation_get() -> usize {
    COUNTER.load(Ordering::Relaxed)
}

// For allocation tests, so allocation can be "undoed"
#[cfg(test)]
pub fn stack_allocation_decrement(nbytes: usize) {
    COUNTER.fetch_sub(nbytes, Ordering::Relaxed);
}

//#endregion

//#region "The Raw HWi resource wrapper"
pub trait RawHwiResource: Deref {
    type Resource;
    fn take(self) -> Self::Resource;
}

//#endregion

//#region "Static Sync Controller"

pub struct StaticSyncController<H>
where
    H: SyncMutexStrategy,
    <H as SyncMutexStrategy>::SyncMutexType: SyncRawMutex + 'static,
    <H as SyncMutexStrategy>::Resource: 'static,
{
    wrapped_mutex: H,
}

impl<H> StaticSyncController<H>
where
    H: SyncMutexStrategy + 'static,
    <H as SyncMutexStrategy>::SyncMutexType: SyncRawMutex + 'static,
    <H as SyncMutexStrategy>::Resource: 'static,
{
    pub const fn new(mutex: H) -> Self {
        Self {
            wrapped_mutex: mutex,
        }
    }
}

impl<H> Clone for StaticSyncController<H>
where
    H: SyncMutexStrategy + 'static,
    <H as SyncMutexStrategy>::SyncMutexType: SyncRawMutex + 'static,
    <H as SyncMutexStrategy>::Resource: 'static,
{
    fn clone(&self) -> Self {
        StaticSyncController {
            wrapped_mutex: self.wrapped_mutex.clone(),
        }
    }
}

impl<H> Deref for StaticSyncController<H>
where
    H: SyncMutexStrategy + 'static,
    <H as SyncMutexStrategy>::SyncMutexType: SyncRawMutex + 'static,
    <H as SyncMutexStrategy>::Resource: 'static,
{
    type Target = H;

    fn deref(&self) -> &Self::Target {
        &self.wrapped_mutex
    }
}

//#endregion

//#region "Static Async Controller"

pub struct StaticAsyncController<H>
where
    H: AsyncMutexStrategy,
    <H as AsyncMutexStrategy>::AsyncMutexType: AsyncRawMutex + 'static,
    <H as AsyncMutexStrategy>::Resource: 'static,
{
    wrapped_mutex: H,
}

impl<H> StaticAsyncController<H>
where
    H: AsyncMutexStrategy + 'static,
    <H as AsyncMutexStrategy>::AsyncMutexType: AsyncRawMutex + 'static,
    <H as AsyncMutexStrategy>::Resource: 'static,
{
    pub const fn new(mutex: H) -> Self {
        Self {
            wrapped_mutex: mutex,
        }
    }
}

impl<H> Clone for StaticAsyncController<H>
where
    H: AsyncMutexStrategy + 'static,
    <H as AsyncMutexStrategy>::AsyncMutexType: AsyncRawMutex + 'static,
    <H as AsyncMutexStrategy>::Resource: 'static,
{
    fn clone(&self) -> Self {
        StaticAsyncController {
            wrapped_mutex: self.wrapped_mutex.clone(),
        }
    }
}

impl<H> Deref for StaticAsyncController<H>
where
    H: AsyncMutexStrategy + 'static,
    <H as AsyncMutexStrategy>::AsyncMutexType: AsyncRawMutex + 'static,
    <H as AsyncMutexStrategy>::Resource: 'static,
{
    type Target = H;

    fn deref(&self) -> &Self::Target {
        &self.wrapped_mutex
    }
}

//#endregion

//#region "Logging redirections"

cfg_if::cfg_if! {
    if #[cfg(feature = "with-log")] {
        pub use log::{trace, debug, info, warn, error};
    }
    else if #[cfg(feature = "with-defmt")] {
        pub use defmt::{trace, debug, info, warn, error};
    }
    else {
        pub use printhor_hwa_common_macros::no_log as trace;
        pub use printhor_hwa_common_macros::no_log as debug;
        pub use printhor_hwa_common_macros::no_log as info;
        pub use printhor_hwa_common_macros::no_log as warn;
        pub use printhor_hwa_common_macros::no_log as error;
    }
}

//#endregion
