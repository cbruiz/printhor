#![cfg_attr(not(feature = "std"), no_std)]

//! A module for tracking static cell memory usage.
//!
//! This module defines a simple monitor to track the amount of memory used for controllers and machinery
//! using static cells. It provides a `TrackedStaticCell` wrapper around `StaticCell` that tracks memory
//! usage when initializing static cells. The memory usage is tracked by incrementing a global counter.
//!
//! # Example
//! ```
//! // [...]
//! ```
//!
//! # Features
//! - `with-log`: Enables logging of allocation details when enabled.
//! - `with-defmt`: Enables logging of allocation details when enabled.

use core::ops::Deref;
use portable_atomic::{AtomicUsize, Ordering};
pub use static_cell::StaticCell;

/// Global counter to keep track of memory allocation.
#[const_env::from_env("MAX_STATIC_ALLOC_BYTES")]
pub const MAX_STATIC_ALLOC_BYTES: usize = 16384;

static COUNTER: AtomicUsize = AtomicUsize::new(0);

/// Increments the stack reservation counter.
///
/// This function checks if the number of bytes `nbytes` exceeds the `MAX_SIZE`.
/// If it does, it panics. Otherwise, it increments a global counter by `nbytes`.
///
/// # Panics
///
/// Panics if `nbytes` is greater than `MAX_SIZE`.
pub fn stack_allocation_increment(nbytes: usize) -> Result<usize, usize> {
    let prev_size = COUNTER.fetch_add(nbytes, Ordering::Relaxed);
    if prev_size + nbytes > MAX_STATIC_ALLOC_BYTES {
        COUNTER.sub(nbytes, Ordering::Relaxed);
        Err(nbytes)
    } else {
        Ok(nbytes)
    }
}

pub fn stack_allocation_get() -> usize {
    COUNTER.load(Ordering::Relaxed)
}

// For allocation tests, so allocation can be "undoed"
#[cfg(test)]
pub fn stack_allocation_decrement(nbytes: usize) {
    COUNTER.fetch_sub(nbytes, Ordering::Relaxed);
}

pub struct StaticController<M, D>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    D: 'static,
{
    mutex: &'static embassy_sync::mutex::Mutex<M, D>,
}

impl<M, D> StaticController<M, D>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
    D: 'static,
{
    pub const fn new(mutex: &'static embassy_sync::mutex::Mutex<M, D>) -> Self {
        Self { mutex }
    }
}

impl<M, D> Clone for StaticController<M, D>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    fn clone(&self) -> Self {
        StaticController { mutex: self.mutex }
    }
}

impl<M, D> Deref for StaticController<M, D>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex + 'static,
{
    type Target = embassy_sync::mutex::Mutex<M, D>;

    fn deref(&self) -> &Self::Target {
        self.mutex
    }
}

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
