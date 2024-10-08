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
//! * The [StaticController] interface, with the proper alternatives based on expected behavior
//!
//! * A convenient exports of [trace], [debug], [info], [warn], [error] log macros redirected
//! to the feature-based log implementation (`defmt` or `log` crates)
//!
//! # Example
//! ```rust
//! use printhor_hwa_utils as hwa;
//!
//! struct DummyDevice {}
//!
//! type MutexType = hwa::SyncSendMutex;
//! static MUTEX_INSTANCE: hwa::StaticCell<hwa::Mutex<MutexType, DummyDevice>> = hwa::StaticCell::new();
//! let controller: hwa::StaticController<hwa::Holdable<MutexType, DummyDevice>> =
//!     hwa::StaticController::new(
//!         hwa::Holdable::new(
//!             MUTEX_INSTANCE.init(
//!                 hwa::Mutex::new(DummyDevice{})
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
use core::cell::RefCell;
use core::ops::{Deref, DerefMut};
use futures;
use portable_atomic::{AtomicUsize, Ordering};
pub use static_cell::StaticCell;

//#region "Utilities"

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

//#endregion

//#region "Useful reexports"

pub use embassy_sync::mutex::Mutex;
pub use embassy_sync::mutex::MutexGuard;

/// A shortcut for [embassy_sync::blocking_mutex::raw::RawMutex]
pub use embassy_sync::blocking_mutex::raw::RawMutex;

/// A [core::marker::Sync] and [core::marker::Send] mutex type. Thread-safe even between cores.
/// A shortcut for [embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex]
pub use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as SyncSendMutex;

/// A NOT [core::marker::Sync] but [core::marker::Send] mutex type. Safe in single thread async runtime only.
/// A shortcut for [embassy_sync::blocking_mutex::raw::NoopRawMutex]
pub use embassy_sync::blocking_mutex::raw::NoopRawMutex as NoopMutex;

//#endregion

//#region "The Raw HWi resource wrapper"
pub trait RawHwiResource: Deref {
    type Resource;
    fn take(self) -> Self::Resource;
}

//#endregion

//#region "The `MutexStrategy` Mutex trait"

pub trait MutexStrategy
where
    Self::MutexType: RawMutex + 'static,
    Self::Resource: Sized + 'static,
    Self: Clone + Deref<Target = Mutex<Self::MutexType, Self::Resource>>,
{
    const CAN_RETAIN: bool;
    type MutexType;
    type Resource;

    fn can_retain(&self) -> bool {
        Self::CAN_RETAIN
    }

    fn retain(&self) -> impl futures::Future<Output = Result<(), ()>> + Send {
        async { Err(()) }
    }

    fn release(&self) -> Result<(), ()> {
        Err(())
    }

    fn apply_or_error<F, U, E>(&self, f: F, error: E) -> Result<U, E>
    where
        F: FnOnce(&mut Self::Resource) -> Result<U, E>;
}

//#endregion

//#region "A Generic NOT Holdable Mutex"

pub struct NotHoldable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    mutex: &'static Mutex<M, D>,
}

impl<M, D> NotHoldable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    pub const fn new(mutex: &'static Mutex<M, D>) -> Self {
        Self { mutex }
    }
}

impl<M, D> Clone for NotHoldable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    fn clone(&self) -> Self {
        Self { mutex: self.mutex }
    }
}

impl<M, D> Deref for NotHoldable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    type Target = Mutex<M, D>;
    fn deref(&self) -> &Self::Target {
        &self.mutex
    }
}

impl<M, D> MutexStrategy for NotHoldable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    const CAN_RETAIN: bool = false;
    type MutexType = M;
    type Resource = D;

    // Not supported in not holdable
    fn apply_or_error<F, U, E>(&self, _f: F, error: E) -> Result<U, E>
    where
        F: FnOnce(&mut Self::Resource) -> Result<U, E>,
    {
        Err(error)
    }
}

//#endregion

//#region "A Generic Holdable Mutex"

//#region "The MutexStrategy"

pub struct Holder<M, T>
where
    M: RawMutex + 'static,
    T: Sized + 'static,
{
    retained_guard: embassy_sync::blocking_mutex::CriticalSectionMutex<
        RefCell<Option<MutexGuard<'static, M, T>>>,
    >,
}
impl<M, T> Holder<M, T>
where
    M: RawMutex + 'static,
    T: Sized + 'static,
{
    const fn new() -> Self {
        Self {
            retained_guard: embassy_sync::blocking_mutex::CriticalSectionMutex::new(RefCell::new(
                None,
            )),
        }
    }
    fn set(&self, guard: MutexGuard<'static, M, T>) {
        self.retained_guard
            .lock(|retained| match retained.try_borrow_mut() {
                Ok(mut _m) => {
                    _m.replace(guard);
                }
                Err(_e) => {
                    unreachable!("Unable to borrow MutexStrategy");
                }
            });
    }
    fn release(&self) {
        self.retained_guard
            .lock(move |retained| match retained.try_borrow_mut() {
                Ok(mut _m) => {
                    let _trash = _m.take();
                }
                Err(_e) => {
                    unreachable!("Unable to borrow MutexStrategy");
                }
            });
    }
}

//#endregion

pub struct Holdable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    mutex: &'static Mutex<M, D>,
    holder: Holder<M, D>,
}

impl<M, D> Holdable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    pub const fn new(mutex: &'static Mutex<M, D>) -> Self {
        Self {
            mutex,
            holder: Holder::new(),
        }
    }
}

impl<M, D> Clone for Holdable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + Sync + Send + 'static,
    Self: Sync,
{
    fn clone(&self) -> Self {
        Self {
            mutex: self.mutex,
            holder: Holder::new(),
        }
    }
}

impl<M, D> Deref for Holdable<M, D>
where
    M: RawMutex + 'static,
    D: Sized + 'static,
{
    type Target = Mutex<M, D>;
    fn deref(&self) -> &Self::Target {
        &self.mutex
    }
}

impl<M, D> MutexStrategy for Holdable<M, D>
where
    M: RawMutex + Sync + 'static,
    D: Sized + Sync + Send + 'static,
    Self: Sync,
{
    const CAN_RETAIN: bool = true;
    type MutexType = M;
    type Resource = D;

    fn retain(&self) -> impl futures::Future<Output = Result<(), ()>> + Send {
        async {
            self.holder.set(self.mutex.lock().await);
            Ok(())
        }
    }

    fn release(&self) -> Result<(), ()> {
        self.holder.release();
        Ok(())
    }

    fn apply_or_error<F, U, E>(&self, f: F, error: E) -> Result<U, E>
    where
        F: FnOnce(&mut Self::Resource) -> Result<U, E>,
    {
        self.holder.retained_guard.lock(|retained| {
            if let Ok(mut ref_mut) = retained.try_borrow_mut() {
                match ref_mut.as_mut() {
                    Some(guard) => f(guard.deref_mut()),
                    None => Err(error),
                }
            } else {
                Err(error)
            }
        })
    }
}

//#endregion

//#region "Static Controller"

pub struct StaticController<H>
where
    H: MutexStrategy,
    <H as MutexStrategy>::MutexType: RawMutex + 'static,
    <H as MutexStrategy>::Resource: 'static,
{
    wrapped_mutex: H,
}

impl<H> StaticController<H>
where
    H: MutexStrategy + 'static,
    <H as MutexStrategy>::MutexType: RawMutex + 'static,
    <H as MutexStrategy>::Resource: 'static,
{
    pub const fn new(mutex: H) -> Self {
        Self {
            wrapped_mutex: mutex,
        }
    }
}

impl<H> Clone for StaticController<H>
where
    H: MutexStrategy + 'static,
    <H as MutexStrategy>::MutexType: RawMutex + 'static,
    <H as MutexStrategy>::Resource: 'static,
{
    fn clone(&self) -> Self {
        StaticController {
            wrapped_mutex: self.wrapped_mutex.clone(),
        }
    }
}

impl<H> Deref for StaticController<H>
where
    H: MutexStrategy + 'static,
    <H as MutexStrategy>::MutexType: RawMutex + 'static,
    <H as MutexStrategy>::Resource: 'static,
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

//#region "Tests"

#[cfg(test)]
mod test {
    use crate as hwa;
    use hwa::MutexStrategy;
    use std::clone::Clone;

    struct DummyDevice {}
    impl DummyDevice {
        pub fn new() -> Self {
            Self {}
        }

        pub fn do_nothing(&mut self) {
            hwa::info!("Did something")
        }
    }

    #[futures_test::test]
    async fn test_not_holdable_static_controller() {
        type MutexType = hwa::NoopMutex;
        static CELL_INSTANCE: hwa::StaticCell<hwa::Mutex<MutexType, DummyDevice>> =
            hwa::StaticCell::new();

        let controller: hwa::StaticController<hwa::NotHoldable<MutexType, DummyDevice>> =
            hwa::StaticController::new(hwa::NotHoldable::new(
                CELL_INSTANCE.init(hwa::Mutex::new(DummyDevice::new())),
            ));

        assert!(
            !controller.can_retain(),
            "A NotHoldable StaticController can NOT retain"
        );

        let _r = controller.retain().await;

        let mut mg = controller.lock().await;
        mg.do_nothing();
    }

    #[futures_test::test]
    async fn test_holdable_static_controller() {
        type MutexType = hwa::SyncSendMutex;
        static MUTEX_INSTANCE: hwa::StaticCell<hwa::Mutex<MutexType, DummyDevice>> =
            hwa::StaticCell::new();

        let controller: hwa::StaticController<hwa::Holdable<MutexType, DummyDevice>> =
            hwa::StaticController::new(hwa::Holdable::new(
                MUTEX_INSTANCE.init(hwa::Mutex::new(DummyDevice::new())),
            ));

        assert!(
            controller.can_retain(),
            "A Holdable StaticController can retain"
        );
        assert!(
            controller.retain().await.is_ok(),
            "A Holdable StaticController retains"
        );
        assert!(
            controller.try_lock().is_err(),
            "A retained Holdable StaticController cannot be lock until released"
        );
        assert!(
            controller.release().is_ok(),
            "A Holdable StaticController releases"
        );

        assert!(
            controller.retain().await.is_ok(),
            "A released Holdable StaticController retains again"
        );

        let other_controller = controller.clone();

        let _c = other_controller.apply_or_error(|_d| Ok(_d.do_nothing()), ());

        let _c2 = controller.apply_or_error(|_d| Ok(_d.do_nothing()), ());
    }
}

//#endregion
