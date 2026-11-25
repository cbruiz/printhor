//! A module for managing async locks.

//#region Useful reexports [...] Async mutexes
#[allow(unused)]
use crate as hwa;
use core::cell::RefCell;
use core::ops::Deref;

pub use embassy_sync::mutex::Mutex as AsyncMutex;
pub use embassy_sync::mutex::MutexGuard as AsyncMutexGuard;

/// A shortcut for [embassy_sync::blocking_mutex::raw::RawMutex]
pub use embassy_sync::blocking_mutex::raw::RawMutex as AsyncRawMutex;

/// A [Sync] and [Send] mutex instance type. Thread-safe even between cores.
/// A shortcut for [embassy_sync::mutex::Mutex] with mutex type [embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex]
pub type AsyncCsMutex<D> =
    embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, D>;

/// A NOT [Sync] but [Send] mutex instance type. Safe in single thread async runtime only.
/// /// A shortcut for [embassy_sync::mutex::Mutex] with mutex type [embassy_sync::blocking_mutex::raw::NoopRawMutex]
pub type AsyncNoopMutex<D> =
    embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::NoopRawMutex, D>;

/// A [core::marker::Sync] and [core::marker::Send] mutex type. Thread-safe even between cores.
/// A shortcut for [embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex]
pub use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as AsyncCsMutexType;

use crate::SyncCsMutex;
/// A NOT [core::marker::Sync] but [core::marker::Send] mutex type. Safe in single thread async runtime only.
/// A shortcut for [embassy_sync::blocking_mutex::raw::NoopRawMutex]
pub use embassy_sync::blocking_mutex::raw::NoopRawMutex as AsyncNoopMutexType;
//#endregion

//#region The AsyncMutexStrategy trait [...]

pub trait AsyncMutexStrategy
where
    Self::AsyncMutexType: AsyncRawMutex + 'static,
    Self::Resource: Sized + 'static,
    Self: Clone + Deref<Target = AsyncMutex<Self::AsyncMutexType, Self::Resource>>,
{
    const CAN_RETAIN: bool;
    type AsyncMutexType;
    type Resource;

    fn can_retain(&self) -> bool {
        Self::CAN_RETAIN
    }

    fn retain(&self) -> impl core::future::Future<Output = Result<(), ()>> {
        hwa::error!("cannot retain");
        async { Err(()) }
    }

    fn release(&self) -> Result<(), ()> {
        hwa::error!("cannot release");
        Err(())
    }

    /// First step of a retain from a future that must not borrow `self`: locks the static mutex
    /// (shared by every clone) and yields its guard, to be kept by [Self::hold]. `None` when the
    /// strategy can not retain.
    ///
    /// Retaining a clone instead keeps nothing for `self`: each clone has its own holder (and
    /// the guard is dropped with the clone, unlocking the mutex).
    fn lock_owned(
        &self,
    ) -> impl core::future::Future<
        Output = Option<AsyncMutexGuard<'static, Self::AsyncMutexType, Self::Resource>>,
    > + use<Self> {
        async { None }
    }

    /// Second step of [Self::lock_owned]: `self` keeps the guard (as [Self::retain] does)
    fn hold(
        &self,
        _guard: AsyncMutexGuard<'static, Self::AsyncMutexType, Self::Resource>,
    ) -> Result<(), ()> {
        Err(())
    }

    fn apply_or_error<F, U, E>(&self, f: F, error: E) -> Result<U, E>
    where
        F: FnOnce(&mut Self::Resource) -> Result<U, E>;
}

//#endregion

//#region "A Generic Mutex holder"

/// A piece for [AsyncHoldableStrategy]
pub struct Holder<M, T>
where
    M: AsyncRawMutex + 'static,
    T: Sized + 'static,
{
    retained_guard: SyncCsMutex<RefCell<Option<AsyncMutexGuard<'static, M, T>>>>,
}
impl<M, T> Holder<M, T>
where
    M: AsyncRawMutex + 'static,
    T: Sized + 'static,
{
    const fn new() -> Self {
        Self {
            retained_guard: SyncCsMutex::new(RefCell::new(None)),
        }
    }
    fn set(&self, guard: AsyncMutexGuard<'static, M, T>) {
        self.retained_guard.lock(|_m| {
            _m.borrow_mut().replace(guard);
        });
    }
    fn release(&self) {
        self.retained_guard.lock(move |retained| {
            let _ = retained.borrow_mut().take();
        });
    }
}

//#endregion

//#region The concrete strategies implementing the AsyncMutexStrategy trait [...]

pub struct AsyncStandardStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    mutex: &'static AsyncMutex<M, D>,
}

impl<M, D> AsyncStandardStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    pub const fn new(mutex: &'static AsyncMutex<M, D>) -> Self {
        Self { mutex }
    }
}

impl<M, D> Clone for AsyncStandardStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    fn clone(&self) -> Self {
        Self { mutex: self.mutex }
    }
}

impl<M, D> Deref for AsyncStandardStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    type Target = AsyncMutex<M, D>;
    fn deref(&self) -> &Self::Target {
        &self.mutex
    }
}

impl<M, D> AsyncMutexStrategy for AsyncStandardStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    const CAN_RETAIN: bool = false;
    type AsyncMutexType = M;
    type Resource = D;

    // Not supported in not holdable
    fn apply_or_error<F, U, E>(&self, _f: F, error: E) -> Result<U, E>
    where
        F: FnOnce(&mut Self::Resource) -> Result<U, E>,
    {
        Err(error)
    }
}

//==================================

pub struct AsyncHoldableStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    mutex: &'static AsyncMutex<M, D>,
    holder: Holder<M, D>,
}

impl<M, D> AsyncHoldableStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    pub const fn new(mutex: &'static AsyncMutex<M, D>) -> Self {
        Self {
            mutex,
            holder: Holder::new(),
        }
    }
}

impl<M, D> Clone for AsyncHoldableStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    fn clone(&self) -> Self {
        Self {
            mutex: self.mutex,
            holder: Holder::new(),
        }
    }
}

impl<M, D> Deref for AsyncHoldableStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    type Target = AsyncMutex<M, D>;
    fn deref(&self) -> &Self::Target {
        &self.mutex
    }
}

impl<M, D> AsyncMutexStrategy for AsyncHoldableStrategy<M, D>
where
    M: AsyncRawMutex + 'static,
    D: Sized + 'static,
{
    const CAN_RETAIN: bool = true;
    type AsyncMutexType = M;
    type Resource = D;

    fn retain(&self) -> impl core::future::Future<Output = Result<(), ()>> {
        async {
            self.holder.set(self.mutex.lock().await);
            Ok(())
        }
    }

    fn release(&self) -> Result<(), ()> {
        self.holder.release();
        Ok(())
    }

    fn lock_owned(
        &self,
    ) -> impl core::future::Future<Output = Option<AsyncMutexGuard<'static, M, D>>> + use<M, D> {
        let mutex = self.mutex;
        async move { Some(mutex.lock().await) }
    }

    fn hold(&self, guard: AsyncMutexGuard<'static, M, D>) -> Result<(), ()> {
        self.holder.set(guard);
        Ok(())
    }

    fn apply_or_error<F, U, E>(&self, f: F, error: E) -> Result<U, E>
    where
        F: FnOnce(&mut Self::Resource) -> Result<U, E>,
    {
        // The guard is taken out for the call and put back: only those two moves run in a
        // critical section, not `f`. Run inside it, a long operation (an SD card block over SPI:
        // hundreds of µs) masked every interrupt, the step ticker included (stalled motion).
        // `f` is synchronous and the holder belongs to the clone that retained it, so nothing
        // else uses it meanwhile
        let taken = self.holder.retained_guard.lock(|retained| retained.borrow_mut().take());
        match taken {
            Some(mut guard) => {
                let result = f(&mut guard);
                self.holder.set(guard);
                result
            }
            None => Err(error),
        }
    }
}

//#endregion

//#region "Tests"

#[cfg(test)]
mod test {
    use crate as hwa;
    use hwa::AsyncMutexStrategy;
    use std::clone::Clone;
    use std::ops::Deref;

    struct DummyDevice {}
    impl DummyDevice {
        pub const fn new() -> Self {
            Self {}
        }

        pub fn do_nothing(&mut self) {
            hwa::info!("Did something")
        }
    }

    /// A retain from a future that can not borrow the controller (the SD card adapter, inside
    /// `VolumeManager::device`): retaining a clone keeps nothing for the original (its own
    /// holder), `lock_owned` + `hold` does
    #[futures_test::test]
    async fn test_holdable_lock_owned_and_hold() {
        type MutexType = hwa::AsyncNoopMutexType;
        static CELL_INSTANCE: hwa::StaticCell<hwa::AsyncMutex<MutexType, DummyDevice>> =
            hwa::StaticCell::new();
        type Controller = hwa::StaticAsyncController<hwa::AsyncHoldableStrategy<MutexType, DummyDevice>>;
        let controller: Controller = hwa::StaticAsyncController::new(hwa::AsyncHoldableStrategy::new(
            CELL_INSTANCE.init(hwa::AsyncMutex::new(DummyDevice::new())),
        ));
        let apply = |c: &Controller| -> Result<(), ()> {
            c.apply_or_error(
                |d: &mut DummyDevice| {
                    d.do_nothing();
                    Ok(())
                },
                (),
            )
        };

        // The former bug: the clone is retained (and dropped), the original is not
        {
            let clone = controller.clone();
            assert_eq!(clone.retain().await, Ok(()));
        }
        assert_eq!(apply(&controller), Err(()), "a retained clone keeps nothing");

        // The future does not borrow the controller
        let lock = controller.lock_owned();
        let guard = lock.await.expect("holdable");
        assert_eq!(controller.hold(guard), Ok(()));
        assert_eq!(apply(&controller), Ok(()));
        assert_eq!(controller.release(), Ok(()));
        assert_eq!(apply(&controller), Err(()));
        // Released: it can be locked again
        assert!(controller.lock_owned().await.is_some());
    }

    #[futures_test::test]
    async fn test_standard_static_controller() {
        type MutexType = hwa::AsyncNoopMutexType;
        static CELL_INSTANCE: hwa::StaticCell<hwa::AsyncMutex<MutexType, DummyDevice>> =
            hwa::StaticCell::new();

        let controller: hwa::StaticAsyncController<
            hwa::AsyncStandardStrategy<MutexType, DummyDevice>,
        > = hwa::StaticAsyncController::new(hwa::AsyncStandardStrategy::new(
            CELL_INSTANCE.init(hwa::AsyncMutex::new(DummyDevice::new())),
        ));

        assert!(
            !controller.can_retain(),
            "A AsyncStandardStrategy can NOT retain"
        );

        let _r = controller.retain().await;
        assert_eq!(_r, Err(()), "NonHoldable cannot retain");
        let _r = controller.release();
        assert_eq!(_r, Err(()), "NonHoldable cannot release");

        let _x = controller.apply_or_error(|_| Ok(()), ());
        assert_eq!(_r, Err(()), "Not supported in not holdable");

        let mut mg = controller.lock().await;
        mg.do_nothing();

        let other_controller = controller.clone();

        let strategy = other_controller.deref();
        let _other_strategy = strategy.clone();
    }

    #[futures_test::test]
    async fn test_holdable_static_controller() {
        type MutexType = hwa::AsyncCsMutexType;
        static MUTEX_INSTANCE: hwa::StaticCell<hwa::AsyncMutex<MutexType, DummyDevice>> =
            hwa::StaticCell::new();

        let controller: hwa::StaticAsyncController<
            hwa::AsyncHoldableStrategy<MutexType, DummyDevice>,
        > = hwa::StaticAsyncController::new(hwa::AsyncHoldableStrategy::new(
            MUTEX_INSTANCE.init(hwa::AsyncMutex::new(DummyDevice::new())),
        ));

        let alloc_size = hwa::stack_allocation_get();

        assert!(
            controller.can_retain(),
            "A Holdable StaticAsyncController can retain"
        );
        assert!(
            controller.retain().await.is_ok(),
            "A Holdable StaticAsyncController retains"
        );
        assert!(
            controller.try_lock().is_err(),
            "Retained Holdable cannot be lock until released"
        );
        assert!(
            controller.release().is_ok(),
            "A Holdable StaticAsyncController releases"
        );

        assert!(
            controller.retain().await.is_ok(),
            "A released Holdable retains again"
        );

        let other_controller = controller.clone();

        let _c = other_controller.apply_or_error(|_d| Ok(_d.do_nothing()), ());

        let _c2 = controller.apply_or_error(|_d| Ok(_d.do_nothing()), ());

        let strategy = other_controller.deref();
        let _other_strategy = strategy.clone();

        hwa::stack_allocation_decrement(alloc_size);
    }
}

//#endregion
