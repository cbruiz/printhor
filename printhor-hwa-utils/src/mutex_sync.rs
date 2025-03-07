//#region Useful reexports [...] Sync mutexes

pub type SyncMutex<M, D> = embassy_sync::blocking_mutex::Mutex<M, RefCell<D>>;
pub use embassy_sync::blocking_mutex::Mutex as SyncMutexGuard;

/// A shortcut for [embassy_sync::blocking_mutex::raw::RawMutex]
pub use embassy_sync::blocking_mutex::raw::RawMutex as SyncRawMutex;

/// A [core::marker::Sync] and [core::marker::Send] mutex instance type. Thread-safe even between cores.
/// A shortcut for [embassy_sync::mutex::Mutex] with mutex type [embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex]
pub type SyncCsMutex<D> = embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    D,
>;

/// A NOT [core::marker::Sync] but [core::marker::Send] mutex instance type. Safe in single thread async runtime only.
/// /// A shortcut for [embassy_sync::mutex::Mutex] with mutex type [embassy_sync::blocking_mutex::raw::NoopRawMutex]
pub type SyncNoopMutex<D> =
    embassy_sync::blocking_mutex::Mutex<embassy_sync::blocking_mutex::raw::NoopRawMutex, D>;

/// A [core::marker::Sync] and [core::marker::Send] mutex type. Thread-safe even between cores.
/// A shortcut for [embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex]
pub use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as SyncCsMutexType;

/// A NOT [core::marker::Sync] but [core::marker::Send] mutex type. Safe in single thread async runtime only.
/// A shortcut for [embassy_sync::blocking_mutex::raw::NoopRawMutex]
pub use embassy_sync::blocking_mutex::raw::NoopRawMutex as SyncNoopMutexType;

use core::cell::RefCell;

//#endregion

//#region The SyncMutexStrategy trait [...]

use core::ops::{Deref, DerefMut};

pub trait SyncMutexStrategy
where
    Self::SyncMutexType: SyncRawMutex + 'static,
    Self::Resource: Sized + 'static,
    Self: Clone + Deref<Target = SyncMutex<Self::SyncMutexType, Self::Resource>>,
{
    type SyncMutexType;
    type Resource;

    /// As the mutex strategy is not async, we have always the guarantee of being able to lock it
    fn apply<F, U>(&self, f: F) -> U
    where
        F: FnOnce(&Self::Resource) -> U;

    fn apply_mut<F, U>(&self, f: F) -> U
    where
        F: FnOnce(&mut Self::Resource) -> U;
}

//#endregion

//#region The concrete strategies implementing the SyncMutexStrategy trait [...]

pub struct SyncStandardStrategy<M, D>
where
    M: SyncRawMutex + 'static,
    D: Sized + 'static,
{
    mutex: &'static SyncMutex<M, D>,
}

impl<M, D> SyncStandardStrategy<M, D>
where
    M: SyncRawMutex + 'static,
    D: Sized + 'static,
{
    pub const fn new(mutex: &'static SyncMutex<M, D>) -> Self {
        Self { mutex }
    }
}

impl<M, D> Clone for SyncStandardStrategy<M, D>
where
    M: SyncRawMutex + 'static,
    D: Sized + 'static,
{
    fn clone(&self) -> Self {
        Self { mutex: self.mutex }
    }
}

impl<M, D> Deref for SyncStandardStrategy<M, D>
where
    M: SyncRawMutex + 'static,
    D: Sized + 'static,
{
    type Target = SyncMutex<M, D>;
    fn deref(&self) -> &Self::Target {
        self.mutex
    }
}

impl<M, D> SyncMutexStrategy for SyncStandardStrategy<M, D>
where
    M: SyncRawMutex + 'static,
    D: Sized + 'static,
{
    type SyncMutexType = M;
    type Resource = D;

    fn apply<F, U>(&self, f: F) -> U
    where
        F: FnOnce(&Self::Resource) -> U,
    {
        self.mutex.lock(|c| f(c.borrow().deref()))
    }

    fn apply_mut<F, U>(&self, f: F) -> U
    where
        F: FnOnce(&mut Self::Resource) -> U,
    {
        self.mutex.lock(|mg| f(mg.borrow_mut().deref_mut()))
    }
}

//#endregion

//#region "Tests"

#[cfg(test)]
mod test {
    use crate as hwa;
    use hwa::SyncMutexStrategy;
    use printhor_hwa_common_macros::make_static_sync_controller;
    use std::ops::Deref;

    struct DummyDevice {
        value: u32,
    }
    impl DummyDevice {
        const fn new() -> Self {
            Self { value: 0 }
        }

        fn increment(&mut self) {
            self.value += 1
        }
        fn get_count(&self) -> u32 {
            self.value
        }
    }

    #[test]
    fn test_standard_static_sync_controller() {
        use core::cell::RefCell;

        type SyncMutexType = hwa::SyncNoopMutexType;
        static SYNC_MUTEX_INSTANCE: hwa::StaticCell<hwa::SyncMutex<SyncMutexType, DummyDevice>> =
            hwa::StaticCell::new();
        let controller: hwa::StaticSyncController<
            hwa::SyncStandardStrategy<SyncMutexType, DummyDevice>,
        > = hwa::StaticSyncController::new(hwa::SyncStandardStrategy::new(
            SYNC_MUTEX_INSTANCE.init(hwa::SyncMutex::new(RefCell::new(DummyDevice::new()))),
        ));
        let c = controller.clone();
        let _v1 = c.apply(|_my| _my.get_count());
        c.apply_mut(|_my| _my.increment());
        let _v2 = c.apply(|_my| _my.get_count());
        let _v3 = c.apply(|_my| _my.get_count());
        assert_eq!(_v1, 0);
        assert_eq!(_v2, 1);
        assert_eq!(_v3, 1);
        let strategy: &hwa::SyncStandardStrategy<SyncMutexType, DummyDevice> = controller.deref();
        let inner_mutex = strategy.deref();
        assert!(inner_mutex.lock(|_| true), "can defer");
    }

    #[test]
    fn test_standard_static_sync_controller_macro() {
        use crate as printhor_hwa_utils;
        use printhor_hwa_common_macros::make_static_sync_controller;

        type MutexType = hwa::SyncNoopMutexType;
        type SyncMutexStrategy = hwa::SyncStandardStrategy<MutexType, DummyDevice>;

        let controller =
            make_static_sync_controller!("DummyDevice", SyncMutexStrategy, DummyDevice::new());

        let _c2 = controller.clone();
        hwa::info!("OK");
    }
}
//#endregion
