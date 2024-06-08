use core::cell::RefCell;
use embassy_sync::mutex::TryLockError;
use crate::{ControllerMutexType, InterruptControllerMutexType};

pub type StandardControllerMutex<D> = embassy_sync::mutex::Mutex<ControllerMutexType, D>;
pub type InterruptControllerMutex<D> = embassy_sync::mutex::Mutex<InterruptControllerMutexType, D>;
pub type ControllerMutex<M, D> = embassy_sync::mutex::Mutex<M, D>;

pub trait ControllerKind<M: 'static, T: 'static> {
    type Type;
    type MutexType;
}

pub struct Holder<M: embassy_sync::blocking_mutex::raw::RawMutex + 'static, T: 'static> {
    pub r: RefCell<Option<embassy_sync::mutex::MutexGuard<'static, M, T>>>
}
impl<M: embassy_sync::blocking_mutex::raw::RawMutex + 'static, T: 'static> Holder<M, T> {
    const fn new() -> Self {
        Self {r: RefCell::new(None) }
    }
    fn set(&self, r: embassy_sync::mutex::MutexGuard<'static, M, T>) {
        self.r.borrow_mut().replace(r);
    }
    fn release(&self) {
        drop(self.r.borrow_mut().take());
    }
}

impl<M: embassy_sync::blocking_mutex::raw::RawMutex + 'static, T: 'static> Clone for Holder<M, T> {
    fn clone(&self) -> Self {
        Self {
            r: RefCell::new(None)
        }
    }
}

pub type InterruptControllerRef<T> = ControllerRef<InterruptControllerMutexType, T>;
pub type StandardControllerRef<T> = ControllerRef<ControllerMutexType, T>;

pub struct ControllerRef<M: embassy_sync::blocking_mutex::raw::RawMutex + 'static, T: 'static> {
    c: &'static embassy_sync::mutex::Mutex<M, T>,
    h: Holder<M, T>,
}
impl<M: embassy_sync::blocking_mutex::raw::RawMutex + 'static, T: 'static> ControllerKind<M, T> for ControllerRef<M, T> {
    type Type = T;
    type MutexType = M;
}

impl<M: embassy_sync::blocking_mutex::raw::RawMutex + 'static, T: 'static> ControllerRef<M, T>
{
    pub fn new(c: &'static embassy_sync::mutex::Mutex<M, T>) -> Self {
        Self{c, h: Holder::new()}
    }

    #[allow(unused)]
    #[inline]
    pub async fn lock(&self) -> embassy_sync::mutex::MutexGuard<'static, M, T> {
        self.c.lock().await
    }

    #[allow(unused)]
    #[inline]
    pub fn try_lock(&self) -> Result<embassy_sync::mutex::MutexGuard<'static, M, T>, TryLockError> {
        self.c.try_lock()
    }

    #[allow(unused)]
    #[inline]
    pub async fn retain(&self)  {
        let guard = self.c.lock().await;
        self.h.set(guard);
    }

    #[allow(unused)]
    #[inline]
    pub async fn release(&self)  {
        self.h.release();
    }

    #[allow(unused)]
    #[inline]
    pub fn apply<F>(&self, f: F) -> Result<(),()>
        where F: FnOnce(&mut <ControllerRef<M, T> as ControllerKind<M, T>>::Type) -> Result<(),()>
    {
        let mut h = self.h.r.borrow_mut();
        let x = h.as_mut();
        match x {
            None => {
                Err(())
            }
            Some(y) => {
                f(y)
            }
        }
    }


    #[allow(unused)]
    #[inline]
    pub fn apply_result<F, ROK, RKO>(&self, f: F, default_err: RKO) -> Result<ROK, RKO>
        where F: FnOnce(&mut <ControllerRef<M, T> as ControllerKind<M, T>>::Type) -> Result<ROK, RKO>
    {
        let mut h = self.h.r.borrow_mut();
        let x = h.as_mut();
        match x {
            None => {
                Err(default_err)
            }
            Some(y) => {
                f(y)
            }
        }
    }

}

impl<M: embassy_sync::blocking_mutex::raw::RawMutex + 'static, T: 'static> Clone for ControllerRef<M, T> {
    fn clone(&self) -> Self {
        ControllerRef {
            c: self.c,
            h: Holder::new(),
        }
    }
}