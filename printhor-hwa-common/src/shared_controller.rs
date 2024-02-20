use core::cell::RefCell;

pub type ControllerMutex<D> = embassy_sync::mutex::Mutex<crate::ControllerMutexType, D>;

pub trait ControllerKind<T: 'static> {
    type Type;
    type MutexType;
}

pub struct Holder<T: 'static> {
    pub r: RefCell<Option<embassy_sync::mutex::MutexGuard<'static, crate::ControllerMutexType, T>>>
}
impl<T: 'static> Holder<T> {
    const fn new() -> Self {
        Self {r: RefCell::new(None) }
    }
    fn set(&self, r: embassy_sync::mutex::MutexGuard<'static, crate::ControllerMutexType, T>) {
        self.r.borrow_mut().replace(r);
    }
    fn release(&self) {
        drop(self.r.borrow_mut().take());
    }
}

impl<T: 'static> Clone for Holder<T> {
    fn clone(&self) -> Self {
        Self {
            r: RefCell::new(None)
        }
    }
}

pub struct ControllerRef<T: 'static, M: embassy_sync::blocking_mutex::raw::RawMutex + 'static = crate::ControllerMutexType> {
    c: &'static embassy_sync::mutex::Mutex<M, T>,
    h: Holder<T>,
}
impl<T: 'static> ControllerKind<T> for ControllerRef<T> {
    type Type = T;
    type MutexType = T;
}

impl<T: 'static> ControllerRef<T> {
    pub fn new(c: &'static embassy_sync::mutex::Mutex<crate::ControllerMutexType, T>) -> Self {
        Self{c, h: Holder::new()}
    }

    #[allow(unused)]
    #[inline]
    pub async fn lock(&self) -> embassy_sync::mutex::MutexGuard<'static, crate::ControllerMutexType, T> {
        self.c.lock().await
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
        where F: FnOnce(&mut <ControllerRef<T> as ControllerKind<T>>::Type) -> Result<(),()>
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
        where F: FnOnce(&mut <ControllerRef<T> as ControllerKind<T>>::Type) -> Result<ROK, RKO>
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

impl<T: 'static> Clone for ControllerRef<T> {
    fn clone(&self) -> Self {
        ControllerRef {
            c: self.c,
            h: Holder::new(),
        }
    }
}