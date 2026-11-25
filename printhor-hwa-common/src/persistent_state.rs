//! A synchronization primitive for polling values from a task.
//! Basically, a copy of embassy_sync::Signal with a dirty hack to not lose the consumed value,
//! and that can be awaited by several tasks at once.
use crate as hwa;
use core::cell::RefCell;
use core::future::{Future, poll_fn};
use core::task::{Context, Poll};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::MultiWakerRegistration;
use hwa::AsyncRawMutex;

/// Tasks that may wait at once without waking each other: e.g. the motion queue availability is
/// awaited by the control task (a host) and the print job task. With more waiters it still works,
/// waking them all (each one registers again).
const MAX_WAITERS: usize = 3;

#[allow(unused)]
pub struct PersistentState<M, T>
where
    M: AsyncRawMutex,
    T: Send + Copy,
{
    state: Mutex<M, RefCell<State<T>>>,
}

struct State<T>
where
    T: Send + Copy,
{
    /// The value signaled. Kept until [PersistentState::reset]: every wait completes with it
    value: Option<T>,
    /// The tasks waiting for a value. A single waker made two waiters wake each other endlessly
    /// (a busy loop) until the value was signaled
    wakers: MultiWakerRegistration<MAX_WAITERS>,
}

#[allow(unused)]
impl<M, T> PersistentState<M, T>
where
    M: AsyncRawMutex,
    T: Send + Copy,
{
    /// Create a new `Signal`.
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(State {
                value: None,
                wakers: MultiWakerRegistration::new(),
            })),
        }
    }
}

#[allow(unused)]
impl<M, T> Default for PersistentState<M, T>
where
    M: AsyncRawMutex,
    T: Send + Copy,
{
    fn default() -> Self {
        Self::new()
    }
}

#[allow(unused)]
impl<M, T> PersistentState<M, T>
where
    M: AsyncRawMutex,
    T: Send + Copy,
{
    /// Mark this Signal as signaled, waking every waiting task.
    pub fn signal(&self, val: T) {
        self.state.lock(|cell| {
            let mut state = cell.borrow_mut();
            state.value = Some(val);
            state.wakers.wake();
        })
    }

    /// Remove the queued value in this `Config`, if any. The waiting tasks keep waiting.
    pub fn reset(&self) {
        self.state.lock(|cell| {
            cell.borrow_mut().value = None;
        });
    }

    fn poll_wait(&self, cx: &mut Context<'_>) -> Poll<T> {
        self.state.lock(|cell| {
            let mut state = cell.borrow_mut();
            match state.value {
                Some(r) => Poll::Ready(r),
                None => {
                    state.wakers.register(cx.waker());
                    Poll::Pending
                }
            }
        })
    }

    /// Future that completes when this Signal has been signaled.
    pub fn wait(&self) -> impl Future<Output = T> + '_ {
        poll_fn(move |cx| self.poll_wait(cx))
    }

    /// non-blocking method to check whether this signal has been signaled.
    pub fn signaled(&self) -> bool {
        self.state.lock(|cell| cell.borrow().value.is_some())
    }
}

#[cfg(test)]
mod tests {
    use crate as hwa;
    use core::future;
    use future::Future;
    use std::task::Poll;

    type MutexType = hwa::AsyncNoopMutexType;

    #[futures_test::test]
    async fn foundation_test() {
        let persistent_state: hwa::PersistentState<MutexType, bool> = hwa::PersistentState::new();
        assert_eq!(persistent_state.signaled(), false);
        persistent_state.signal(true);
        assert_eq!(persistent_state.signaled(), true);
        // The value stored is `true`
        assert_eq!(persistent_state.wait().await, true);
        // The value stored is still `true`
        assert_eq!(persistent_state.wait().await, true);
        persistent_state.reset();

        let persistent_state: hwa::PersistentState<MutexType, bool> = Default::default();

        future::poll_fn(|cx| {
            let mut pinned_fut = core::pin::pin!(persistent_state.wait());
            let p = pinned_fut.as_mut().poll(cx);
            assert!(p.is_pending());

            persistent_state.signal(true);

            let p = pinned_fut.as_mut().poll(cx);
            assert!(!p.is_pending());

            Poll::Ready(())
        })
        .await;
    }

    /// Two tasks waiting at once do not wake each other (that was a busy loop), and both are
    /// woken by the signal
    #[test]
    fn two_waiters() {
        use futures_test::task::new_count_waker;
        let persistent_state: hwa::PersistentState<MutexType, bool> = hwa::PersistentState::new();
        let (waker_a, count_a) = new_count_waker();
        let (waker_b, count_b) = new_count_waker();
        let mut cx_a = core::task::Context::from_waker(&waker_a);
        let mut cx_b = core::task::Context::from_waker(&waker_b);
        let mut fut_a = core::pin::pin!(persistent_state.wait());
        let mut fut_b = core::pin::pin!(persistent_state.wait());
        for _ in 0..3 {
            assert!(fut_a.as_mut().poll(&mut cx_a).is_pending());
            assert!(fut_b.as_mut().poll(&mut cx_b).is_pending());
        }
        assert_eq!((count_a.get(), count_b.get()), (0, 0));
        // A reset while waiting keeps them waiting
        persistent_state.reset();
        persistent_state.signal(true);
        assert_eq!((count_a.get(), count_b.get()), (1, 1));
        assert_eq!(fut_a.as_mut().poll(&mut cx_a), Poll::Ready(true));
        assert_eq!(fut_b.as_mut().poll(&mut cx_b), Poll::Ready(true));
    }
}
