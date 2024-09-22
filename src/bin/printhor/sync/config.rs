//! A synchronization primitive for polling values from a task.
//! Basically, a copy of embassy_sync::Signal with a dirty hack to not lose the consumed value
use core::cell::Cell;
use core::future::{poll_fn, Future};
use core::task::{Context, Poll, Waker};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::blocking_mutex::Mutex;

#[allow(unused)]
pub struct Config<M, T>
where
    M: RawMutex,
    T: Send + Copy,
{
    state: Mutex<M, Cell<State<T>>>,
}

#[allow(unused)]
#[derive(Clone)]
enum State<T>
where
    T: Send + Copy,
{
    None,
    Waiting(Waker),
    Signaled(T),
}

#[allow(unused)]
impl<M, T> Config<M, T>
where
    M: RawMutex,
    T: Send + Copy,
{
    /// Create a new `Signal`.
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(Cell::new(State::None)),
        }
    }
}

#[allow(unused)]
impl<M, T> Default for Config<M, T>
where
    M: RawMutex,
    T: Send + Copy,
{
    fn default() -> Self {
        Self::new()
    }
}

#[allow(unused)]
impl<M, T> Config<M, T>
where
    M: RawMutex,
    T: Send + Copy,
{
    /// Mark this Signal as signaled.
    pub fn signal(&self, val: T) {
        self.state.lock(|cell| {
            //info!("signal -> Replacing to {}", val);
            let state = cell.replace(State::Signaled(val));
            if let State::Waiting(waker) = state {
                waker.wake();
            }
        })
    }

    /// Remove the queued value in this `Config`, if any.
    pub fn reset(&self) {
        self.state.lock(|cell| cell.set(State::None));
    }

    fn poll_wait(&self, cx: &mut Context<'_>) -> Poll<T> {
        self.state.lock(|cell| {
            let state = cell.replace(State::None);

            match state {
                State::None => {
                    //info!("Poll State::None");
                    cell.set(State::Waiting(cx.waker().clone()));
                    Poll::Pending
                }
                State::Waiting(w) if w.will_wake(cx.waker()) => {
                    //info!("Poll State::Waiting.1");
                    cell.set(State::Waiting(w));
                    Poll::Pending
                }
                State::Waiting(w) => {
                    //info!("Poll State::Waiting.2");
                    cell.set(State::Waiting(cx.waker().clone()));
                    w.wake();
                    Poll::Pending
                }
                State::Signaled(r) => {
                    //info!("Poll State::Signaled cv=({})", r);
                    cell.set(State::Signaled(r));
                    Poll::Ready(r)
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
        self.state.lock(|cell| {
            let state = cell.replace(State::None);

            let res = matches!(state, State::Signaled(_));

            cell.set(state);

            res
        })
    }
}
