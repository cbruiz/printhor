//! Error and boundary paths of the step plan executor (the ISR side of the motion pipeline).
//!
//! The integration scenarios always install the actuator before any plan is pushed, so the
//! defensive paths (no actuator, inconsistent queue, full ring) are only reachable from here.
//! Every test works on its own [`SoftTimer`] instance: the global [`STEP_DRIVER`] is owned by
//! the integration scenarios running in the same test binary.
use super::*;
use crate::hwa::math::TVector;
use crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US;
use hwa::CoordSel;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::task::{Wake, Waker};

/// A waker that counts how many times it was woken.
struct CountingWaker(AtomicUsize);

impl Wake for CountingWaker {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}

fn counting_waker() -> (Arc<CountingWaker>, Waker) {
    let counter = Arc::new(CountingWaker(AtomicUsize::new(0)));
    let waker = Waker::from(counter.clone());
    (counter, waker)
}

fn wakes(counter: &Arc<CountingWaker>) -> usize {
    counter.0.load(Ordering::SeqCst)
}

/// A one-millisecond window emitting `steps` uniform steps on X.
fn plan_x(steps: u32) -> MultiTimer {
    let mut mt = MultiTimer::new();
    mt.set_displace_width(1000);
    mt.set_channel_uniform(
        CoordSel::X,
        Some(1000 / steps.max(1)),
        STEPPER_PLANNER_CLOCK_PERIOD_US,
    );
    let mut count: TVector<u32> = TVector::new();
    count.set_coord(CoordSel::X, Some(steps));
    mt.set_max_count(&count);
    mt
}

/// Polls a push of a 10-step X plan exactly once.
fn push_once(timer: &SoftTimer, order_num: u32, cx: &mut Context<'_>) -> Poll<()> {
    timer.poll_push(
        cx,
        order_num,
        #[cfg(feature = "with-motion-broadcast")]
        hwa::MotionDelta::new(),
        plan_x(10),
        CoordSel::X,
        CoordSel::X,
    )
}

fn with_executor<R>(timer: &SoftTimer, f: impl FnOnce(&mut StepPlanExecutor) -> R) -> R {
    critical_section::with(|cs| f(&mut timer.0.borrow_ref_mut(cs)))
}

/// Number of occupied queue slots. It must always match `num_queued`.
fn occupied_slots(ex: &StepPlanExecutor) -> usize {
    ex.queue.iter().filter(|slot| slot.is_some()).count()
}

fn x_pulses(pulses: &FlushResult) -> u32 {
    pulses.get_coord(CoordSel::X).unwrap_or(0)
}

#[test]
fn a_new_timer_is_idle_empty_and_flushes_at_once() {
    // Built at runtime (the global one is built at compile time and never shows up in coverage).
    let timer = SoftTimer::new();
    with_executor(&timer, |ex| {
        assert!(ex.state == StepPlannerExecutorState::Idle);
        assert_eq!((ex.head, ex.tail, ex.num_queued), (0, 0, 0));
        assert_eq!(ex.tick_count, 0);
        assert_eq!(occupied_slots(ex), 0);
        assert!(ex.actuator.is_none());
        assert!(ex.current_stepper_enable_flags.is_empty());
        assert!(ex.current_stepper_dir_fwd_flags.is_empty());
    });

    let (_, waker) = counting_waker();
    let mut cx = Context::from_waker(&waker);
    match timer.poll_flush(&mut cx) {
        Poll::Ready(pulses) => assert_eq!(x_pulses(&pulses), 0),
        Poll::Pending => panic!("an idle executor with an empty queue must flush at once"),
    }
}

#[test]
fn an_interrupt_with_an_empty_queue_only_wakes_the_producer() {
    let timer = SoftTimer::new();
    let (counter, waker) = counting_waker();
    with_executor(&timer, |ex| {
        ex.waker.register(&waker);
        ex.on_interrupt();
        assert!(ex.state == StepPlannerExecutorState::Idle);
        assert_eq!(ex.tick_count, 0, "no window may be advanced without a plan");
    });
    assert_eq!(wakes(&counter), 1, "the producer must be told that there is room");
}

#[test]
fn the_ring_fills_up_wraps_and_then_blocks_the_producer() {
    let timer = SoftTimer::new();
    let (counter, waker) = counting_waker();
    let mut cx = Context::from_waker(&waker);

    for order_num in 0..TIMER_QUEUE_SIZE as u32 {
        assert!(push_once(&timer, order_num, &mut cx).is_ready());
    }
    with_executor(&timer, |ex| {
        assert_eq!(ex.num_queued as usize, TIMER_QUEUE_SIZE);
        assert_eq!(occupied_slots(ex), TIMER_QUEUE_SIZE);
        assert_eq!(ex.tail, 0, "the tail must wrap around after the last slot");
        assert_eq!(ex.head, 0);
    });

    // One more push does not fit: it stays pending and nothing gets overwritten.
    assert!(push_once(&timer, 99, &mut cx).is_pending());
    with_executor(&timer, |ex| {
        assert_eq!(ex.num_queued as usize, TIMER_QUEUE_SIZE);
        assert_eq!(ex.queue[0].as_ref().map(|p| p._order_num), Some(0));
    });

    // Nothing is flushed while plans are queued.
    assert!(timer.poll_flush(&mut cx).is_pending());

    // While the ring is full, notify must not wake the blocked producer...
    with_executor(&timer, |ex| ex.notify());
    assert_eq!(wakes(&counter), 0);

    // ...but it must as soon as one slot is released.
    with_executor(&timer, |ex| {
        ex.queue[ex.head as usize] = None;
        ex.head += 1;
        ex.num_queued -= 1;
        ex.notify();
    });
    assert_eq!(wakes(&counter), 1);
    assert!(push_once(&timer, 99, &mut cx).is_ready());
    with_executor(&timer, |ex| {
        assert_eq!(ex.num_queued as usize, TIMER_QUEUE_SIZE);
        assert_eq!(ex.queue[0].as_ref().map(|p| p._order_num), Some(99));
        assert_eq!(ex.tail, 1);
    });
}

#[test]
fn no_window_starts_without_an_actuator() {
    let timer = SoftTimer::new();
    let (_, waker) = counting_waker();
    let mut cx = Context::from_waker(&waker);
    assert!(push_once(&timer, 1, &mut cx).is_ready());

    with_executor(&timer, |ex| {
        for _ in 0..3 {
            ex.on_interrupt();
            assert!(ex.state == StepPlannerExecutorState::Idle);
            assert_eq!(ex.tick_count, 0, "no step can be emitted without an actuator");
            assert_eq!(ex.head, 0, "the ring must not advance");
        }
    });
}

/// Regression: the plan used to be taken out of its slot while still counted in `num_queued`,
/// leaving an inconsistent queue that `flush()` would wait on forever.
#[test]
fn a_failed_consume_keeps_the_plan_queued() {
    let timer = SoftTimer::new();
    let (_, waker) = counting_waker();
    let mut cx = Context::from_waker(&waker);
    assert!(push_once(&timer, 1, &mut cx).is_ready());

    with_executor(&timer, |ex| {
        ex.on_interrupt();
        ex.on_interrupt();
        assert_eq!(ex.num_queued, 1);
        assert_eq!(occupied_slots(ex), 1);
        assert_eq!(ex.queue[ex.head as usize].as_ref().map(|p| p._order_num), Some(1));
    });
}

#[test]
fn an_inconsistent_queue_is_not_consumed() {
    let timer = SoftTimer::new();
    with_executor(&timer, |ex| {
        // Counted as queued, but the slot at head is empty.
        ex.num_queued = 1;
        assert!(!ex.try_consume());
        assert!(ex.state == StepPlannerExecutorState::Idle);
        assert_eq!((ex.head, ex.num_queued), (0, 1));

        // The interrupt survives it too (no panic, no tick).
        ex.on_interrupt();
        assert!(ex.state == StepPlannerExecutorState::Idle);
        assert_eq!(ex.tick_count, 0);
    });
}

#[test]
fn steps_cannot_be_applied_without_an_actuator() {
    let timer = SoftTimer::new();
    with_executor(&timer, |ex| {
        assert!(ex.apply(CoordSel::empty()), "an empty tick needs no actuator");
        assert!(!ex.apply(CoordSel::X));
        assert!(!ex.apply(CoordSel::all_axis()));
    });
}

#[test]
fn flush_hands_over_the_pulse_counters_and_clears_them() {
    let timer = SoftTimer::new();
    let (_, waker) = counting_waker();
    let mut cx = Context::from_waker(&waker);
    with_executor(&timer, |ex| {
        ex.pulses.set_coord(CoordSel::X, Some(7));
    });

    match timer.poll_flush(&mut cx) {
        Poll::Ready(pulses) => assert_eq!(x_pulses(&pulses), 7),
        Poll::Pending => panic!("an idle executor with an empty queue must flush at once"),
    }
    match timer.poll_flush(&mut cx) {
        Poll::Ready(pulses) => assert_eq!(x_pulses(&pulses), 0, "counters must be cleared"),
        Poll::Pending => panic!("an idle executor with an empty queue must flush at once"),
    }
}

#[test]
fn reset_forces_the_direction_to_be_applied_again() {
    let timer = SoftTimer::new();
    with_executor(&timer, |ex| ex.current_stepper_dir_fwd_flags = CoordSel::X);
    timer.reset();
    with_executor(&timer, |ex| {
        assert_eq!(ex.current_stepper_dir_fwd_flags, CoordSel::UNSET);
    });
}

/// Regression: reset() used to clear the direction flags twice and keep the enable flags, so
/// after park() (or M84) a plan with the same enable flags as the last one stepped disabled
/// drivers. The end-to-end check is integration scenario T25.
#[test]
fn reset_forces_the_drivers_to_be_enabled_again() {
    let timer = SoftTimer::new();
    with_executor(&timer, |ex| ex.current_stepper_enable_flags = CoordSel::X);
    timer.reset();
    with_executor(&timer, |ex| {
        assert_eq!(ex.current_stepper_enable_flags, CoordSel::UNSET);
    });
}
