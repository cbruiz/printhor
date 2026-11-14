//! Tests for the event bus channel controller and the fault-tolerant subscriber waits.
use super::*;
use core::future::Future;
use core::pin::{Pin, pin};
use core::task::{Context, Poll, Waker};
use hwa::{EventFlags, EventStatus};

type PubSubMutexType = hwa::AsyncCsMutexType;
type ResourceType = hwa::EventBusChannelController<PubSubMutexType>;
type MutexStrategyType = hwa::AsyncHoldableStrategy<hwa::AsyncCsMutexType, ResourceType>;
type Bus = hwa::GenericEventBus<MutexStrategyType, PubSubMutexType>;

/// Creates an independent event bus. Every expansion owns its own statics, so each test must
/// expand it exactly once.
macro_rules! new_bus {
    ($name:literal) => {{
        #[allow(unused_imports)]
        use crate as printhor_hwa_utils;
        let bus: Bus = hwa::GenericEventBus::new(hwa::make_static_async_controller!(
            $name,
            MutexStrategyType,
            hwa::EventBusChannelController::new(hwa::make_static_ref!(
                $name,
                hwa::EventBusPubSubType<PubSubMutexType>,
                hwa::EventBusPubSubType::new(),
            )),
        ));
        bus
    }};
}

/// Polls a future once with a no-op waker.
fn poll_once<F: Future>(f: Pin<&mut F>) -> Poll<F::Output> {
    f.poll(&mut Context::from_waker(Waker::noop()))
}

#[test]
fn controller_publishes_only_masked_changes() {
    #[allow(unused_imports)]
    use crate as printhor_hwa_utils;
    let channel = hwa::make_static_ref!(
        "CoverageControllerChannel",
        hwa::EventBusPubSubType<PubSubMutexType>,
        hwa::EventBusPubSubType::new(),
    );
    let mut controller = EventBusChannelController::new(channel);
    let mut sub = controller.subscriber().unwrap();
    assert_eq!(controller.get_status(), EventFlags::empty());

    controller.publish_event(EventStatus::containing(EventFlags::MOVING));
    assert_eq!(controller.get_status(), EventFlags::MOVING);
    assert_eq!(sub.try_next_message_pure(), Some(EventFlags::MOVING));

    // Re-publishing the current state is not a change: nothing is broadcast
    controller.publish_event(EventStatus::containing(EventFlags::MOVING));
    assert_eq!(sub.try_next_message_pure(), None);

    // Only the bits inside the mask are applied
    controller.publish_event(EventStatus {
        flags: EventFlags::HOMING | EventFlags::SYS_ALARM,
        mask: EventFlags::HOMING,
    });
    assert_eq!(controller.get_status(), EventFlags::MOVING | EventFlags::HOMING);
    assert_eq!(
        sub.try_next_message_pure(),
        Some(EventFlags::MOVING | EventFlags::HOMING)
    );

    // An empty mask never changes anything
    controller.publish_event(EventStatus {
        flags: EventFlags::all(),
        mask: EventFlags::empty(),
    });
    assert_eq!(sub.try_next_message_pure(), None);

    // Clearing a flag
    controller.publish_event(EventStatus::not_containing(EventFlags::MOVING));
    assert_eq!(controller.get_status(), EventFlags::HOMING);
    assert_eq!(sub.try_next_message_pure(), Some(EventFlags::HOMING));
}

#[futures_test::test]
async fn subscriber_status_tracks_the_latest_published_state() {
    let bus = new_bus!("CoverageBusStatus");
    let mut sub = bus.subscriber().await;
    assert_eq!(sub.get_status().await, EventFlags::empty());

    // Several changes before the subscriber looks: it reports the last one
    bus.publish_event(EventStatus::containing(EventFlags::ATX_ON)).await;
    bus.publish_event(EventStatus::containing(EventFlags::SYS_READY)).await;
    assert_eq!(
        sub.get_status().await,
        EventFlags::ATX_ON | EventFlags::SYS_READY
    );
    // And keeps reporting it while nothing changes
    assert_eq!(
        sub.get_status().await,
        EventFlags::ATX_ON | EventFlags::SYS_READY
    );
    assert!(bus.has_flags(EventFlags::ATX_ON | EventFlags::SYS_READY).await);
    assert!(!bus.has_flags(EventFlags::ATX_ON | EventFlags::HOMING).await);
}

#[futures_test::test]
async fn waits_complete_when_the_condition_holds() {
    let bus = new_bus!("CoverageBusWaits");
    bus.publish_event(EventStatus::containing(EventFlags::SYS_BOOTING)).await;
    let mut sub = bus.subscriber().await;

    // No expectation at all: immediately satisfied
    assert_eq!(sub.ft_wait_for(EventStatus::new()).await, Ok(()));
    // Already satisfied conditions
    assert_eq!(sub.ft_wait_until(EventFlags::SYS_BOOTING).await, Ok(()));
    assert_eq!(sub.ft_wait_until_reset(EventFlags::SYS_READY).await, Ok(()));
    assert_eq!(sub.ft_wait_while(EventFlags::HOMING).await, Ok(()));
    assert_eq!(
        sub.ft_wait_for(
            EventStatus::containing(EventFlags::SYS_BOOTING).and_not_containing(EventFlags::SYS_READY)
        )
        .await,
        Ok(())
    );
}

#[futures_test::test]
async fn wait_resumes_after_a_later_publication() {
    let bus = new_bus!("CoverageBusResume");
    bus.publish_event(EventStatus::containing(EventFlags::MOVING)).await;
    let mut sub = bus.subscriber().await;

    {
        let mut wait = pin!(sub.ft_wait_while(EventFlags::MOVING));
        assert_eq!(poll_once(wait.as_mut()), Poll::Pending);
        // An unrelated change does not release the wait
        bus.publish_event(EventStatus::containing(EventFlags::HOMING)).await;
        assert_eq!(poll_once(wait.as_mut()), Poll::Pending);
        bus.publish_event(EventStatus::not_containing(EventFlags::MOVING)).await;
        assert_eq!(poll_once(wait.as_mut()), Poll::Ready(Ok(())));
    }
    {
        let mut wait = pin!(sub.ft_wait_until(EventFlags::JOB_COMPLETED));
        assert_eq!(poll_once(wait.as_mut()), Poll::Pending);
        bus.publish_event(EventStatus::containing(EventFlags::JOB_COMPLETED)).await;
        assert_eq!(poll_once(wait.as_mut()), Poll::Ready(Ok(())));
    }
}

#[futures_test::test]
async fn system_alarm_aborts_waits_unless_it_is_awaited() {
    let bus = new_bus!("CoverageBusAlarm");
    let mut sub = bus.subscriber().await;
    {
        let mut wait = pin!(sub.ft_wait_until(EventFlags::SYS_READY));
        assert_eq!(poll_once(wait.as_mut()), Poll::Pending);
        bus.publish_event(EventStatus::containing(EventFlags::SYS_ALARM)).await;
        assert_eq!(poll_once(wait.as_mut()), Poll::Ready(Err(())));
    }
    // While the alarm is active every other wait fails immediately...
    assert_eq!(sub.ft_wait_until_reset(EventFlags::MOVING).await, Err(()));
    // ...but waiting for the alarm itself succeeds
    assert_eq!(sub.ft_wait_until(EventFlags::SYS_ALARM).await, Ok(()));
    bus.publish_event(EventStatus::not_containing(EventFlags::SYS_ALARM)).await;
    assert_eq!(sub.ft_wait_until_reset(EventFlags::SYS_ALARM).await, Ok(()));
    assert_eq!(sub.ft_wait_until_reset(EventFlags::MOVING).await, Ok(()));
}

/// `ft_wait_for` returns early when `what.flags` is empty, but an `EventStatus` built only from
/// negative conditions (`EventStatus::new().and_not_containing(X)`, or `not_containing` of every
/// flag) has empty `flags` and a non-empty `mask`: the wait completes immediately even though
/// the awaited flags are still set. The early return should check the mask instead.
#[futures_test::test]
async fn negative_only_conditions_wait_for_the_flags_to_clear() {
    let bus = new_bus!("CoverageBusNegative");
    bus.publish_event(EventStatus::containing(EventFlags::MOVING)).await;
    let mut sub = bus.subscriber().await;

    let mut wait = pin!(sub.ft_wait_for(EventStatus::new().and_not_containing(EventFlags::MOVING)));
    assert_eq!(poll_once(wait.as_mut()), Poll::Pending, "MOVING is still set");
    bus.publish_event(EventStatus::not_containing(EventFlags::MOVING)).await;
    assert_eq!(poll_once(wait.as_mut()), Poll::Ready(Ok(())));
}
