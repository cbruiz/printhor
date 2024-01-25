//! TODO: This feature is still incomplete and very experimental
// This module provides a task that notifies to the usart the acceptation or the completion of the
// gcodes that aren't processed immediately, so processor can accept more
// Some firmwares resolves this by allocating extra space in the queue, but that case issues because you can get blocked
use crate::hwa;
#[cfg(feature = "with-motion")]
use crate::hwa::controllers::{DeferEvent, DeferType};
#[allow(unused)]
#[allow(unreachable_patterns)]
#[embassy_executor::task(pool_size=1)]
pub async fn defer_task(
    mut processor: hwa::GCodeProcessor,
) -> ! {
    //let mut subscriber: state_manager::EventBusSubscriber<'static> = hwa::task_allocations::init_defer_subscriber(processor.event_bus.clone()).await;
    //subscriber.wait_until(EventStatus::containing(EventFlags::SYS_READY)).await;
    hwa::debug!("defer_task started");
    let mut num_homes = 0u8;
    let mut num_linear = 0u8;
    let mut num_rapid = 0u8;
    let mut num_dwell = 0u8;

    loop {
        match processor.motion_planner.defer_channel.receive().await {
            DeferEvent::Homing(DeferType::AwaitRequested) => {
                num_homes += 1;
                //processor.write("D. G28 (AwaitRequested @defer_task)\n").await;
            }
            DeferEvent::Homing(DeferType::Completed) => {
                if num_homes > 0 {
                    num_homes -= 1;
                    // FIXME: Control when a confirmation is needed from specific channel
                    processor.write("ok; G28 completed (@defer_task)\n").await;
                }
            }
            DeferEvent::Dwell(DeferType::AwaitRequested) => {
                num_dwell += 1;
                //processor.write("D. G28 (AwaitRequested @defer_task)\n").await;
            }
            DeferEvent::Dwell(DeferType::Completed) => {
                if num_dwell > 0 {
                    num_dwell -= 1;
                    processor.write("ok; G4 completed (@defer_task)\n").await;
                }
            }
            DeferEvent::LinearMove(DeferType::AwaitRequested) => {
                num_linear += 1;
                //processor.write("D. G1 (AwaitRequested @defer_task)\n").await;
            }
            DeferEvent::LinearMove(DeferType::Completed) => {
                if num_linear > 0 {
                    num_linear -= 1;
                    processor.write("echo: G1 completed (@defer_task)\n").await;
                }
            }
            DeferEvent::RapidMove(DeferType::AwaitRequested) => {
                num_rapid += 1;
            }
            DeferEvent::RapidMove(DeferType::Completed) => {
                if num_rapid > 0 {
                    num_rapid -= 1;
                    processor.write("echo: G0 completed (@defer_task)\n").await;
                }
            }
            DeferEvent::HotendTemperature(DeferType::AwaitRequested) => {
                //processor.write("D. MXXX (AwaitRequested @defer_task)\n").await;
            }
            DeferEvent::HotbedTemperature(DeferType::Completed) => {
                //processor.write("O. MXXX (Completed @defer_task)\n").await;
            }
            _ => {
                todo!("not yet impl")
            }
        }
    }
}