//! This feature is being established
//! This module provides a task that notifies to the U(S)ART the acceptation or the completion of the
//! GCodes that aren't processed immediately, so processor can accept more
//! Some firmwares resolves this by allocating extra space in the queue, but that case issues because you can get blocked
use embassy_time::Duration;
use crate::hwa::{CommChannel, DeferEvent, DeferAction};

#[derive(Clone, Copy, Default)]
struct SubscriptionCountings {
    num_homes: u8,
    num_linear: u8,
    num_rapid: u8,
    num_dwell: u8,
    #[cfg(feature = "with-hot-end")]
    num_hotend: u8,
    #[cfg(feature = "with-hot-bed")]
    num_hotbed: u8,
}

struct Subscriptions {
    channel_counts:  [SubscriptionCountings; CommChannel::count() - 1],
}

impl Subscriptions {
    fn new() -> Self {
        Self {
            channel_counts: [SubscriptionCountings::default(); CommChannel::count() - 1]
        }
    }

    fn update(&mut self, action: DeferAction, channel: CommChannel, increment: i8) -> bool {
        if let Some(counts) = self.channel_counts.get_mut(CommChannel::index_of(channel)) {
            let counter = match action {
                DeferAction::Homing => {
                    &mut counts.num_homes
                }
                DeferAction::RapidMove => {
                    &mut counts.num_rapid
                }
                DeferAction::LinearMove => {
                    &mut counts.num_linear
                }
                DeferAction::Dwell => {
                    &mut counts.num_dwell
                }
                #[cfg(feature = "with-hot-end")]
                DeferAction::HotendTemperature => {
                    &mut counts.num_hotend
                }
                #[cfg(feature = "with-hot-bed")]
                DeferAction::HotbedTemperature => {
                    &mut counts.num_hotbed
                }
            };
            let new_value = *counter as i8 + increment;
            if new_value < 0 {
                *counter = 0;
                return false;
            }
            else {
                *counter = new_value as u8;
                return true;
            }
        }
        false
    }
}

#[embassy_executor::task(pool_size=1)]
pub async fn task_defer(processor: crate::hwa::GCodeProcessor) -> ! {
    crate::hwa::debug!("task_defer started");

    let mut subscriptions = Subscriptions::new();

    loop {
        match embassy_time::with_timeout(Duration::from_secs(10), processor.motion_planner.defer_channel.receive()).await {
            Err(_) => {
                crate::hwa::trace!("task_defer timeout");
            }

            Ok(DeferEvent::AwaitRequested(action, channel)) => {
                crate::hwa::debug!("AwaitRequested {:?}", action);
                let _ = subscriptions.update(action, channel, 1);
            }
            Ok(DeferEvent::Completed(action, channel)) => {
                crate::hwa::debug!("AwaitCompleted {:?}", action);
                if subscriptions.update(action, channel, -1) {
                    cfg_if::cfg_if! {
                        if #[cfg(feature="trace-commands")] {
                            let msg = alloc::format!("ok; {:?} completed @{:?}\n", action, channel);
                            processor.write(channel, msg.as_str()).await;
                        }
                        else {
                            processor.write_ok(channel).await;
                        }
                    }

                }
            }
        }
    }
}