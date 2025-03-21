//! This feature is being established
//! This module provides a task that notifies to the U(S)ART the acceptation or the completion of the
//! GCodes that aren't processed immediately, so processor can accept more
//! Some firmwares resolves this by allocating extra space in the queue, but that case issues because you can get blocked
use crate::{hwa, processing};

use embassy_time::{Duration, with_timeout};
use hwa::{CommChannel, DeferAction, DeferEvent};

#[derive(Clone, Copy, Default)]
struct SubscriptionCounting {
    #[cfg(feature = "with-motion")]
    num_homes: u8,
    #[cfg(feature = "with-motion")]
    num_linear: u8,
    #[cfg(feature = "with-motion")]
    num_rapid: u8,
    #[cfg(feature = "with-motion")]
    num_dwell: u8,
    #[cfg(feature = "with-motion")]
    num_set_position: u8,
    #[cfg(feature = "with-hot-end")]
    num_hotend: u8,
    #[cfg(feature = "with-hot-bed")]
    num_hotbed: u8,
}

struct Subscriptions {
    channel_counts: [SubscriptionCounting; CommChannel::count()],
    total_counts: i32,
}

impl Subscriptions {
    fn new() -> Self {
        Self {
            channel_counts: [SubscriptionCounting::default(); CommChannel::count()],
            total_counts: 0,
        }
    }

    fn update(
        &mut self,
        action: DeferAction,
        channel: CommChannel,
        increment: i8,
    ) -> Result<bool, ()> {
        if let Some(counts) = self.channel_counts.get_mut(CommChannel::index_of(channel)) {
            if increment == 1 {
                self.total_counts += 1;
            } else if increment == -1 {
                self.total_counts -= 1;
            }

            let counter = match action {
                #[cfg(feature = "with-motion")]
                DeferAction::Homing => &mut counts.num_homes,
                #[cfg(feature = "with-motion")]
                DeferAction::RapidMove => &mut counts.num_rapid,
                #[cfg(feature = "with-motion")]
                DeferAction::LinearMove => &mut counts.num_linear,
                #[cfg(feature = "with-motion")]
                DeferAction::Dwell => &mut counts.num_dwell,
                #[cfg(feature = "with-motion")]
                DeferAction::SetPosition => &mut counts.num_set_position,
                #[cfg(feature = "with-hot-end")]
                DeferAction::HotEndTemperature => &mut counts.num_hotend,
                #[cfg(feature = "with-hot-bed")]
                DeferAction::HotBedTemperature => &mut counts.num_hotbed,
            };
            let new_value = *counter as i8 + increment;
            if new_value < 0 {
                *counter = 0;
                Ok(false)
            } else {
                *counter = new_value as u8;
                Ok(true)
            }
        } else {
            Err(())
        }
    }
}

#[embassy_executor::task(pool_size = 1)]
pub async fn task_defer(
    processor: processing::GCodeProcessor,
    defer_channel: hwa::types::DeferChannel,
) {
    hwa::info!("[task_defer] started");

    let mut subscriptions = Subscriptions::new();

    loop {
        match with_timeout(Duration::from_secs(10), defer_channel.receive()).await {
            Err(_) => {
                #[cfg(feature = "trace-commands")]
                hwa::info!(
                    "[trace-commands] [task_defer] Timeout. Actual subscriptions count: {}",
                    subscriptions.total_counts
                );
                #[cfg(test)]
                if crate::tasks::task_integration::INTEGRATION_STATUS.signaled() {
                    hwa::info!("[task_defer] Ending gracefully");
                    return ();
                }
            }

            Ok(DeferEvent::AwaitRequested(action, channel, _order_num)) => {
                #[cfg(feature = "trace-commands")]
                hwa::info!("AwaitRequested {:?} {}", action, _order_num);
                let _ = subscriptions.update(action, channel, 1);
                #[cfg(feature = "trace-commands")]
                hwa::info!(
                    "[task_defer] Actual subscriptions count: {}",
                    subscriptions.total_counts
                );
            }

            Ok(DeferEvent::Completed(action, channel, _order_num)) => {
                #[cfg(feature = "trace-commands")]
                hwa::info!("AwaitCompleted {:?} {}", action, _order_num);
                if subscriptions.update(action, channel, -1).unwrap_or(false) {
                    cfg_if::cfg_if! {
                        if #[cfg(feature="trace-commands")] {
                            let msg = alloc::format!("ok; [trace-commands] {:?} completed @{:?}\n", action, channel);
                            hwa::info!("{}", msg.trim_end());
                            processor.write(channel, msg.as_str()).await;

                        }
                        else {
                            processor.write_ok(channel).await;
                        }
                    }
                }
                #[cfg(feature = "trace-commands")]
                hwa::info!(
                    "[task_defer] Actual subscriptions count: {}",
                    subscriptions.total_counts
                );
            }
        }
    }
}
