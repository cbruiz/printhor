//! This feature is being established
//! This module provides a task that broadcasts near to real-time motion updates to dedicated break-boards

use crate::hwa;
use hwa::math;
use hwa::{Contract, HwiContract};
use math::{Real, TVector};

#[embassy_executor::task(pool_size = 1)]
pub async fn task_motion_broadcast(
    _motion_broadcast_channel: hwa::types::MotionBroadcastChannel,
    _motion_config: hwa::controllers::MotionConfig,
    _motion_sender: hwa::types::MotionSender,
) -> ! {
    // The motion interpolation period width
    const MICRO_SEGMENT_PERIOD_US: u32 =
        1_000_000 / Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY;
    const MAX_CMD_DELAY: u32 = MICRO_SEGMENT_PERIOD_US / 2;

    hwa::info!(
        "[task_motion_broadcast] started. Max expected delay: {} us",
        MAX_CMD_DELAY
    );

    let mut current_order = None;
    let mut lag: embassy_time::Instant = embassy_time::Instant::now();

    let receiver = _motion_broadcast_channel.receiver();

    loop {
        match receiver.receive().await {
            hwa::MotionBroadcastEvent::Reset => {
                if current_order.is_some() {
                    flush(&mut current_order, &TVector::zero(), &TVector::zero());
                }
                lag = embassy_time::Instant::now();
            }
            hwa::MotionBroadcastEvent::Delta(motion_event) => {
                let elapsed = match current_order {
                    Some(order_num) => {
                        if order_num != motion_event.order_num {
                            flush(
                                &mut current_order,
                                &motion_event.pos_steps,
                                &motion_event.pos_wu,
                            );
                            current_order = Some(motion_event.order_num);
                            lag = embassy_time::Instant::now();
                            0
                        } else {
                            lag.elapsed().as_micros()
                        }
                    }
                    None => {
                        current_order = Some(motion_event.order_num);
                        lag = embassy_time::Instant::now();
                        0
                    }
                };

                let mut mg = _motion_sender.lock().await;
                let mut changed = false;
                motion_event.pos_wu.foreach_values(|coord, val| {
                    changed |= mg.set_angle(coord, val);
                });

                if changed {
                    hwa::info!("[task_motion_broadcast] at: [t: {:?} s, t_ref: {:?} us] #[{:?}, {:?}] pos: [{:?}]",
                        Real::from_f32((elapsed as f32) / 1000000.0).rdp(4), motion_event.micro_segment_time.rdp(4),
                        motion_event.order_num, motion_event.micro_segment_id,
                        motion_event.pos_wu
                    );

                    if embassy_time::with_timeout(
                        embassy_time::Duration::from_micros((MAX_CMD_DELAY) as u64),
                        mg.apply(),
                    )
                    .await
                    .is_err()
                    {
                        hwa::warn!("[task_motion_broadcast] I2C Timeout");
                    }
                }
            }
        }
    }
}

fn flush(
    current_id: &mut Option<u32>,
    absolute_stp_pos: &TVector<i32>,
    absolute_wu_pos: &TVector<Real>,
) {
    hwa::info!(
        "[task_motion_broadcast] DONE order_num:{:?} pos: [ step: [{:?}], world [{:?}] {} ]",
        current_id,
        absolute_stp_pos,
        absolute_wu_pos,
        Contract::WORLD_UNIT_MAGNITUDE,
    );
    *current_id = None;
}
