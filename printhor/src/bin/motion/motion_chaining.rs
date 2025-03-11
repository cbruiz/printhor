use crate::hwa;
use hwa::{Contract,math, HwiContract};
use hwa::controllers::PlanEntry;
use hwa::controllers::motion_control::motion_ring_buffer::RingBuffer;

/// This method performs the cornering optimization algorithm on the motion [RingBuffer].
///
/// # Arguments
///
/// * `rb` - A mutable guard that locks the `hwa::ControllerMutexType` and provides access to the `RingBuffer`.
///
/// # Returns
///
/// * `Result<(), ()>` - Returns `Ok(())` if the cornering operation is successful, otherwise returns `Err(())`.
///
/// # Usage
///
/// This method is used to optimize the movement segments that are stored in the `RingBuffer`,
/// ensuring smooth transitions and efficient processing of corners or turns in the motion path.
/// It does this by adjusting the `speed_enter_mms` and `speed_exit_mms` fields of the segments
/// based on their projected speeds and constraints.
///
/// The optimization runs a flood fill algorithm to determine the best possible speed values
/// for entering and exiting each segment, aiming for minimal speed variations and smooth transitions.
///
/// The method is typically invoked during the planning phase of motion control, where multiple
/// segments are queued, and the goal is to optimize the entire motion path by smoothing out the
/// corners, hence enhancing overall motion performance.
pub fn perform_motion_chaining(
    mut rb: embassy_sync::mutex::MutexGuard<
        <Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >,
) -> Result<(), ()> {
    let mut left_offset = 2;
    let mut left_watermark = math::ZERO;
    let mut right_watermark = math::ZERO;
    // Assuming queued items are computed left to right:
    // First, locate the top left segment. That one with null projection
    // Also, set
    for index in 2..rb.used {
        match rb.planned_segment_from_tail(index) {
            Ok(prev_segment_candidate) => {
                if prev_segment_candidate.proj_next.is_defined_positive() {
                    left_offset = index;
                    left_watermark = prev_segment_candidate.speed_enter_su_s;
                } else {
                    break;
                }
            }
            Err(_) => break,
        }
    }

    #[allow(unused)]
    let from_offset = left_offset;
    #[allow(unused)]
    let to_offset = 1;

    hwa::debug!("Cornering algorithm START");
    let mut right_offset = 1;

    // Perform cornering optimization in a single pass with a flood fill algorithm
    loop {
        if left_offset <= right_offset {
            let mid_segment = rb.mut_planned_segment_from_tail(left_offset)?;

            mid_segment.speed_enter_su_s = left_watermark;
            mid_segment.speed_exit_su_s = right_watermark;
            break;
        } else {
            let (left_segment, right_segment) =
                match rb.entries_from_tail(left_offset, right_offset) {
                    (
                        Some(PlanEntry::PlannedMove(_s, _, _, _, _)),
                        Some(PlanEntry::PlannedMove(_t, _, _, _, _)),
                    ) => (_s, _t),
                    _ => return Err(()),
                };

            hwa::trace!("\tleft [{}] right[{}]", left_offset, right_offset);

            let left_max_inc = (left_segment.proj_next * left_segment.speed_target_su_s)
                .min(left_segment.speed_exit_constrained_su_s);

            let right_max_inc = (right_segment.proj_prev * right_segment.speed_target_su_s)
                .min(right_segment.speed_enter_constrained_su_s);

            let water_left = (left_watermark + left_max_inc).min(left_segment.speed_target_su_s);
            let water_right =
                (right_watermark + right_max_inc).min(right_segment.speed_target_su_s);

            if water_left <= water_right {
                // Flood at right
                hwa::trace!("flood to right  [{:?} {:?}]", water_left, water_right);
                left_segment.speed_enter_su_s = left_watermark;
                left_segment.speed_exit_su_s = water_left;
                right_segment.speed_enter_su_s = water_left;
                right_segment.speed_exit_su_s = right_watermark;
                left_watermark = water_left;
                left_offset -= 1;
            } else {
                // Flood at left
                hwa::trace!("flood to left [{:?} {:?}]", water_left, water_right);
                left_segment.speed_enter_su_s = left_watermark;
                left_segment.speed_exit_su_s = water_right;
                right_segment.speed_enter_su_s = water_right;
                right_segment.speed_exit_su_s = right_watermark;
                right_watermark = water_right;
                right_offset += 1;
            }
        }
    }
    #[cfg(feature = "native")]
    display_content(&rb, from_offset, to_offset)?;

    hwa::debug!("Cornering algorithm END");
    Ok(())
}

#[cfg(feature = "native")]
#[allow(unused)]
pub fn display_content(
    rb: &embassy_sync::mutex::MutexGuard<
        <hwa::types::MotionRingBufferMutexStrategy as hwa::AsyncMutexStrategy>::AsyncMutexType,
        hwa::controllers::motion_control::motion_ring_buffer::RingBuffer,
    >,
    left_offset: u8,
    right_offset: u8,
) -> Result<(), ()> {
    let mut stb = Vec::new();
    for i in 0..left_offset - right_offset + 1 {
        let s = rb.planned_segment_from_tail(left_offset - i).unwrap();
        stb.push(format!("[{:?},{:?}]", s.speed_enter_su_s, s.speed_exit_su_s))
    }
    hwa::debug!(": {:?}", stb.join(" "));
    Ok(())
}

#[cfg(test)]
mod motion_chaining_test {
    #[test]
    fn to_do() {
        
    }
}