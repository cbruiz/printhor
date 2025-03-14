//! The motion chaining algorithm
use crate::hwa;
#[allow(unused)]
use hwa::{
    Contract, HwiContract,
    controllers::{PlanEntry, motion_control::motion_ring_buffer::RingBuffer},
    math,
};

/// Compute the maximum speed that can be gained between the current segment and the next one.
/// 1. `H1` Traveling at maximum acceleration (3rd order acceleration: constant jerk)
/// 2. Compute `t` as the time needed to travel target displacement given `H1`.
/// 3. Final speed (`v_max`) is the velocity reached at `t`.
/// <p>\[
/// \begin{align*} \textrm{Solve}\; s_{i1}(t, 0) & \le q_{1} \;\textrm{given}: &\\
/// & s_{i1}(t, \delta)_{|t>\delta} = \frac{j_{max} (t-\delta)^3}{6} + v_{0} (t-\delta)\\
/// & \delta = 0 \land t_{max} > 0 \land j_{max} > 0 \land q_1 = s_{i1}(t_{max},\delta)\\
/// \\
/// \hline
/// \\
/// & t_{max} = \frac{\sqrt[3]{3j_{max}^2 q_1 + \sqrt{9j_{max}^4q_{1}^2+8j_{max}^3v_o^3}}}{j_{max}} - \frac{2v_o}{\sqrt[3]{3j_{max}^2 q_1 + \sqrt{9j_{max}^4q_1^2+8j_{max}^3v_o^3}}}\\
/// & v_{1} = v_0 + \frac{j_{max}t_{max}^2}{2}\\
/// \end{align*}
/// \]</p>
/// ...
pub fn compute_gain(
    is_initial: bool,
    _num_order: u32,
    curr_segment: &mut hwa::controllers::Segment,
) {
    if is_initial {
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[motion_chaining] Computing initial gain for segment {:?}",
            _num_order
        );
    } else {
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[motion_chaining] Re-computing gain for segment @[head -{:?}]",
            _num_order
        );
    }
    let q_1 = curr_segment.displacement_su;
    let v_0 = curr_segment.speed_enter_su_s;

    let accel = curr_segment.constraints.j_max;
    let accel_sq = accel * accel;
    let t1 = ((math::THREE * accel_sq * q_1)
        + ((math::NINE * accel_sq * accel_sq * q_1 * q_1)
            + (math::EIGHT * accel_sq * accel * v_0 * v_0 * v_0))
            .sqrt()
            .or_else(|| Some(math::ZERO))
            .unwrap())
    .cbrt();
    let t_max = (t1 / accel) - ((math::TWO * v_0) / t1);
    let v_max = v_0 + ((accel * t_max * t_max) * math::HALF);
    cfg_if::cfg_if! {
        if #[cfg(feature = "debug-motion-planning")] {
            hwa::info!("--");
            hwa::info!("q_1: {:?}", q_1);
            hwa::info!("v_0: {:?}", v_0);
            hwa::info!("accel: {:?}", accel);
            hwa::info!("t_max: {:?}", t_max);
            hwa::info!("v_max: {:?}", v_max);
            let smax = v_0 * t_max + ((accel * t_max * t_max * t_max ) * math::SIXTH);
            hwa::info!("s_max(check): {:?}", smax);
            let vmax = v_0 + ((accel * t_max * t_max ) * math::HALF);
            hwa::info!("v_max(check): {:?}", vmax);
        }
    }
    curr_segment.speed_exit_constrained_su_s = v_max.min(curr_segment.constraints.v_max);
    curr_segment.speed_enter_constrained_su_s = curr_segment.speed_exit_su_s - v_max;
    if is_initial {
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[motion_chaining] Initial gain for segment {:?}: {:?} , {:?}",
            _num_order,
            curr_segment.speed_enter_constrained_su_s,
            curr_segment.speed_exit_constrained_su_s,
        );
    } else {
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[motion_chaining] Gain for segment @[head-{:?}]: {:?} , {:?}",
            _num_order,
            curr_segment.speed_enter_constrained_su_s,
            curr_segment.speed_exit_constrained_su_s,
        );
    }
}

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
    // The current left segment offset (from tail)
    let mut offset_left = 2;
    // The current right segment offset (from tail)
    let mut offset_right = 1;
    // The amount of level increased at left
    let mut water_level_left = math::ZERO;
    // The amount of level increased at right
    let mut water_level_right = math::ZERO;
    // Assuming queued items are computed left to right:
    // First, locate the top left segment. That one with null projection
    for index in 1..rb.used {
        match rb.planned_segment_from_tail(index) {
            Ok(segment) => {
                if index == 1 {
                    water_level_right = segment.speed_exit_su_s;
                } else {
                    if segment.proj_next.is_defined_positive() {
                        offset_left = index;
                        water_level_left = segment.speed_enter_su_s;
                    } else {
                        break;
                    }
                }
            }
            Err(_) => break,
        }
    }

    #[allow(unused)]
    let from_offset = offset_left;
    #[allow(unused)]
    let to_offset = 1;

    //if from_offset < 19 {
    //    return Ok(())
    //}

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!("[motion_chaining] #########################");
    #[cfg(feature = "debug-motion-planning")]
    hwa::info!("[motion_chaining] Cornering algorithm START");

    #[cfg(all(test, feature = "debug-motion-planning"))]
    display_content(&rb, from_offset, to_offset)?;

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!(
        "[motion_chaining] Applying from tail -{:?} to -{:?}",
        offset_left,
        offset_right
    );

    // Perform cornering optimization in a single pass with a flood fill algorithm
    loop {
        #[cfg(feature = "debug-motion-planning")]
        hwa::debug!(
            "[motion_chaining] # Segment indexes (from tail): left: -{} right: -{}",
            offset_left,
            offset_right
        );
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[motion_chaining] # Water level: {:?} right: {:?}",
            water_level_left,
            water_level_right
        );
        if offset_left <= offset_right {
            let mid_segment = rb.mut_planned_segment_from_tail(offset_left)?;

            mid_segment.speed_enter_su_s = water_level_left;
            mid_segment.speed_exit_su_s = water_level_right;
            break;
        } else {
            let (left_segment, right_segment) =
                match rb.entries_from_tail(offset_left, offset_right) {
                    (
                        Some(PlanEntry::PlannedMove(_s, _, _, _, _)),
                        Some(PlanEntry::PlannedMove(_t, _, _, _, _)),
                    ) => (_s, _t),
                    _ => return Err(()),
                };

            let level_left = left_segment.speed_enter_su_s;
            let level_right = right_segment.speed_exit_su_s;
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!(
                "[motion_chaining] # with current level [{:?} {:?}]",
                level_left,
                level_right
            );

            let diff_at_left = water_level_left - level_left;
            let diff_at_right = water_level_right - level_right;

            #[cfg(feature = "debug-motion-planning")]
            hwa::info!(
                "[motion_chaining] # and diff [{:?} {:?}]",
                diff_at_left,
                diff_at_right
            );

            if !diff_at_left.is_negligible() {
                left_segment.speed_enter_su_s = water_level_left;
                left_segment.speed_exit_su_s = water_level_left;
                compute_gain(false, offset_left as u32, left_segment);
            }
            if !diff_at_right.is_negligible() {
                right_segment.speed_enter_su_s = water_level_right;
                right_segment.speed_exit_su_s = water_level_right;
                compute_gain(false, offset_right as u32, right_segment);
            }

            let gain_at_left = left_segment.speed_exit_constrained_su_s;

            let gain_at_right =
                right_segment.speed_exit_su_s - right_segment.speed_enter_constrained_su_s;

            #[cfg(feature = "debug-motion-planning")]
            hwa::info!(
                "[motion_chaining] # and possible gain [{:?} {:?}]",
                gain_at_left,
                gain_at_right
            );

            if gain_at_left <= gain_at_right {
                // Flood to right
                #[cfg(feature = "debug-motion-planning")]
                hwa::info!("[motion_chaining] # flood [ .. |-> .. ])");
                left_segment.speed_enter_su_s = water_level_left;
                left_segment.speed_exit_su_s = gain_at_left;
                water_level_left = left_segment.speed_exit_su_s;
                #[cfg(all(test, feature = "debug-motion-planning"))]
                display_content(&rb, from_offset, to_offset)?;
                offset_left -= 1;
            } else {
                // Flood to left
                #[cfg(feature = "debug-motion-planning")]
                hwa::info!("[motion_chaining] # flood [ .. <-| .. ])");
                right_segment.speed_exit_su_s = water_level_right;
                right_segment.speed_enter_su_s = gain_at_right;
                water_level_right = right_segment.speed_enter_su_s;
                #[cfg(all(test, feature = "debug-motion-planning"))]
                display_content(&rb, from_offset, to_offset)?;
                offset_right += 1;
            }
        }
    }
    #[cfg(all(test, feature = "debug-motion-planning"))]
    display_content(&rb, from_offset, to_offset)?;

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!("[motion_chaining] Cornering algorithm END");
    Ok(())
}

#[cfg(all(test, feature = "debug-motion-planning"))]
pub fn display_content(
    rb: &embassy_sync::mutex::MutexGuard<
        <hwa::types::MotionRingBufferMutexStrategy as hwa::AsyncMutexStrategy>::AsyncMutexType,
        RingBuffer,
    >,
    left_offset: u8,
    right_offset: u8,
) -> Result<(), ()> {
    let mut stb = Vec::new();
    for i in 0..left_offset - right_offset + 1 {
        let s = rb.planned_segment_from_tail(left_offset - i)?;
        stb.push(format!(
            "[{:?},{:?}]",
            s.speed_enter_su_s, s.speed_exit_su_s
        ))
    }
    hwa::info!(": {:?}", stb.join(" "));
    Ok(())
}

#[cfg(test)]
mod motion_chaining_test {
    #[test]
    fn to_do() {}
}
