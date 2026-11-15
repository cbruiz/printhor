//! Interpolator Step algorithm.
//!
//! The IS algorithm works the following way:
//! - Try to retrieve an execution-ready motion segment from the motion queue
//!   - If no motion segment is present within [`STEPPER_INACTIVITY_TIMEOUT`] seconds, disable all steppers
//!   - If motion segment is execution-ready, dequeue it and then:
//!     - Enable all steppers
//!     - Evaluate the motion profile displacement at [`Contract::STEP_PLANNER_CLOCK_FREQUENCY`]
//!     - Compute number of steps to do in each axis (independently)
//!     - Compute pulse rate across each axis (independently) and construct an iterator leveraging [`MultiTimer`](hwa::controllers::motion_control::MultiTimer)
//!     - Consume a micro-segment until iterator is exhausted
//!
//! TODO: This is a work still in progress

use crate::{hwa, motion};

use embassy_time::Duration;
use hwa::controllers::{ConsumedPlan, ExecPlan};
use hwa::controllers::motion_control::STEP_DRIVER;
use hwa::controllers::motion_control::motion_metrics as metrics;
use hwa::math;
use motion::MicroSegmentInterpolator;
#[allow(unused)]
use motion::MotionProfile;
use motion::HybridMotionProfile;
use motion::SegmentSampler;

use hwa::math::{CoordSel, Real, TVector};
#[allow(unused)]
use hwa::{Contract, HwiContract};
use hwa::{EventFlags, EventStatus};

///
/// This constant defines the duration of inactivity after which the stepper motors
/// are automatically disabled to save power and prevent overheating.
///
/// # Value
/// The timeout is set to [STEPPER_INACTIVITY_TIMEOUT] seconds, represented using `embassy_time::Duration`.
///
/// # Usage
/// This constant is used in the stepper motor control task to determine the period
/// of inactivity before disabling the stepper motors. When no motion segment is
/// retrieved within this duration, they are turned off automatically.
///
/// # Effect on Task Behavior
/// In the `task_stepper` routine, this timeout plays a crucial role. If no motion
/// segment is available within this [STEPPER_INACTIVITY_TIMEOUT] seconds window, the routine will automatically
/// disable the stepper motors. This means that:
/// - **Energy Savings**: Steppers are not left running indefinitely when they are not needed, saving energy.
/// - **Overheating Prevention**: Prevents the steppers from overheating due to continuous power being applied without motion.
/// - **Safety**: Ensures that the system does not remain in an unintended state.
///
/// # Example
/// ```
/// if embassy_time::with_timeout(STEPPER_INACTIVITY_TIMEOUT, /* some async operation */).await.is_err() {
///     // Disable steppers here
/// }
/// ```
#[cfg(not(test))]
const STEPPER_INACTIVITY_TIMEOUT: Duration = Duration::from_secs(10);
/// Shorter in the tests (native simulator), so that the park scenarios do not wait 10 s
#[cfg(test)]
const STEPPER_INACTIVITY_TIMEOUT: Duration = Duration::from_secs(3);

/// Whether something consumes the step queue (the stepper ticker, or the motion broadcast). When
/// not, the moves are planned and tracked, but executed virtually (see `task_stepper`)
const HAS_STEP_ACTUATOR: bool =
    cfg!(any(feature = "with-motion-stepper", feature = "with-motion-broadcast"));

// This constant defines the period for generating the stepper planner micro segments.
//
// # Value
// The period is defined in microseconds (`us`) and is determined by the `STEPPER_PLANNER_MICROSEGMENT_FREQUENCY`
//
// # Usage
// This constant is pivotal in the stepper task to dictate the intervals at which micro-segments are evaluated.
// It is a core part of the stepping pulse generation process.
//
// # Effect on Task Behavior
// The period influences the following aspects of the task:
// - **Precision**: Shorter periods mean higher frequency evaluation, leading to more precise control over motion segments and thus finer control of the stepping motors.
// - **Latency**: Directly affects the responsiveness of the stepper control; shorter periods can lead to more immediate adjustments.
// - **Performance**: A very short period increases the computational load on the CPU as the micro-segments need to be processed more frequently.
//
// In practical terms, this means:
// - If `STEPPER_PLANNER_MICROSEGMENT_PERIOD_US` is too large, motion might be less smooth and precise, with potential jitters.
// - If it's too small, the system might become CPU-bound and could miss steps or react slower to changes in motion segments.
//
// # Example
// ```rust
// let micro_segment_period_secs: Real = Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 6);
// // Used in task_stepper to control stepper motor behavior
// ```
pub const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
    1_000_000 / Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY;

///
/// This constant defines the period in microseconds for generating the clock ticks
/// used by the stepper planner.
///
/// # Value
/// The period is defined in microseconds (`us`) and is determined by the
/// `STEPPER_PLANNER_CLOCK_FREQUENCY` value.
///
/// # Usage
/// This constant is essential in the `task_stepper` routine as it dictates the clock
/// frequency used for timing the stepper motor pulses.
///
/// # Effect on Task Behavior
/// The clock period has a significant impact on the execution of the `task_stepper` routine:
/// - **Timing Precision**: A shorter clock period implies higher frequency, thus
///   providing more precise timing control over the stepper movements.
/// - **System Load**: Very short clock periods can increase the computational load
///   on the CPU, as the task needs to handle more clock interrupts per second.
/// - **Motion Smoothness**: Appropriate clock period settings can help achieve smoother
///   stepper motor movements, reducing the risk of jerky or uneven motion.
///
/// In more practical terms:
/// - If `STEPPER_PLANNER_CLOCK_PERIOD_US` is set too large, the control over the stepping
///   process becomes coarser, potentially resulting in less smooth motor operation.
/// - Conversely, if it is too small, the CPU may become overwhelmed with clock interrupts,
///   impacting overall system performance.
///
/// # Example
/// ```rust
/// let clock_period = STEPPER_PLANNER_CLOCK_PERIOD_US;
/// // Use this value in timing calculations for stepper control
/// ```
pub const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / Contract::STEP_PLANNER_CLOCK_FREQUENCY;

/// This asynchronous task manages the steps of the stepper motors
/// based on motion segments from the motion queue.
///
/// # Arguments
///
/// * `event_bus` - Reference to the event bus for handling events.
/// * `motion_planner` - Reference to the motion planner controller.
/// * `_watchdog` - Reference to the watchdog controller.
///
/// # Behavior
///
/// - Waits for the `ATX_ON` signal to proceed.
/// - Disables steppers upon inactivity or `SYS_ALARM`.
/// - Retrieves and processes motion segments to drive the stepper motors.
/// - Handles homing procedure if no segments are available.
/// - This task feeds watchdog to ensure no reset happen due high CPU starvation when feed rate is very high
///
/// This function runs indefinitely and controls the stepper motors
/// according to the motion planning profile.
#[embassy_executor::task]
pub async fn task_stepper(
    event_bus: hwa::types::EventBus,
    motion_planner: hwa::controllers::MotionPlanner,
    _watchdog: hwa::types::WatchDogController,
) {
    let mut steppers_off = true;

    let micro_segment_period_secs: Real =
        Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 6).rdp(6);

    let _sampling_time: Real = Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US as i64, 6).rdp(6);

    motion_planner.start(&event_bus).await;

    if !HAS_STEP_ACTUATOR {
        hwa::warn!(
            "[task_stepper] No step actuator (with-motion-stepper/with-motion-broadcast): moves are executed virtually"
        );
    }

    STEP_DRIVER.setup(
        motion_planner
            .motion_driver()
            .lock()
            .await
            .step_actuator()
            .clone(),
    );
    // The tool outputs switched by the step executor (see `hwa::controllers::ToolActions`)
    STEP_DRIVER.setup_tools(tool_outputs(&motion_planner).await);

    #[allow(unused_mut)]
    let mut _s = event_bus.subscriber().await;

    hwa::info!(
        "[task_stepper] Number of axis: {}. Segment sampling: {} Hz ({} us period). Micro-segment interpolation: {} Hz ({} us period)",
        CoordSel::num_axis(),
        Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY,
        STEPPER_PLANNER_MICROSEGMENT_PERIOD_US,
        Contract::STEP_PLANNER_CLOCK_FREQUENCY,
        STEPPER_PLANNER_CLOCK_PERIOD_US,
    );

    #[cfg(feature = "with-e-axis")]
    hwa::info!("Extruder enabled");

    event_bus
        .publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY))
        .await;

    cfg_if::cfg_if! {
        if #[cfg(feature="verbose-timings")] {
            //let mut global_timer = embassy_time::Instant::now();
        }
    }

    // Metrics: when the previous segment ended, if nothing was queued behind it (the wait for the
    // next one is a gap in the motion)
    #[allow(unused_mut, unused_variables)]
    let mut _starving_since: Option<u64> = None;

    loop {
        let consumed = match embassy_time::with_timeout(
            STEPPER_INACTIVITY_TIMEOUT,
            motion_planner.next_plan(&event_bus),
        )
        .await
        {
            Err(_) => {
                // Timeout
                let moves_left = motion_planner.num_queued().await;
                #[cfg(feature = "trace-commands")]
                hwa::info!(
                    "[trace-commands] [task_stepper] Timeout. Queue size: {}",
                    moves_left
                );
                if !steppers_off {
                    park(&motion_planner).await;
                    steppers_off = true;
                }
                #[cfg(test)]
                if crate::tasks::task_integration::INTEGRATION_STATUS.signaled() {
                    hwa::info!("[task_stepper] Ending gracefully");
                    return ();
                }
                ConsumedPlan {
                    moves_left,
                    tools: hwa::controllers::ToolActions::NONE,
                    order_num: 0,
                }
            }
            Ok(ExecPlan::Segment(_, _channel, _order_num))
                if motion_aborted() || event_bus.has_flags(EventFlags::SYS_ALARM).await =>
            {
                hwa::debug!("[task_stepper] order_num:{:?} Segment discarded", _order_num);
                motion_planner.consume_current_plan(&event_bus).await
            }
            Ok(ExecPlan::Segment(mut segment, _channel, _order_num)) => 'segment: {
                #[cfg(all(test, feature = "native"))]
                segment_trace::record(&segment, _order_num);
                // Metrics: the gap since the previous segment ran out of queue (bounded: a longer
                // wait is the machine being idle, not starving)
                let _t_fetch = metrics::now_us();
                if let Some(since) = _starving_since.take() {
                    let gap = _t_fetch.saturating_sub(since);
                    if gap < 1_000_000 {
                        metrics::record_starved_gap(gap);
                    }
                }
                metrics::set_motion_active(true);
                #[cfg(feature = "motion-metrics")]
                metrics::record_lookahead(motion_planner.num_queued().await.saturating_sub(1));
                // Process segment plan. Only when the machine is ready (power on, drivers
                // configured): waited for, bounded
                if !wait_motion_ready(&mut _s, &event_bus, &motion_planner).await {
                    hwa::warn!("[task_stepper] order_num:{:?} Segment discarded (not ready)", _order_num);
                    metrics::set_motion_active(false);
                    break 'segment motion_planner.consume_current_plan(&event_bus).await;
                }

                #[cfg(feature = "verbose-timings")]
                let t0 = embassy_time::Instant::now();
                #[cfg(feature = "verbose-timings")]
                let mut tq = 0;

                #[cfg(any(feature = "debug-motion", feature = "debug-position"))]
                hwa::debug!(
                    "[task_stepper] order_num:{:?} Commiting segment",
                    _order_num
                );

                let current_real_pos = motion_planner.motion_status().get_current_position();

                #[cfg(feature = "debug-motion")]
                hwa::info!(
                    "[task_stepper] order_num:{:?} Current real position is: world [{:#?}] {}",
                    _order_num,
                    current_real_pos.world_pos,
                    Contract::WORLD_UNIT_MAGNITUDE,
                );
                #[cfg(any(feature = "debug-motion", feature = "debug-position"))]
                hwa::info!(
                    "[task_stepper] order_num:{:?} Current real position is: space [{:#?}] {}",
                    _order_num,
                    current_real_pos.space_pos,
                    Contract::SPACE_UNIT_MAGNITUDE,
                );

                #[cfg(any(feature = "debug-motion", feature = "debug-position"))]
                hwa::info!(
                    "[task_stepper] order_num:{:?} Planned src: [{:#?}] dest: [{:#?}] {}",
                    _order_num,
                    segment.src_pos,
                    segment.dest_pos,
                    Contract::SPACE_UNIT_MAGNITUDE,
                );
                #[cfg(feature = "debug-motion")]
                hwa::info!(
                    "[task_stepper] order_num:{:?} MotionPlan dequeued. vector displacement space: [{:#?}] {}, speed: [ vin: {:?}, vout: {:?} ]",
                    _order_num,
                    segment.unit_vector_dir * segment.displacement_su,
                    Contract::SPACE_UNIT_MAGNITUDE,
                    segment.speed_enter_su_s,
                    segment.speed_exit_su_s,
                );

                let position_offset = segment.src_pos - current_real_pos.space_pos;
                #[cfg(any(
                    feature = "debug-motion",
                    feature = "debug-position",
                    feature = "debug-position-deviation"
                ))]
                hwa::info!(
                    "[task_stepper] order_num:{:?} Correcting offset [{:?}] {}",
                    _order_num,
                    position_offset,
                    Contract::SPACE_UNIT_MAGNITUDE,
                );
                if !position_offset.abs().bounded_by(&TVector::one()) {
                    hwa::warn!(
                        "[task_stepper] order_num:{:?} Too much offset to correct",
                        _order_num
                    )
                }
                // The real position is at a step (the closest one to the ideal position where the
                // previous segment ended). Of the deviation of the motion axes, only whole steps
                // redirect the segment: the fraction (within ±½ step) is handed to the
                // interpolator, so that the motors follow the ideal segment instead of a segment
                // shifted by that fraction (which, on short segments such as those of arcs and
                // splines, adds up to one step of path error). The extruder is corrected as before.
                let deviation_spu: TVector<Real> = motion_planner
                    .motion_config()
                    .get_units_per_space_magnitude()
                    * motion_planner.motion_config().get_micro_steps_as_vector();
                let motion_axes = CoordSel::motion_relevant_axis();
                // Per axis: (deviation fixed by redirecting the segment, remaining fraction in steps)
                let whole_deviation = position_offset.map_values(|coord, offset| {
                    match (motion_axes.contains(coord), deviation_spu.get_coord(coord)) {
                        (true, Some(spu)) if spu.is_defined_positive() => {
                            Some((offset * spu).round() / spu)
                        }
                        _ => Some(offset),
                    }
                });
                let fraction_steps = position_offset.map_values(|coord, offset| {
                    match (motion_axes.contains(coord), deviation_spu.get_coord(coord)) {
                        (true, Some(spu)) if spu.is_defined_positive() => {
                            Some(offset * spu - (offset * spu).round())
                        }
                        _ => None,
                    }
                });
                segment.fix_deviation(&whole_deviation);
                // In the direction of motion of each axis: (src - real) along the axis direction
                let start_offset_steps = segment.unit_vector_dir.map_values(|coord, dir| {
                    let f = fraction_steps.get_coord(coord)?;
                    Some(if dir < math::ZERO { -f } else { f })
                });

                #[cfg(feature = "debug-motion")]
                hwa::info!(
                    "[task_stepper] order_num:{:?} MotionPlan refined. vector displacement space: [{:#?}] {}, speed: [ vin: {:?}, vout: {:?} ]",
                    _order_num,
                    segment.unit_vector_dir * segment.displacement_su,
                    Contract::SPACE_UNIT_MAGNITUDE,
                    segment.speed_enter_su_s,
                    segment.speed_exit_su_s,
                );

                #[cfg(feature = "verbose-timings")]
                let t_calc = embassy_time::Instant::now();

                // Compute the Motion Profile
                let _t_profile = metrics::now_us();
                let _profile = HybridMotionProfile::compute(
                    segment.displacement_su,
                    segment.speed_enter_su_s,
                    segment.speed_exit_su_s,
                    #[cfg(feature = "with-motion-chaining")]
                    segment.accel_enter_su_s,
                    #[cfg(feature = "with-motion-chaining")]
                    segment.accel_exit_su_s,
                    &segment.constraints,
                    false,
                );
                metrics::record_profile(metrics::now_us().saturating_sub(_t_profile));
                match _profile {
                    Ok(trajectory) => {
                        #[cfg(feature = "debug-motion")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory computed. linear displacement: {:?} time: {:?} vlim: {:?}",
                            _order_num,
                            trajectory.end_pos(),
                            trajectory.end_time(),
                            trajectory.v_lim,
                        );

                        // The relevant coords (those that will move)
                        let mut relevant_coords = CoordSel::empty();
                        // The relevant coords that will move forward
                        let mut relevant_coords_dir_fwd = CoordSel::empty();

                        segment.unit_vector_dir.foreach_values(|coord, val| {
                            relevant_coords.set(coord, !val.is_negligible());
                            relevant_coords_dir_fwd.set(coord, val.is_defined_positive());
                        });
                        hwa::debug!("Relevant coords: [{:?}]", relevant_coords);
                        hwa::debug!("Relevant coords_dir_fwd: [{:?}]", relevant_coords_dir_fwd);

                        #[cfg(any(
                            feature = "debug-motion",
                            feature = "debug-motion-displacement"
                        ))]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory: [ src: [{:?}] dest: [{:?}] {} ] v0: {:?} v1: {:?} vlim: {:?}",
                            _order_num,
                            segment
                                .src_pos
                                .with_coord(relevant_coords.complement(), None),
                            segment
                                .dest_pos
                                .with_coord(relevant_coords.complement(), None),
                            Contract::SPACE_UNIT_MAGNITUDE,
                            segment.speed_enter_su_s,
                            segment.speed_exit_su_s,
                            trajectory.v_lim,
                        );

                        cfg_if::cfg_if! {
                            if #[cfg(feature="assert-motion")] {
                                let mut steps_to_advance: TVector<u32> = TVector::zero();
                                let mut steps_advanced: TVector<u32> = TVector::zero();
                            }
                        }

                        // Steps per world unit ratio
                        let steps_per_su: TVector<Real> = motion_planner
                            .motion_config()
                            .get_units_per_space_magnitude()
                            * motion_planner.motion_config().get_micro_steps_as_vector();

                        #[cfg(feature = "with-motion-broadcast")]
                        let mut delta = hwa::MotionDelta::new();

                        let mut segment_iterator =
                            SegmentSampler::new(&trajectory, micro_segment_period_secs);

                        let mut micro_segment_interpolator =
                            MicroSegmentInterpolator::new_with_start_offset(
                                segment
                                    .unit_vector_dir
                                    .with_coord(relevant_coords.complement(), None)
                                    .abs(),
                                segment.displacement_su,
                                steps_per_su.with_coord(relevant_coords.complement(), None),
                                start_offset_steps,
                            );

                        #[cfg(feature = "verbose-timings")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory computation took: {} us. displacement: {:?} {} in {:?} s",
                            _order_num,
                            t_calc.elapsed().as_micros(),
                            trajectory.end_pos(),
                            Contract::SPACE_UNIT_MAGNITUDE,
                            trajectory.end_time(),
                        );
                        #[cfg(feature = "verbose-timings")]
                        let t_bus = embassy_time::Instant::now();
                        event_bus
                            .publish_event(EventStatus::containing(EventFlags::MOVING))
                            .await;
                        // Without a step actuator (no stepper nor broadcast) nothing drains the
                        // step queue: moves are executed virtually (as in dry run) so the system
                        // keeps progressing instead of waiting forever
                        let do_dry_run = !HAS_STEP_ACTUATOR
                            || event_bus.has_flags(EventFlags::DRY_RUN).await;
                        if steppers_off {
                            #[cfg(feature = "trace-commands")]
                            hwa::info!("[trace-commands] Powering steppers on");
                            unpark(&motion_planner, false).await;
                        }
                        steppers_off = false;
                        #[cfg(feature = "verbose-timings")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Status update took: {} us",
                            _order_num,
                            t_bus.elapsed().as_micros()
                        );

                        ////
                        //// TRAJECTORY INTERPOLATION START
                        ////
                        #[cfg(feature = "debug-motion")]
                        hwa::debug!(
                            "[task_stepper] order_num:{:?} Trajectory interpolation START",
                            _order_num
                        );

                        // The current micro-segment id
                        #[cfg(any(feature = "verbose-timings", feature = "with-motion-broadcast"))]
                        let mut micro_segment_id: u32 = 0;

                        // Profile velocity at the start of the current window (second-order DDA)
                        let mut v_window_start = trajectory.eval_velocity(math::ZERO);

                        // Laser dynamic power (M4): the mean speed of each window relative to the
                        // programmed one, per mille (`(v_start + v_end) · 500 / v_programmed`).
                        // None for the moves without a programmed cutting speed (G0)
                        #[cfg(feature = "with-laser")]
                        let laser_ratio_scale = match segment.speed_programmed_su_s.is_defined_positive() {
                            true => Some(Real::from_lit(500, 0) / segment.speed_programmed_su_s),
                            false => None,
                        };

                        // Metrics: interpolation CPU time of the segment (push waits excluded)
                        let _t_interp = metrics::now_us();
                        #[allow(unused_mut)]
                        let mut _push_wait_us: u64 = 0;

                        // Micro-segments interpolation along segment
                        loop {
                            // Micro-segment start

                            hwa::trace!("Micro-segment START");

                            if motion_aborted() {
                                hwa::warn!(
                                    "[task_stepper] order_num:{:?} Motion aborted",
                                    _order_num
                                );
                                break;
                            }

                            if let Some(estimated_position) = segment_iterator.next() {
                                cfg_if::cfg_if! {
                                    if #[cfg(any(feature = "verbose-timings", feature = "with-motion-broadcast"))] {
                                        micro_segment_id += 1;
                                    }
                                }
                                cfg_if::cfg_if! {
                                    if #[cfg(feature = "with-motion-broadcast")] {
                                        delta.pos_wu = segment.src_pos + segment.unit_vector_dir * segment_iterator.current_position();
                                        delta.micro_segment_id = micro_segment_id;
                                        delta.micro_segment_time = segment_iterator.current_time() - segment_iterator.dt();
                                    }
                                }

                                #[cfg(feature = "verbose-timings")]
                                hwa::info!(
                                    "[task_stepper] order_num:{}|{} Trajectory micro-segment sampled at {:?} secs. dt = {:?} ds = {:?} v = {:?}",
                                    _order_num,
                                    micro_segment_id,
                                    segment_iterator.current_time(),
                                    segment_iterator.dt(),
                                    segment_iterator.ds(),
                                    segment_iterator.instant_velocity(),
                                );

                                // Exact integer window width (µs). The sampler never yields a zero-width
                                // window (short tails are merged), so no step can be dropped here.
                                let w = Real::from_lit(segment_iterator.dt_us().max(1) as i64, 0);
                                // Step rate ramps linearly from the profile velocity at the start
                                // of the window to the one at its end (continuous motor velocity).
                                let v_window_end = segment_iterator.current_speed();
                                #[cfg(feature = "with-laser")]
                                let speed_ratio = laser_ratio_scale.map(|scale| {
                                    ((v_window_start + v_window_end) * scale)
                                        .int()
                                        .clamp(0, u16::MAX as i64) as u16
                                });
                                #[cfg(not(feature = "with-laser"))]
                                let speed_ratio = None;
                                let _has_more = match micro_segment_interpolator
                                    .advance_to_with_velocity(
                                        estimated_position,
                                        w,
                                        v_window_start,
                                        v_window_end,
                                        STEPPER_PLANNER_CLOCK_PERIOD_US,
                                    )
                                {
                                    Ok(has_more) => {
                                        v_window_start = v_window_end;
                                        has_more
                                    }
                                    Err(_) => {
                                        hwa::error!(
                                            "[task_stepper] order_num:{:?} Unable to advance micro-segment (w = {:?} us)",
                                            _order_num,
                                            w
                                        );
                                        break;
                                    }
                                };

                                #[cfg(feature = "verbose-timings")]
                                hwa::info!(
                                    "[task_stepper] order_num:{:?}|{:?} Trajectory micro-segment advanced: {:?} {} {:?} steps",
                                    _order_num,
                                    micro_segment_id,
                                    micro_segment_interpolator
                                        .advanced_units()
                                        .map_nan_coords(relevant_coords, &math::ZERO),
                                    Contract::SPACE_UNIT_MAGNITUDE,
                                    micro_segment_interpolator
                                        .advanced_steps()
                                        .map_nan_coords(relevant_coords, &0),
                                );

                                cfg_if::cfg_if! {
                                    if #[cfg(feature="assert-motion")] {
                                        steps_to_advance += micro_segment_interpolator.delta;
                                    }
                                }

                                #[cfg(feature = "verbose-timings")]
                                let t1 = embassy_time::Instant::now();

                                let _t_push = metrics::now_us();
                                if !do_dry_run {
                                    STEP_DRIVER
                                        .push(
                                            _order_num,
                                            #[cfg(feature = "with-motion-broadcast")]
                                            delta,
                                            micro_segment_interpolator.state().clone(),
                                            relevant_coords,
                                            relevant_coords_dir_fwd,
                                            speed_ratio,
                                        )
                                        .await;
                                }
                                _push_wait_us += metrics::now_us().saturating_sub(_t_push);
                                #[cfg(feature = "verbose-timings")]
                                {
                                    tq += t1.elapsed().as_micros();
                                }
                                if !_has_more {
                                    break;
                                }
                            } else {
                                // No advance
                                break;
                            }
                            hwa::trace!("Micro-segment END");
                            // Microsegment end
                        }

                        metrics::record_interp(
                            metrics::now_us()
                                .saturating_sub(_t_interp)
                                .saturating_sub(_push_wait_us),
                        );
                        metrics::record_push_wait(_push_wait_us);

                        ////
                        //// TRAJECTORY INTERPOLATION END
                        ////
                        #[cfg(feature = "debug-motion")]
                        hwa::debug!(
                            "[task_stepper] order_num:{:?} Trajectory interpolation END",
                            _order_num
                        );

                        #[cfg(feature = "debug-motion")]
                        let adv_steps = micro_segment_interpolator.advanced_steps();
                        let adv_pos = micro_segment_interpolator.advanced_units();

                        #[cfg(feature = "debug-motion")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory advanced. vector displacement space: [{:#?}] {}, vlim: {:?} {}/s",
                            _order_num,
                            adv_pos,
                            Contract::SPACE_UNIT_MAGNITUDE,
                            trajectory.v_lim,
                            Contract::SPACE_UNIT_MAGNITUDE
                        );

                        #[cfg(feature = "debug-motion")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory advanced. vector displacement space: [{:#?}] steps",
                            _order_num,
                            adv_steps.excluding_negligible()
                        );

                        cfg_if::cfg_if! {
                            if #[cfg(feature="assert-motion")] {
                                if !do_dry_run {
                                    cfg_if::cfg_if! {
                                        if #[cfg(feature="assert-motion")] {
                                            steps_advanced += STEP_DRIVER.flush().await;
                                        }
                                    }
                                    if steps_advanced != steps_to_advance {
                                        hwa::error!("[task_stepper] order_num:{:?} Motion assertion failure. Advanced: {:?} Expected: {:?}",
                                            _order_num, steps_advanced, steps_to_advance);
                                    }
                                }
                            }
                        }
                        {
                            let adv_delta = segment.unit_vector_dir.sign() * adv_pos;
                            #[cfg(feature = "debug-motion")]
                            hwa::info!(
                                "[task_stepper] order_num:{:?} Advanced space: [{:#?}] {} {:?} steps",
                                _order_num,
                                adv_delta,
                                Contract::SPACE_UNIT_MAGNITUDE,
                                micro_segment_interpolator.advanced_steps()
                            );
                            let next_real_pos = (current_real_pos.space_pos
                                + adv_delta.map_nan(&math::ZERO))
                            .rdp(6);
                            let mut pos = current_real_pos.clone();
                            pos.update_from_space_coordinates(&next_real_pos);

                            #[cfg(feature = "debug-motion")]
                            hwa::info!(
                                "[task_stepper] order_num:{:?} Next real position: world [{:#?}] {}",
                                _order_num,
                                pos.world_pos,
                                Contract::WORLD_UNIT_MAGNITUDE,
                            );

                            #[cfg(any(feature = "debug-motion", feature = "debug-position"))]
                            hwa::info!(
                                "[task_stepper] order_num:{:?} Next real position: space [{:#?}] {}",
                                _order_num,
                                pos.space_pos,
                                Contract::SPACE_UNIT_MAGNITUDE,
                            );
                            motion_planner
                                .motion_status()
                                .update_current_position(_order_num, &pos);
                        }

                        let consumed = motion_planner.consume_current_plan(&event_bus).await;
                        #[allow(unused)]
                        let moves_left = consumed.moves_left;
                        #[cfg(feature = "motion-metrics")]
                        {
                            let t_end = metrics::now_us();
                            // Integer conversions (no f64: soft-float on every board)
                            // TMPTRACE
                            hwa::debug!(
                                "TMPTRACE S t={} plan={} wall={} left={} vi={} vo={} s={}",
                                t_end as u32,
                                (trajectory.end_time() * math::ONE_MILLION).int(),
                                t_end.saturating_sub(_t_fetch) as u32,
                                moves_left,
                                (segment.speed_enter_su_s * Real::from_f32(10.0)).int(),
                                (segment.speed_exit_su_s * Real::from_f32(10.0)).int(),
                                (segment.displacement_su * math::ONE_THOUSAND).int()
                            );
                            metrics::record_segment(
                                (trajectory.end_time() * math::ONE_MILLION).int().max(0) as u64,
                                t_end.saturating_sub(_t_fetch),
                                (segment.displacement_su * math::ONE_THOUSAND).int().max(0) as u64,
                                segment.speed_exit_su_s.is_negligible(),
                            );
                            if moves_left == 0 {
                                // The queue ran out: what follows is a gap (or the end of the job)
                                metrics::set_motion_active(false);
                                _starving_since = Some(t_end);
                            }
                        }
                        event_bus
                            .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                            .await;

                        cfg_if::cfg_if! {
                            if #[cfg(feature="verbose-timings")] {
                                use motion::MotionProfile;

                                //global_timer = embassy_time::Instant::now();
                                hwa::info!("[task_stepper] v_0: {:?}, v_lim: {:?}, v_1: {:?}; t: {:?}; dist: {{ [{:#?}] {}, |dist|: {:?} {}, steps: [{:#?}] }}; [{:?} moves left]",
                                    segment.speed_enter_su_s.rdp(3),
                                    trajectory.v_lim.rdp(3),
                                    segment.speed_exit_su_s.rdp(3),
                                    trajectory.end_time(),
                                    micro_segment_interpolator.advanced_units(),
                                    Contract::SPACE_UNIT_MAGNITUDE,
                                    micro_segment_interpolator.advanced_units().norm2().unwrap_or(math::ZERO).rdp(4),
                                    Contract::SPACE_UNIT_MAGNITUDE,
                                    micro_segment_interpolator.advanced_steps(),
                                    moves_left,
                                );
                                let t_tot = t0.elapsed().as_micros() as f32 / 1000000.0;
                                let t_qw = tq as f32 / 1000000.0;
                                hwa::debug!("end in {:?} s : {:?} - {:?}", t_tot - t_qw, t_tot, t_qw);
                            }
                        }
                        hwa::debug!("[task_stepper] MOTION_PLAN END ({} moves left)", moves_left);

                        consumed
                        // segment end
                    }
                    Err(_e) => {
                        hwa::error!("Unable to compute motion plan. Discarding. err: {:?}. Segment: {:?}", _e, segment);
                        let consumed = motion_planner.consume_current_plan(&event_bus).await;
                        event_bus
                            .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                            .await;
                        consumed
                    }
                }
            }
            Ok(ExecPlan::Dwell(sleep_ms, _channel, _order_num)) => {
                // Dwell. From the end of the motion queued before it (the steps still queued in
                // the step executor, and the tool actions among them), so that a dwell (G4)
                // lasts what was requested once the head is still, and M400 (a dwell of zero) is
                // answered once the motion really ended
                if !motion_aborted() {
                    STEP_DRIVER.flush().await;
                }
                if motion_aborted() {
                    hwa::debug!("[task_stepper] Dwell discarded (motion aborted)");
                } else if let Some(millis) = sleep_ms {
                    hwa::debug!("G4: Sleeping {} millis", millis);
                    embassy_time::Timer::after(Duration::from_millis(millis.into())).await;
                } else {
                    hwa::debug!("G4 Not sleeping");
                }
                let consumed = motion_planner.consume_current_plan(&event_bus).await;
                event_bus
                    .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                    .await;
                consumed
            }
            Ok(ExecPlan::SetPosition(position, _channel, _order_num)) => {
                // SetPosition
                motion_planner
                    .motion_status()
                    .update_current_position(_order_num, &position);
                motion_planner.consume_current_plan(&event_bus).await
            }
            Ok(ExecPlan::Tools(_channel, _order_num)) => {
                // Tool actions queued alone: applied below, after the steps already queued
                motion_planner.consume_current_plan(&event_bus).await
            }
            Ok(ExecPlan::Homing(_channel, _order_num)) => {
                // Homing
                hwa::debug!("Homing init");

                let mut homing_failed = false;
                // Where the homing ended (native): the expected point unless it ran (dry run and
                // debug skip leave the machine where it is assumed to be)
                #[allow(unused_mut)]
                let mut homed_native = motion_planner
                    .motion_config()
                    .get_native_post_homing_position();
                if motion_aborted() {
                    hwa::warn!("[task_stepper] Homing discarded (motion aborted)");
                    homing_failed = true;
                } else if !wait_motion_ready(&mut _s, &event_bus, &motion_planner).await {
                    hwa::warn!("[task_stepper] Homing discarded (not ready)");
                    homing_failed = true;
                } else if !event_bus.has_flags(EventFlags::DRY_RUN).await {
                    if steppers_off {
                        #[cfg(feature = "trace-commands")]
                        hwa::info!("[trace-commands] Powering steppers on");
                        unpark(&motion_planner, true).await;
                        steppers_off = false;
                    }
                    cfg_if::cfg_if! {
                        if #[cfg(feature="debug-skip-homing")] {
                            hwa::info!("Skipping homing for debug purposes");
                            // Do nothing
                        }
                        else {
                            match motion_planner.do_homing(_order_num, &event_bus).await {
                                Ok(native) => homed_native = native,
                                Err(_) => {
                                homing_failed = true;
                                if motion_aborted() {
                                    // Interrupted on purpose (M410/M112): not an end-stop failure
                                    hwa::warn!("[task_stepper] Homing aborted");
                                } else {
                                    hwa::error!("[task_stepper] Homing failed. Raising SYS_ALARM");
                                    event_bus
                                        .publish_event(EventStatus::containing(EventFlags::SYS_ALARM))
                                        .await;
                                }
                                }
                            }
                        }
                    }
                } else {
                    hwa::debug!("dry_run: Skipping homing");
                }
                if homing_failed {
                    // The position is unknown
                    park(&motion_planner).await;
                    steppers_off = true;
                    motion_planner.motion_status().invalidate_position();
                } else {
                    // Right after homing, the logical position is where the homing ended (the safe
                    // homing point, Z at the clearance above the bed plane) plus the home offsets
                    // (M206)
                    let position = hwa::controllers::Position::new_with_world_projection(
                        &motion_planner.motion_config().logical_from_native(&homed_native),
                    );
                    motion_planner
                        .motion_status()
                        .update_last_planned_position(_order_num, &position);
                    motion_planner
                        .motion_status()
                        .update_current_position(_order_num, &position);
                    motion_planner.motion_status().set_homed(true);
                }

                let consumed = motion_planner.consume_current_plan(&event_bus).await;
                hwa::debug!("Homing done");
                consumed
            }
        };
        hwa::trace!("Moves left: {}", consumed.moves_left);
        // The tool actions requested right after the consumed entry
        if !consumed.tools.is_empty() {
            deliver_tools(&event_bus, consumed.tools, consumed.order_num, steppers_off).await;
        }
    }
}

/// Hands the tool actions requested right after an executed entry to the step executor: they
/// are applied right after the step windows already queued (those of the entry), and right
/// before the windows of the next motion, which is not delayed (see
/// [hwa::controllers::StepPlan::marker]). Applied at once when nothing moves (the steppers are
/// parked: the step executor is idle and its ticker paused) or the motion is virtual (dry run, no
/// step actuator), and discarded with the motion on abort (M410/M112) or alarm.
async fn deliver_tools(
    event_bus: &hwa::types::EventBus,
    tools: hwa::controllers::ToolActions,
    order_num: u32,
    steppers_off: bool,
) {
    if motion_aborted() || event_bus.has_flags(EventFlags::SYS_ALARM).await {
        hwa::debug!("[task_stepper] order_num:{:?} Tool actions discarded", order_num);
        return;
    }
    let dry_run = event_bus.has_flags(EventFlags::DRY_RUN).await;
    let tools = if dry_run { tools.for_dry_run() } else { tools };
    if steppers_off || dry_run || !HAS_STEP_ACTUATOR {
        STEP_DRIVER.apply_tools_now(&tools);
    } else {
        STEP_DRIVER.push_tools(order_num, tools).await;
    }
}

/// The tool outputs the step executor switches: the PWM of the fans and the laser (shared with
/// their controllers)
async fn tool_outputs(
    _motion_planner: &hwa::controllers::MotionPlanner,
) -> hwa::controllers::ToolOutputs {
    #[allow(unused)]
    let driver = _motion_planner.motion_driver().lock().await;
    hwa::controllers::ToolOutputs::new(
        #[cfg(feature = "with-fan-layer")]
        (*driver.fan_layer_controller.lock().await).clone(),
        #[cfg(feature = "with-fan-extra-1")]
        (*driver.fan_extra_1_controller.lock().await).clone(),
        #[cfg(feature = "with-laser")]
        (*driver._laser_controller.lock().await).clone(),
    )
}

/// Waits until the machine is ready for motion ([crate::helpers::MOTION_READY_FLAGS]: power on,
/// drivers configured), for [crate::helpers::MOTION_READY_TIMEOUT_MS] at most. Normally immediate:
/// the commands are only planned when ready. Fails in alarm mode, and on timeout (e.g. the power
/// went off with moves queued): then the alarm is raised (the queued moves are discarded) and the
/// position is lost (homing is required). Never waits forever.
async fn wait_motion_ready<M: hwa::AsyncRawMutex>(
    subscriber: &mut hwa::EventBusSubscriber<'static, M>,
    event_bus: &hwa::types::EventBus,
    motion_planner: &hwa::controllers::MotionPlanner,
) -> bool {
    match embassy_time::with_timeout(
        Duration::from_millis(crate::helpers::MOTION_READY_TIMEOUT_MS),
        subscriber.ft_wait_for(EventStatus::containing(crate::helpers::MOTION_READY_FLAGS)),
    )
    .await
    {
        Ok(Ok(())) => true,
        // SYS_ALARM: the queued moves are being discarded
        Ok(Err(())) => false,
        Err(_) => {
            hwa::error!(
                "[task_stepper] Not ready for motion (power / TMC drivers): {:?}. Raising SYS_ALARM",
                event_bus.get_status().await
            );
            motion_planner.motion_status().invalidate_position();
            event_bus
                .publish_event(EventStatus::containing(EventFlags::SYS_ALARM))
                .await;
            false
        }
    }
}

/// Test instrumentation of the native simulator: the planned junction speeds of the segments, as
/// the stepper executes them
#[cfg(all(test, feature = "native"))]
pub mod segment_trace {
    use std::sync::Mutex;

    /// A segment: order number, entry, exit and rest speeds (space units per second)
    #[derive(Clone, Copy, Debug)]
    pub struct Record {
        pub order_num: u32,
        pub v_enter: f64,
        pub v_exit: f64,
        pub v_rest: f64,
    }

    static LOG: Mutex<Option<std::vec::Vec<Record>>> = Mutex::new(None);

    /// Starts recording
    pub fn start() {
        *LOG.lock().unwrap() = Some(std::vec::Vec::new());
    }

    /// Stops recording and returns the segments executed since [start], in order
    pub fn stop() -> std::vec::Vec<Record> {
        LOG.lock().unwrap().take().unwrap_or_default()
    }

    pub(crate) fn record(segment: &crate::hwa::controllers::Segment, order_num: u32) {
        if let Some(log) = LOG.lock().unwrap().as_mut() {
            log.push(Record {
                order_num,
                v_enter: segment.speed_enter_su_s.to_f64(),
                v_exit: segment.speed_exit_su_s.to_f64(),
                #[cfg(feature = "with-motion-chaining")]
                v_rest: segment.speed_rest_su_s.to_f64(),
                #[cfg(not(feature = "with-motion-chaining"))]
                v_rest: 0.0,
            });
        }
    }
}

/// Request to abort the ongoing motion and discard the queued plans (M410/M112)
static MOTION_ABORT: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Sets or clears the motion abort request.
///
/// While set, the ongoing segment is interrupted at the next micro-segment boundary (the real
/// position is kept consistent with the steps actually performed) and every queued plan is
/// discarded without being executed.
pub fn abort_motion(abort: bool) {
    MOTION_ABORT.store(abort, core::sync::atomic::Ordering::Release);
}

/// Whether a motion abort (M410/M112) is in progress
#[inline]
pub fn motion_aborted() -> bool {
    MOTION_ABORT.load(core::sync::atomic::Ordering::Acquire)
}

async fn park(motion_planner: &hwa::controllers::MotionPlanner) {
    #[cfg(feature = "trace-commands")]
    hwa::info!("[trace-commands] [task_stepper] Stepping parked");
    STEP_DRIVER.flush().await;
    motion_planner
        .motion_driver()
        .lock()
        .await
        .step_actuator()
        .disable_steppers(CoordSel::all_axis());
    STEP_DRIVER.reset();
    hwa::controllers::motion_control::pause_ticker();
}

async fn unpark(motion_planner: &hwa::controllers::MotionPlanner, enable_steppers: bool) {
    hwa::controllers::motion_control::resume_ticker();
    if enable_steppers {
        STEP_DRIVER.flush().await;
        motion_planner
            .motion_driver()
            .lock()
            .await
            .step_actuator()
            .enable_steppers(CoordSel::all_axis());
        STEP_DRIVER.reset();
    }
    #[cfg(feature = "trace-commands")]
    hwa::info!("[trace-commands] [task_stepper] Stepping resumed");
}
