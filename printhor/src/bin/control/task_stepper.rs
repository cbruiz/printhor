//! Interpolator Step algorithm.
//!
//! The IS algorithm works the following way:
//! - Try to retrieve an execution-ready motion segment from the motion queue
//!   - If no motion segment is present within [`STEPPER_INACTIVITY_TIMEOUT`] seconds, disable all steppers
//!   - If motion segment is execution-ready, dequeue it and then:
//!     - Enable all steppers
//!     - Evaluate the motion profile displacement at [`Contract::STEP_PLANNER_CLOCK_FREQUENCY`]
//!     - Compute number of steps to do in each axis (independently)
//!     - Compute pulse rate across each axis (independently) and construct an iterator leveraging [`MultiTimer`](hwa::controllers::motion::MultiTimer)
//!     - Consume a micro-segment until iterator is exhausted
//!
//! TODO: This is a work still in progress

use crate::control;
use crate::hwa;
#[allow(unused)]
use control::motion::MotionProfile;
use control::motion::SCurveMotionProfile;
use embassy_time::Duration;
use hwa::controllers::ExecPlan;
use hwa::controllers::LinearMicrosegmentStepInterpolator;
use hwa::controllers::motion::STEP_DRIVER;
use hwa::controllers::motion::SegmentIterator;
use hwa::math;

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
const STEPPER_INACTIVITY_TIMEOUT: Duration = Duration::from_secs(10);

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

    STEP_DRIVER.setup(motion_planner.motion_driver().lock().await.step_actuator().clone());

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

    loop {
        let _moves_left = match embassy_time::with_timeout(
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
                if control::task_integration::INTEGRATION_STATUS.signaled() {
                    hwa::info!("[task_stepper] Ending gracefully");
                    return ();
                }
                moves_left
            }
            Ok(ExecPlan::Segment(mut segment, _channel, _order_num)) => {
                // Process segment plan
                #[cfg(feature = "with-ps-on")]
                match _s
                    .ft_wait_for(EventStatus::containing(EventFlags::ATX_ON))
                    .await
                {
                    Ok(_) => {}
                    Err(_) => {
                        let _ = _s.ft_wait_while(EventFlags::SYS_ALARM).await;
                    }
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
                segment.fix_deviation(
                    &position_offset,
                    motion_planner.motion_config().get_flow_rate_as_real(),
                );

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
                match SCurveMotionProfile::compute(
                    segment.displacement_su,
                    segment.speed_enter_su_s,
                    segment.speed_exit_su_s,
                    &segment.constraints,
                    false,
                ) {
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
                            SegmentIterator::new(&trajectory, micro_segment_period_secs);

                        let mut micro_segment_interpolator =
                            LinearMicrosegmentStepInterpolator::new(
                                segment
                                    .unit_vector_dir
                                    .with_coord(relevant_coords.complement(), None)
                                    .abs(),
                                segment.displacement_su,
                                steps_per_su.with_coord(relevant_coords.complement(), None),
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
                        let do_dry_run = event_bus.has_flags(EventFlags::DRY_RUN).await;
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

                        // Micro-segments interpolation along segment
                        loop {
                            // Micro-segment start

                            hwa::trace!("Micro-segment START");

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
                                    segment_iterator.speed(),
                                );

                                let w = (segment_iterator.dt() * math::ONE_MILLION).round();
                                if w.is_negligible() {
                                    hwa::info!("giving up for any reason");
                                    break;
                                }
                                let _has_more =
                                    micro_segment_interpolator.advance_to(estimated_position, w);

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

                                if !do_dry_run {
                                    STEP_DRIVER
                                        .push(
                                            _order_num,
                                            #[cfg(feature = "with-motion-broadcast")]
                                            delta,
                                            micro_segment_interpolator.state().clone(),
                                            relevant_coords,
                                            relevant_coords_dir_fwd,
                                        )
                                        .await;
                                }
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

                        let moves_left = motion_planner.consume_current_plan(&event_bus).await;
                        event_bus
                            .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                            .await;

                        cfg_if::cfg_if! {
                            if #[cfg(feature="verbose-timings")] {
                                use crate::control::motion::profile::MotionProfile;

                                //global_timer = embassy_time::Instant::now();
                                hwa::info!("[task_stepper] v_0: {:?}, v_lim: {:?}, v_1: {:?}; t: {:?}; dist: {{ [{:#?}] {}, |dist|: {:?} {}, steps: [{:#?}] }}; [{:?} moves left]",
                                    segment.speed_enter_su_s.rdp(3).inner(),
                                    trajectory.v_lim.rdp(3).inner(),
                                    segment.speed_exit_su_s.rdp(3).inner(),
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

                        moves_left
                        // segment end
                    }
                    Err(_) => {
                        hwa::error!("Unable to compute motion plan. Discarding...");
                        let moves_left = motion_planner.consume_current_plan(&event_bus).await;
                        event_bus
                            .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                            .await;
                        moves_left
                    }
                }
            }
            Ok(ExecPlan::Dwell(sleep_ms, _channel, _order_num)) => {
                // Dwell
                if let Some(millis) = sleep_ms {
                    hwa::info!("G4: Sleeping {} millis", millis);
                    embassy_time::Timer::after(Duration::from_millis(millis.into())).await;
                } else {
                    hwa::info!("G4 Not sleeping");
                }
                let moves_left = motion_planner.consume_current_plan(&event_bus).await;
                event_bus
                    .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                    .await;
                moves_left
            }
            Ok(ExecPlan::SetPosition(position, _channel, _order_num)) => {
                // SetPosition
                motion_planner
                    .motion_status()
                    .update_current_position(_order_num, &position);
                let moves_left = motion_planner.consume_current_plan(&event_bus).await;
                moves_left
            }
            Ok(ExecPlan::Homing(_channel, _order_num)) => {
                // Homing
                hwa::debug!("Homing init");

                if !event_bus.has_flags(EventFlags::DRY_RUN).await {
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
                            if !motion_planner.do_homing(_order_num, &event_bus).await.is_ok() {
                                hwa::error!("Error homing");
                                // TODO
                            }
                        }
                    }
                } else {
                    hwa::debug!("dry_run: Skipping homing");
                }
                let position = hwa::controllers::Position::new_with_world_projection(
                    &Contract::DEFAULT_WORLD_HOMING_POINT_WU,
                );
                motion_planner
                    .motion_status()
                    .update_last_planned_position(_order_num, &position);
                motion_planner
                    .motion_status()
                    .update_current_position(_order_num, &position);

                let moves_left = motion_planner.consume_current_plan(&event_bus).await;
                hwa::debug!("Homing done");
                moves_left
            }
        };
        hwa::trace!("Moves left: {}", _moves_left);
    }
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
    Contract::pause_ticker();
}

async fn unpark(motion_planner: &hwa::controllers::MotionPlanner, enable_steppers: bool) {
    Contract::resume_ticker();
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
