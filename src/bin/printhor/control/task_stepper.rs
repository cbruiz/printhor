//! Interpolator Step algorithm.
//!
//! The IS algorithm works the following way:
//! - Try to retrieve an execution-ready motion segment from the motion queue
//!   - If no motion segment is present within [`STEPPER_INACTIVITY_TIMEOUT`], disable all steppers
//!   - If motion segment is execution-ready, dequeue it and then:
//!     - Enable all steppers
//!     - Evaluate the motion profile displacement at [`crate::hwa::STEPPER_PLANNER_CLOCK_FREQUENCY`]
//!     - Compute number of steps to do in each axis (independently)
//!     - Compute pulse rate across each axis (independently) and construct an iterator leveraging [`MultiTimer`](hwa::controllers::motion::MultiTimer)
//!     - Consume a micro-segment until iterator is exhausted
//!
//! TODO: This is a work still in progress

use crate::control;
use crate::hwa;
use crate::math;
use crate::math::Real;
use crate::tgeo::{CoordSel, TVector};
use control::motion::SCurveMotionProfile;
use embassy_time::Duration;
use hwa::controllers::motion::SegmentIterator;
use hwa::controllers::motion::STEP_DRIVER;
use hwa::controllers::ExecPlan;
use hwa::controllers::LinearMicrosegmentStepInterpolator;
use hwa::StepperChannel;
use hwa::{DeferAction, DeferEvent, EventFlags, EventStatus};
use num_traits::ToPrimitive;

///
/// This constant defines the duration of inactivity after which the stepper motors
/// are automatically disabled to save power and prevent overheating.
///
/// # Value
/// The timeout is set to 5 seconds, represented using `embassy_time::Duration`.
///
/// # Usage
/// This constant is used in the stepper motor control task to determine the period
/// of inactivity before disabling the stepper motors. When no motion segment is
/// retrieved within this duration, the steppers are turned off automatically.
///
/// # Effect on Task Behavior
/// In the `task_stepper` routine, this timeout plays a crucial role. If no motion
/// segment is available within this 5-second window, the routine will automatically
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
const STEPPER_INACTIVITY_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(10);

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
const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
    1_000_000 / hwa::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;

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
pub const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / hwa::STEPPER_PLANNER_CLOCK_FREQUENCY;

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
    event_bus: hwa::EventBus<hwa::EventBusHolderType, hwa::EventBusPubSubMutexType>,
    motion_planner: hwa::controllers::MotionPlanner,
    _watchdog: hwa::StaticController<hwa::WatchDogHolderType<hwa::device::WatchDog>>,
) {
    let mut steppers_off = true;

    let mut real_steppper_pos: TVector<i32> = TVector::zero();

    let micro_segment_period_secs: Real =
        Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 6);
    let sampling_time: Real = Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US as i64, 6);

    motion_planner.start(&event_bus).await;

    STEP_DRIVER.setup(motion_planner.motion_driver());

    #[allow(unused_mut)]
    let mut _s = event_bus.subscriber().await;

    hwa::info!(
        "[task_stepper] Segment sampling: {} Hz ({} us period). Micro-segment interpolation: {} Hz ({} us period)",
        hwa::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY,
        STEPPER_PLANNER_MICROSEGMENT_PERIOD_US,
        hwa::STEPPER_PLANNER_CLOCK_FREQUENCY,
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
            Ok(ExecPlan::Segment(segment, channel)) => {
                // Process segment plan
                #[cfg(feature="with-ps-on")]
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

                hwa::trace!("SEGMENT START");

                #[cfg(feature = "verbose-timings")]
                let tx = embassy_time::Instant::now();
                event_bus
                    .publish_event(EventStatus::containing(EventFlags::MOVING))
                    .await;

                // Vector helper to filter out irrelevant axes
                let neutral_element = segment.segment_data.unit_vector_dir.map_val(&math::ZERO);
                // Compute the Motion Profile
                match SCurveMotionProfile::compute(
                    segment.segment_data.displacement_mm,
                    segment.segment_data.speed_enter_mms,
                    segment.segment_data.speed_exit_mms,
                    &segment.segment_data.constraints,
                    false,
                ) {
                    Ok(motion_profile) => {
                        cfg_if::cfg_if! {
                            if #[cfg(feature="assert-motion")] {
                                let mut steps_to_advance: TVector<u32> = TVector::zero();
                                let mut steps_advanced: TVector<u32> = TVector::zero();
                            }
                        }

                        // First, translate displacement in mm to steps
                        let (units_per_mm, micro_steps) = hwa::interrupt_free(|| {
                            let motion_cfg = motion_planner.motion_cfg();
                            let mg = motion_cfg.try_lock();
                            match mg {
                                Ok(_g) => (
                                    neutral_element + _g.units_per_mm,
                                    neutral_element + _g.get_usteps_as_vector(),
                                ),
                                Err(_e) => {
                                    panic!("Unexpectedly, cannot lock motion cfg")
                                }
                            }
                        });

                        let steps_per_mm: TVector<Real> = units_per_mm * micro_steps;

                        // The relative real time position (starting after first micro-segment)
                        let mut micro_segment_real_time_rel = micro_segment_period_secs;
                        let mut microsegment_iterator =
                            SegmentIterator::new(&motion_profile, math::ZERO);

                        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
                            segment.segment_data.unit_vector_dir.abs(),
                            segment.segment_data.displacement_mm,
                            steps_per_mm,
                        );

                        // Prepare enable and dir flags
                        let mut stepper_enable_flags = StepperChannel::empty();
                        let mut stepper_dir_fwd_flags = StepperChannel::empty();
                        segment.segment_data.unit_vector_dir.apply_coords(|cs| {
                            #[cfg(feature = "with-x-axis")]
                            if cs.0.contains(CoordSel::X) {
                                stepper_enable_flags.set(StepperChannel::X, true);
                                stepper_dir_fwd_flags
                                    .set(StepperChannel::X, cs.1.is_defined_positive());
                            }
                            #[cfg(feature = "with-y-axis")]
                            if cs.0.contains(CoordSel::Y) {
                                stepper_enable_flags.set(StepperChannel::Y, true);
                                stepper_dir_fwd_flags
                                    .set(StepperChannel::Y, cs.1.is_defined_positive());
                            }
                            #[cfg(feature = "with-z-axis")]
                            if cs.0.contains(CoordSel::Z) {
                                stepper_enable_flags.set(StepperChannel::Z, true);
                                stepper_dir_fwd_flags
                                    .set(StepperChannel::Z, cs.1.is_defined_positive());
                            }
                            #[cfg(feature = "with-e-axis")]
                            if cs.0.contains(CoordSel::E) {
                                stepper_enable_flags.set(StepperChannel::E, true);
                                stepper_dir_fwd_flags
                                    .set(StepperChannel::E, cs.1.is_defined_positive());
                            }
                        });
                        if steppers_off {
                            #[cfg(feature = "trace-commands")]
                            hwa::info!("[trace-commands] Powering steppers on");
                            unpark(&motion_planner, false).await;
                        }
                        steppers_off = false;

                        //#[cfg(feature = "verbose-timings")]
                        //let leap = global_timer.elapsed();

                        #[cfg(feature = "verbose-timings")]
                        hwa::debug!("\tCalculation elapsed: {} us", tx.elapsed().as_micros());

                        ////
                        //// MICRO-SEGMENTS INTERP START
                        ////
                        hwa::debug!("Segment interpolation START");

                        let mut prev_time = math::ZERO;
                        let mut p0 = math::ZERO;
                        // Micro-segments interpolation along segment
                        loop {
                            // Microsegment start

                            if event_bus.has_flags(EventFlags::DRY_RUN).await {
                                hwa::debug!("dry_run: Skipping move");
                                break;
                            }
                            hwa::trace!("Micro-segment START");

                            if let Some((estimated_position, _)) =
                                microsegment_iterator.next(micro_segment_real_time_rel)
                            {
                                let ds = estimated_position - p0;
                                let tprev = micro_segment_real_time_rel - prev_time;
                                let tmax = motion_profile.i7_end() - prev_time;
                                let _dt = tmax.min(tprev);

                                hwa::trace!(
                                    "at [{}] dt = {} ds = {} v = {}",
                                    micro_segment_real_time_rel.rdp(4),
                                    _dt.rdp(4),
                                    ds.rdp(4),
                                    (ds / _dt).rdp(4)
                                );

                                p0 = estimated_position;
                                let current_period_width_0 = if tprev < tmax {
                                    tprev
                                } else {
                                    if segment.segment_data.speed_exit_mms > math::ZERO {
                                        (ds / segment.segment_data.speed_exit_mms)
                                            .max(sampling_time)
                                    } else {
                                        tmax.max(sampling_time)
                                    }
                                };

                                let current_period_width = current_period_width_0; //(current_period_width_0 / sampling_time).ceil() * sampling_time;

                                hwa::trace!(
                                    "  current_period_width = {}",
                                    current_period_width.rdp(6)
                                );

                                prev_time += current_period_width;
                                micro_segment_real_time_rel += current_period_width;

                                let w = (current_period_width * math::ONE_MILLION).round();
                                let _has_more =
                                    microsegment_interpolator.advance_to(estimated_position, w);

                                cfg_if::cfg_if! {
                                    if #[cfg(feature="assert-motion")] {
                                        steps_to_advance += microsegment_interpolator.delta;
                                    }
                                }

                                #[cfg(feature = "verbose-timings")]
                                let t1 = embassy_time::Instant::now();
                                STEP_DRIVER
                                    .push(
                                        microsegment_interpolator.state().clone(),
                                        stepper_enable_flags,
                                        stepper_dir_fwd_flags,
                                    )
                                    .await;
                                cfg_if::cfg_if! {
                                    if #[cfg(feature="assert-motion")] {
                                        steps_advanced += STEP_DRIVER.flush().await;
                                    }
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
                        hwa::debug!("\t\t+Advanced: {}", microsegment_interpolator.advanced_mm());
                        hwa::debug!(
                            " + segment advanced: {}",
                            microsegment_interpolator.advanced_steps()
                        );

                        ////
                        //// MICRO-SEGMENTS INTERP END
                        ////
                        hwa::debug!("Micro-segment interpolation END");

                        cfg_if::cfg_if! {
                            if #[cfg(feature="assert-motion")] {
                                if steps_advanced != steps_to_advance {
                                    hwa::error!("Motion assertion failure. Advanced: {} Expected: {}", steps_advanced, steps_to_advance);
                                }
                            }
                        }

                        {
                            let adv_steps = microsegment_interpolator.advanced_steps();
                            if let Some(c) = segment.segment_data.unit_vector_dir.x {
                                if c.is_defined_positive() {
                                    real_steppper_pos.set_coord(
                                        CoordSel::X,
                                        Some(
                                            real_steppper_pos.x.unwrap()
                                                + adv_steps.x.unwrap().to_i32().unwrap(),
                                        ),
                                    );
                                } else {
                                    real_steppper_pos.set_coord(
                                        CoordSel::X,
                                        Some(
                                            real_steppper_pos.x.unwrap()
                                                - adv_steps.x.unwrap().to_i32().unwrap(),
                                        ),
                                    );
                                }
                            }
                            if let Some(c) = segment.segment_data.unit_vector_dir.y {
                                if c.is_defined_positive() {
                                    real_steppper_pos.set_coord(
                                        CoordSel::Y,
                                        Some(
                                            real_steppper_pos.y.unwrap()
                                                + adv_steps.y.unwrap().to_i32().unwrap(),
                                        ),
                                    );
                                } else {
                                    real_steppper_pos.set_coord(
                                        CoordSel::Y,
                                        Some(
                                            real_steppper_pos.y.unwrap()
                                                - adv_steps.y.unwrap().to_i32().unwrap(),
                                        ),
                                    );
                                }
                            }
                            if let Some(c) = segment.segment_data.unit_vector_dir.z {
                                if c.is_defined_positive() {
                                    real_steppper_pos.set_coord(
                                        CoordSel::Z,
                                        Some(
                                            real_steppper_pos.z.unwrap()
                                                + adv_steps.z.unwrap().to_i32().unwrap(),
                                        ),
                                    );
                                } else {
                                    real_steppper_pos.set_coord(
                                        CoordSel::Z,
                                        Some(
                                            real_steppper_pos.z.unwrap()
                                                - adv_steps.z.unwrap().to_i32().unwrap(),
                                        ),
                                    );
                                }
                            }
                        }

                        hwa::debug!(" + POS: {}", real_steppper_pos);
                        let moves_left = motion_planner
                            .consume_current_segment_data(&event_bus)
                            .await;
                        motion_planner
                            .defer_channel
                            .send(DeferEvent::Completed(DeferAction::LinearMove, channel))
                            .await;
                        event_bus
                            .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                            .await;

                        cfg_if::cfg_if! {
                            if #[cfg(feature="verbose-timings")] {
                                use crate::control::motion::profile::MotionProfile;

                                //global_timer = embassy_time::Instant::now();
                                hwa::info!("\t[v_0 = {}, v_lim = {}, v_1 = {}, t = {} d = {} mm = {} stp = {}]; {} moves left",
                                    segment.segment_data.speed_enter_mms.rdp(3).inner(),
                                    motion_profile.v_lim.rdp(3).inner(),
                                    segment.segment_data.speed_exit_mms.rdp(3).inner(),
                                    motion_profile.end_time(),
                                    microsegment_interpolator.advanced_mm(),
                                    microsegment_interpolator.advanced_mm().norm2().unwrap(),
                                    microsegment_interpolator.advanced_steps(),
                                    moves_left,
                                );
                                let t_tot = t0.elapsed().as_micros() as f32 / 1000000.0;
                                let t_qw = tq as f32 / 1000000.0;
                                hwa::debug!("SEGMENT END in {} s : {} - {}", t_tot - t_qw, t_tot, t_qw);
                            }
                        }
                        moves_left
                        // segment end
                    }
                    Err(_) => {
                        unreachable!("Unable to compute motion plan")
                    }
                }
            }
            Ok(ExecPlan::Dwell(sleep_ms, channel)) => {
                // Dwell
                if let Some(millis) = sleep_ms {
                    hwa::info!("G4: Sleeping {} millis", millis);
                    embassy_time::Timer::after(Duration::from_millis(millis.into())).await;
                } else {
                    hwa::info!("G4 Not sleeping");
                }
                let moves_left = motion_planner
                    .consume_current_segment_data(&event_bus)
                    .await;
                motion_planner
                    .defer_channel
                    .send(DeferEvent::Completed(DeferAction::Dwell, channel))
                    .await;
                event_bus
                    .publish_event(EventStatus::not_containing(EventFlags::MOVING))
                    .await;
                moves_left
            }
            Ok(ExecPlan::Homing(_channel)) => {
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
                            // Do nothing
                        }
                        else {
                            if !motion_planner.do_homing(&event_bus).await.is_ok() {
                                // TODO
                            }
                        }
                    }
                } else {
                    hwa::debug!("dry_run: Skipping homing");
                    break;
                }
                real_steppper_pos.set_coord(CoordSel::all(), Some(0));
                let moves_left = motion_planner
                    .consume_current_segment_data(&event_bus)
                    .await;
                hwa::debug!("Homing done");
                moves_left
            }
        };
        hwa::debug!("Moves left: {}", _moves_left);
    }
}

async fn park(motion_planner: &hwa::controllers::MotionPlanner) {
    hwa::warn!("Stepping parked");
    STEP_DRIVER.flush().await;
    motion_planner
        .motion_driver
        .lock()
        .await
        .disable_steppers(StepperChannel::all());
    STEP_DRIVER.reset();
    hwa::pause_ticker();
}

async fn unpark(motion_planner: &hwa::controllers::MotionPlanner, enable_steppers: bool) {
    hwa::resume_ticker();
    if enable_steppers {
        STEP_DRIVER.flush().await;
        motion_planner
            .motion_driver
            .lock()
            .await
            .enable_steppers(StepperChannel::all());
        STEP_DRIVER.reset();
    }
    #[cfg(feature = "trace-commands")]
    hwa::info!("[trace-commands] [task_stepper] Stepping resumed");
}
