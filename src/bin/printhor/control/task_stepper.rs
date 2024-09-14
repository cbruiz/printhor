//! Interpolator Step algorithm.
//!
//! The IS algorithm works the following way:
//! - Try to retrieve an execution-ready motion segment from the motion queue
//!   - If no motion segment is present within [`STEPPER_INACTIVITY_TIMEOUT`], disable all steppers
//!   - If motion segment is execution-ready, dequeue it and then:
//!     - Enable all steppers
//!     - Evaluate the motion profile displacement at [`MICRO_SEGMENT_PERIOD_HZ`]
//!     - Compute number of steps to do in each axis (independently)
//!     - Compute pulse rate across each axis (independently) and construct an iterator leveraging [`MultiTimer`]
//!     - Consume a micro-segment until iterator is exhausted
//!
//! TODO: This is a work still in progress
use crate::control::motion::SCurveMotionProfile;
use crate::hwa;
use crate::math;
use crate::math::Real;
use crate::tgeo::{CoordSel, TVector};
use num_traits::ToPrimitive;
use hwa::controllers::motion::SegmentIterator;
use printhor_hwa_common::{DeferAction, DeferEvent};
use printhor_hwa_common::{EventBusRef, StepperChannel};
use printhor_hwa_common::{EventFlags, EventStatus};
use crate::hwa::controllers::LinearMicrosegmentStepInterpolator;
use crate::hwa::controllers::motion::STEP_DRIVER;

const DO_NOTHING: bool = false;

/// Inactivity Timeout until steppers are disabled
const STEPPER_INACTIVITY_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(5);


///
/// This constant defines the micro-segment period for the stepper planner in microseconds.
///
/// The period represents the duration of each micro-segment in the stepper motor control algorithm, 
/// calculated based on the `STEPPER_PLANNER_MICROSEGMENT_FREQUENCY`. It is used to break down motion 
/// profiles into smaller time intervals to achieve precise control of stepper motors.
///
/// # Calculation
/// This constant value is computed by dividing 1,000,000 microseconds (which is equivalent to 1 second)
/// by the `STEPPER_PLANNER_MICROSEGMENT_FREQUENCY`.
///
/// # Usage
/// The constant is used in the stepper task to determine the sampling period for evaluating motion 
/// profiles and generating stepping pulses.
///
/// # Note
/// Ensure that the `STEPPER_PLANNER_MICROSEGMENT_FREQUENCY` is correctly set to match the desired motion 
/// control frequency.
///
const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
    1_000_000 / hwa::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;

/**
This constant defines the micro-segment period for the stepper planner in microseconds.

It represents the duration of each micro-segment in the stepper motor control algorithm, which is 
calculated based on the `STEPPER_PLANNER_MICROSEGMENT_FREQUENCY`. A micro-segment is a small 
time interval used to break down motion profiles for precise control of stepper motors.

# Calculation
The constant value is computed by dividing 1,000,000 microseconds (equivalent to 1 second) by 
the `STEPPER_PLANNER_MICROSEGMENT_FREQUENCY`.

# Usage
This constant is used in the stepper task to determine the sampling period for evaluating 
motion profiles and generating stepping pulses.

# Note
Ensure the `STEPPER_PLANNER_MICROSEGMENT_FREQUENCY` is set correctly to match the desired motion 
control frequency.
*/
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
    event_bus: EventBusRef,
    motion_planner: hwa::controllers::MotionPlannerRef,
    _watchdog: hwa::WatchdogRef,
) -> ! {
    let mut steppers_off = true;

    let mut real_steppper_pos: TVector<i32> = TVector::zero();

    let micro_segment_period_secs: Real =
        Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 6);
    let sampling_time: Real = Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US as i64, 6);

    motion_planner.start(&event_bus).await;

    STEP_DRIVER.setup(motion_planner.motion_driver());

    let mut s = event_bus.subscriber().await;

    hwa::info!(
        "Micro-segment controller starting with {} us micro-segment period at {} us resolution",
        STEPPER_PLANNER_MICROSEGMENT_PERIOD_US,
        STEPPER_PLANNER_CLOCK_PERIOD_US
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
        let mut wait_for_sysalarm = false;
        if !s.get_status().await.contains(EventFlags::ATX_ON) {
            hwa::info!("task_stepper waiting for ATX_ON");
            // For safely, disable steppers
            // TODO: Park
            STEP_DRIVER.flush().await;
            motion_planner
                .motion_driver
                .lock()
                .await
                .disable_steppers(StepperChannel::all());
            STEP_DRIVER.reset();
            steppers_off = true;

            if s.ft_wait_until(EventFlags::ATX_ON).await.is_err() {
                hwa::info!("Interrupted waiting for ATX_ON. SYS_ALARM?");
                wait_for_sysalarm = true;
            } else {
                hwa::info!("task_stepper got ATX_ON. Continuing.");
            }
        }
        if wait_for_sysalarm || s.get_status().await.contains(EventFlags::SYS_ALARM) {
            hwa::warn!("task stepper waiting for SYS_ALARM release");
            // TODO: Park
            STEP_DRIVER.flush().await;
            motion_planner
                .motion_driver
                .lock()
                .await
                .disable_steppers(StepperChannel::all());
            STEP_DRIVER.reset();
            steppers_off = true;
            if s.ft_wait_while(EventFlags::SYS_ALARM).await.is_err() {
                panic!("Unexpected situation");
            }
        }
        match embassy_time::with_timeout(
            STEPPER_INACTIVITY_TIMEOUT,
            motion_planner.get_current_segment_data(&event_bus),
        )
            .await
        {
            // Timeout
            Err(_) => {
                hwa::trace!("stepper_task timeout");
                if !steppers_off {
                    hwa::info!("Timeout. Powering steppers off");
                    STEP_DRIVER.flush().await;
                    motion_planner
                        .motion_driver
                        .lock()
                        .await
                        .disable_steppers(StepperChannel::all());
                    STEP_DRIVER.reset();
                    steppers_off = true;
                }
            }
            // Process segment plan
            Ok(Some((segment, channel))) => {
                #[cfg(feature = "verbose-timings")]
                let t0 = embassy_time::Instant::now();
                #[cfg(feature = "verbose-timings")]
                let mut tq = 0;

                hwa::debug!("SEGMENT START");

                #[cfg(feature = "verbose-timings")]
                let tx = embassy_time::Instant::now();
                event_bus
                    .publish_event(EventStatus::containing(EventFlags::MOVING))
                    .await;

                // Vector helper to filter out irrelevant axes
                let neutral_element = segment.segment_data.vdir.map_val(&math::ZERO);
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
                            match motion_cfg.try_lock() {
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
                            segment.segment_data.vdir.abs(),
                            segment.segment_data.displacement_mm,
                            steps_per_mm,
                        );

                        // Prepare enable and dir flags
                        let mut stepper_enable_flags = StepperChannel::empty();
                        let mut stepper_dir_fwd_flags = StepperChannel::empty();
                        segment.segment_data.vdir.apply_coords(|cs| {
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
                            hwa::info!("\tPowering steppers on");
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

                            if DO_NOTHING {
                                break;
                            }
                            hwa::debug!("Micro-segment START");

                            if let Some((estimated_position, _)) =
                                microsegment_iterator.next(micro_segment_real_time_rel)
                            {
                                let ds = estimated_position - p0;
                                let tprev = micro_segment_real_time_rel - prev_time;
                                let tmax = motion_profile.i7_end() - prev_time;
                                let dt = tmax.min(tprev);

                                hwa::trace!(
                                    "at [{}] dt = {} ds = {} v = {}",
                                    micro_segment_real_time_rel.rdp(4),
                                    dt.rdp(4),
                                    ds.rdp(4),
                                    (ds / dt).rdp(4)
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
                                STEP_DRIVER.push(
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
                            if let Some(c) = segment.segment_data.vdir.x {
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
                            if let Some(c) = segment.segment_data.vdir.y {
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
                            if let Some(c) = segment.segment_data.vdir.z {
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

                        hwa::info!(" + POS: {}", real_steppper_pos);
                        let _moves_left = motion_planner
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
                                    _moves_left,
                                );
                                let t_tot = t0.elapsed().as_micros() as f32 / 1000000.0;
                                let t_qw = tq as f32 / 1000000.0;
                                hwa::debug!("SEGMENT END in {} s : {} - {}", t_tot - t_qw, t_tot, t_qw);
                            }
                        }
                        // segment end
                    }
                    Err(_) => {
                        unreachable!("Unable to compute motion plan")
                    }
                }
            }
            // Homing
            Ok(None) => {
                hwa::debug!("Homing init");

                STEP_DRIVER.flush().await;
                motion_planner
                    .motion_driver
                    .lock()
                    .await
                    .enable_steppers(StepperChannel::all());
                STEP_DRIVER.reset();

                if steppers_off {
                    hwa::info!("\tPowering steppers on");
                }
                steppers_off = false;
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
                motion_planner
                    .consume_current_segment_data(&event_bus)
                    .await;
                real_steppper_pos.set_coord(CoordSel::all(), Some(0));
                hwa::debug!("Homing done");
            }
        }
    }
}
