//! Interpolator Step algorithm and proper (high prio) task.
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
//!     - Notify segment as completed
//!
//! TODO: This is a work still in progress
//!
//! TODO: Refactor pending
//!
//! Average Error Deviation in runtime: around 200us (Still pending to measure precisely)
use crate::control::motion_planning::StepperChannel;
#[allow(unused)]
use crate::math::{Real, ONE_MILLION, ONE_THOUSAND, ONE_HUNDRED};
#[allow(unused_imports)]
use crate::tgeo::{CoordSel, TVector};
#[allow(unused)]
use embassy_time::{Instant, Duration, with_timeout};
#[allow(unused_imports)]
use crate::hwa;
use printhor_hwa_common::{DeferAction, DeferEvent};
#[allow(unused)]
use printhor_hwa_common::{EventStatus, EventFlags};
#[cfg(feature = "timing-stats")]
use hwa::drivers::timing_stats::Timings;

use super::motion_timing::*;

/// Micro-segment sampling frequency in Hz
const MICRO_SEGMENT_PERIOD_HZ: u64 = 400;
/// Stepper pulse period in microseconds
const STEPPER_PULSE_WIDTH_US: Duration = Duration::from_micros(Duration::from_hz(embassy_time::TICK_HZ).as_micros());
const STEPPER_PULSE_WIDTH_TICKS: u32 = STEPPER_PULSE_WIDTH_US.as_ticks() as u32;

const DO_NOTHING: bool = false;

/// Inactivity Timeout until steppers are disabled
const STEPPER_INACTIVITY_TIMEOUT: Duration = Duration::from_secs(20);

/// Precomputed micro-segment period in microseconds
const MICRO_SEGMENT_PERIOD_US: u32 = Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ).as_micros() as u32;
/// Precomputed micro-segment period in ticks
const MICRO_SEGMENT_PERIOD_TICKS: u32 = Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ).as_ticks() as u32;
/// Precomputed micro-segment period in milliseconds
#[cfg(feature="verbose-timings")]
const PERIOD_MS: i32 = (MICRO_SEGMENT_PERIOD_US / 1000) as i32;

/// Precomputed default rate
const DEFAULT_RATE: u64 = (MICRO_SEGMENT_PERIOD_TICKS + MICRO_SEGMENT_PERIOD_TICKS) as u64;

/***
This task feeds watchdog to ensure no reset happen due high CPU starvation when feed rate is very high
 */
#[embassy_executor::task]
pub async fn task_stepper(
    motion_planner: hwa::controllers::MotionPlannerRef, _watchdog: hwa::WatchdogRef)
{
    let mut steppers_off = true;

    motion_planner.start().await;

    let mut s = motion_planner.event_bus.subscriber().await;

    hwa::info!("Micro-segment controller starting with {} us ({} ticks) micro-segment period and {} us step hold", MICRO_SEGMENT_PERIOD_US, MICRO_SEGMENT_PERIOD_TICKS, STEPPER_PULSE_WIDTH_US.as_micros());
    #[cfg(feature = "with-hotend")]
    hwa::info!("Extruder enabled");
    motion_planner.event_bus.publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)).await;

    cfg_if::cfg_if! {
        if #[cfg(any(feature="verbose-timings", feature="timing-stats"))] {
            let mut segment_id = 1;
        }
    }

    let mut global_timer = embassy_time::Instant::now();

    loop {
        let mut wait_for_sysalarm = false;
        if !s.get_status().await.contains(EventFlags::ATX_ON) {
            hwa::info!("task_stepper waiting for ATX_ON");
            if s.ft_wait_until(EventFlags::ATX_ON).await.is_err() {
                hwa::info!("Interrupted waiting for ATX_ON. SYS_ALARM?");
                wait_for_sysalarm = true;
            }
            else {
                hwa::info!("task_stepper got ATX_ON. Continuing.");
            }
        }
        if wait_for_sysalarm || s.get_status().await.contains(EventFlags::SYS_ALARM) {
            hwa::warn!("task stepper waiting for SYS_ALARM release");
            if s.ft_wait_while(EventFlags::SYS_ALARM).await.is_err() {
                panic!("Unexpected situation");
            }
        }
        match with_timeout(STEPPER_INACTIVITY_TIMEOUT, motion_planner.get_current_segment_data()).await {
            // Timeout
            Err(_) => {
                hwa::trace!("stepper_task timeout");
                if !steppers_off {
                    hwa::info!("Timeout. Powering steppers off");
                    let mut drv = motion_planner.motion_driver.lock().await;
                    drv.pins.disable_all_steppers();
                    steppers_off = true;
                }
            }
            // Process segment plan
            Ok(Some((segment, channel))) => {

                core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

                // segment metronome
                let absolute_ticker_period = Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS.into());
                let mut absolute_ticker_start = now();
                let t_segment_start = now();
                // Annotate how much time in ticks the executor is duty in this segment
                let mut duty = Duration::from_ticks(0);

                duty += t_segment_start.elapsed();

                motion_planner.event_bus.publish_event(EventStatus::containing(EventFlags::MOVING)).await;

                let neutral_element = segment.segment_data.vdir.map_val(Real::zero());

                let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
                    segment.segment_data.vdir.abs(),
                    segment.segment_data.displacement_mm,
                    (neutral_element + motion_planner.get_steps_per_mm_as_vector().await) * motion_planner.get_usteps_as_vector().await
                );

                let mut t_ref = t_segment_start;
                #[cfg(all(feature = "native", feature = "plot-timings"))]
                {
                    motion_planner.start_segment(t_segment_start, t_segment_start).await;
                }

                motion_planner.motion_driver.lock().await.enable_and_set_dir(&segment.segment_data.vdir);
                if steppers_off {
                    hwa::info!("\tPowering steppers on");
                }
                steppers_off = false;

                use hwa::controllers::motion_segment::SegmentIterator;

                // The reference time. Threathed as logic zero
                let t_segment_start = embassy_time::Instant::now();

                let leap = global_timer.elapsed();
                // Micro-segment realtime timer is sliced one period in the future
                // Also, compensate leap due response times with a max of half period
                let offset = Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS as u64) +
                    Duration::from_ticks(leap.as_ticks().min((MICRO_SEGMENT_PERIOD_TICKS >> 1) as u64));
                let mut microsegment_iterator = SegmentIterator::new(&segment.motion_profile, t_segment_start, offset);

                //hwa::debug!("Micro-segment interpolation START");
                // Micro-segments interpolation along segment
                loop {
                    if DO_NOTHING {
                        break;
                    }
                    //hwa::info!("\tMicro-segment START");

                    // Feed watchdog because this high prio task could cause CPU starvation
                    cfg_if::cfg_if! {
                        if #[cfg(feature="motion-async-task-preemptive")] {
                            _watchdog.lock().await.pet();
                        }
                    }

                    t_ref += Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS as u64);

                    if let Some(estimated_position) = microsegment_iterator.next(embassy_time::Instant::now()) {
                        hwa::trace!("p = {:?}", estimated_position);

                        // Microsegment logic
                        microsegment_interpolator.advance_to(estimated_position);

                        // The default rate is larger than a micro-segment period when there is not move in an axis, so no pulses are driven
                        cfg_if::cfg_if! {
                            if #[cfg(feature = "no-real-time")] {
                                microsegment_interpolator.set_microsegment_time(t_ref)
                            }
                        }

                        microsegment_interpolator.interpolate(&motion_planner.motion_driver).await;

                        if !microsegment_interpolator.bounded() {
                            //hwa::debug!("Segment completed");
                            break;
                        }

                        let tn = now();
                        let elapsed = tn.checked_duration_since(absolute_ticker_start).unwrap_or(Duration::from_ticks(0));
/////////////////////////
// PREEMPTION START
/////////////////////////
                        if absolute_ticker_period > elapsed {
                            let pend = absolute_ticker_period - elapsed;
                            absolute_ticker_start = tn + pend;
                            cfg_if::cfg_if! {
                                if #[cfg(feature="motion-async-task-preemptive")] {
                                    s_block_for(pend);
                                }
                                else {
                                    embassy_time::Timer::after(pend).await;
                                }
                            }
                        }
                        else {
                            absolute_ticker_start = tn;
                            hwa::debug!("\t\tuSegment lagging");
                        }
////////////////////////
// PREEMPTION END
////////////////////////
                        #[cfg(all(feature = "native", feature = "plot-timings"))]
                        motion_planner.mark_microsegment().await;
                    }
                    else { // No advance
                        // Reached end-of-segment, but still missing any step
                        break;
                    }
                    //hwa::trace!("\tMicro-segment END");
                    hwa::trace!("\t\t+Advanced: {}", microsegment_interpolator.advanced_mm());
                }
                //hwa::debug!("Micro-segment interpolation END");

                cfg_if::cfg_if! {
                    if #[cfg(feature="plot-timings")] {
                        motion_planner.end_segment().await;
                    }
                }
                cfg_if::cfg_if! {
                    if #[cfg(feature="verbose-timings")] {
                        let segment_time = t_segment_start.elapsed();
                    }
                }
                let _moves_left = motion_planner.consume_current_segment_data().await;
                motion_planner.defer_channel.send(DeferEvent::Completed(DeferAction::LinearMove, channel)).await;
                motion_planner.event_bus.publish_event(EventStatus::not_containing(EventFlags::MOVING)).await;
                let do_linger = _moves_left < 1 && segment.segment_data.speed_exit_mms.is_defined_positive();

                cfg_if::cfg_if! {
                    if #[cfg(feature="verbose-timings")] {
                        let segment_time_us = Real::from_lit(segment_time.as_micros().try_into().unwrap_or(0), 6).rdp(3);
                        hwa::info!("\tSEGMENT at +{} us, v_0 = {}, v_lim = {}, v_1 = {} |disp| = {} took: {} s adv: [{}] steps [{}] mm spd: [{}] mm/s",
                            leap.as_micros(),
                            segment.segment_data.speed_enter_mms.rdp(3).inner(),
                            segment.motion_profile.v_lim.rdp(3).inner(),
                            segment.segment_data.speed_exit_mms.rdp(3).inner(),
                            segment.segment_data.displacement_mm.rdp(3).inner(),
                            segment_time,
                            microsegment_interpolator.advanced_steps(),
                            microsegment_interpolator.advanced_mm(),
                            microsegment_interpolator.advanced_mm() / segment_time,
                        );
                    }
                }
                hwa::info!("{} moves ahead, {} leap", _moves_left, leap);
                if do_linger {
                    hwa::info!("linger");
                    embassy_time::Timer::after(Duration::from_micros(500)).await;
                }
                global_timer = embassy_time::Instant::now();
            }
            // Homing
            Ok(None) => {
                hwa::debug!("Homing init");
                motion_planner.motion_driver.lock().await.pins.enable_all_steppers();
                if steppers_off {
                    hwa::info!("\tPowering steppers on");
                }
                steppers_off = false;
                if !motion_planner.do_homing().await.is_ok() {
                    // TODO
                }
                motion_planner.consume_current_segment_data().await;
                hwa::debug!("Homing done");
            }
        }
    }
}

pub struct LinearMicrosegmentStepInterpolator {
    vdir_abs: TVector<Real>,
    usteps_per_mm: TVector<Real>,

    /// Number of discrete steps to advance in total
    usteps_to_advance: TVector<u32>,
    /// Number of discrete steps already advanced
    usteps_advanced: TVector<u32>,

    /// The number of discrete steps already advanced
    axis_steps_advanced_precise: TVector<Real>,

    multi_timer: MultiTimer,

    #[cfg(feature = "no-real-time")]
    t_micro_segment: embassy_time::Instant,
}

impl LinearMicrosegmentStepInterpolator {

    fn new(vdir_abs: TVector<Real>, displacement_module: Real, usteps_per_mm: TVector<Real>) -> Self {

        let usteps_to_advance: TVector<u32> = (
            (vdir_abs * displacement_module) * usteps_per_mm
        ).round().map_coords(|c| { Some(c.to_i32().unwrap_or(0) as u32) });

        Self {
            vdir_abs,
            usteps_per_mm,
            usteps_to_advance,
            usteps_advanced: TVector::zero(),
            axis_steps_advanced_precise: TVector::zero(),
            multi_timer: MultiTimer::new(),
            #[cfg(feature = "no-real-time")]
            t_micro_segment: embassy_time::Instant::now(),
        }
    }

    fn advance_to(&mut self, estimated_position: Real) {

        let axial_pos: TVector<Real> = self.vdir_abs * estimated_position;
        let step_pos: TVector<Real> = axial_pos * self.usteps_per_mm;
        let steps_to_advance_precise: TVector<Real> = (step_pos - self.axis_steps_advanced_precise).round();
        self.axis_steps_advanced_precise += steps_to_advance_precise;

        let tick_period_by_axis_rounded: TVector<Real> = (steps_to_advance_precise
            .map_val(Real::from_lit((MICRO_SEGMENT_PERIOD_TICKS - STEPPER_PULSE_WIDTH_TICKS) as i64, 0)) / steps_to_advance_precise
        ).round();

        let ticks = tick_period_by_axis_rounded.map_coords(|cv|
            cv.to_i64().and_then(|v| Some(v as u64))
        );

        self.multi_timer.set_channel_ticks( StepperChannel::X, ticks.x.unwrap_or(DEFAULT_RATE));
        self.multi_timer.set_channel_ticks( StepperChannel::Y, ticks.y.unwrap_or(DEFAULT_RATE));
        self.multi_timer.set_channel_ticks( StepperChannel::Z, ticks.z.unwrap_or(DEFAULT_RATE));
        #[cfg(feature = "with-hot-end")]
        self.multi_timer.set_channel_ticks( StepperChannel::E, ticks.e.unwrap_or(DEFAULT_RATE));
        self.multi_timer.reset(MICRO_SEGMENT_PERIOD_TICKS as u64);
    }

    #[inline]
    fn advanced_steps(&self) -> TVector<u32> {
        self.usteps_advanced
    }

    #[allow(unused)]
    fn advanced_mm(&self) -> TVector<Real> {
        self.advanced_steps().map_coords(|c| Some(Real::from_lit(c.into(), 0))) / self.usteps_per_mm
    }

    fn bounded(&self) -> bool {
        // eof && usteps_advanced.is_nan_or_zero()) || !usteps_advanced.bounded_by(&usteps_to_advance)
        true
    }

    #[cfg(feature = "no-real-time")]
    fn set_microsegment_time(&mut self, t_tick: embassy_time::Instant) {
        self.t_micro_segment = t_tick;
    }

    async fn interpolate(&mut self, driver: &embassy_sync::mutex::Mutex<hwa::ControllerMutexType, hwa::drivers::MotionDriver>) {

        let mut drv = driver.lock().await;
        loop {
            match self.multi_timer.next() {
                None => {
                    break;
                },
                Some((channel, _delay)) => {
                    if !self.usteps_advanced.bounded_by(&self.usteps_to_advance) {
                        // Prevent infinite loop when reaching the limits and delay is 0
                        break;
                    }

                    #[cfg(all(feature = "native", feature = "plot-timings", feature = "no-real-time"))]
                    {
                        self.t_micro_segment += Duration::from_ticks(_delay.as_ticks());
                        drv.update_clock(self.t_micro_segment);
                    }

                    //drv.laser_controller.lock().await.set_power(1.0f32).await;
                    if channel.contains(StepperChannel::X) {
                        self.usteps_advanced.increment(CoordSel::X, 1u32);
                        drv.x_step_pin_high();
                    }
                    if channel.contains(StepperChannel::Y) {
                        self.usteps_advanced.increment(CoordSel::Y, 1u32);
                        drv.y_step_pin_high();
                    }
                    if channel.contains(StepperChannel::Z) {
                        self.usteps_advanced.increment(CoordSel::Z, 1u32);
                        drv.z_step_pin_high();
                    }
                    #[cfg(feature = "with-hot-end")]
                    if channel.contains(StepperChannel::E) {
                        self.usteps_advanced.increment(CoordSel::E, 1u32);
                        drv.e_step_pin_high();
                    }

                    #[cfg(all(feature = "native", feature = "plot-timings", feature = "no-real-time"))]
                    {
                        self.t_micro_segment += STEPPER_PULSE_WIDTH_US;
                        drv.update_clock(self.t_micro_segment);
                    }
                    s_block_for(STEPPER_PULSE_WIDTH_US);
                    if channel.contains(StepperChannel::X) {
                        drv.x_step_pin_low();
                    }
                    if channel.contains(StepperChannel::Y) {
                        drv.y_step_pin_low();
                    }
                    if channel.contains(StepperChannel::Z) {
                        drv.z_step_pin_low();
                    }
                    #[cfg(feature = "with-hot-end")]
                    if channel.contains(StepperChannel::E) {
                        drv.e_step_pin_low();
                    }
                    #[cfg(feature = "no-real-time")]
                    {
                        self.multi_timer.sync_clock(STEPPER_PULSE_WIDTH_US);
                    }
                },
            }
        }
    }
}


