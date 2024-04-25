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
use embassy_sync::semaphore::{GreedySemaphore, Semaphore};
use crate::control::motion_planning::{StepperChannel};
#[allow(unused)]
use crate::math::{Real, ONE_MILLION, ONE_THOUSAND, ONE_HUNDRED};
#[allow(unused_imports)]
use crate::tgeo::{CoordSel, TVector};
#[allow(unused_imports)]
use crate::hwa;
use printhor_hwa_common::{ControllerMutexType, DeferAction, DeferEvent};
#[allow(unused)]
use printhor_hwa_common::{EventStatus, EventFlags};
use hwa::controllers::motion_segment::SegmentIterator;
use crate::control::motion_planning::SCurveMotionProfile;

use super::motion_timing::*;

/// Micro-segment sampling frequency in Hz
const MICRO_SEGMENT_PERIOD_HZ: u64 = 200;
/// Stepper pulse period in microseconds
const STEPPER_PULSE_WIDTH_US: embassy_time::Duration = embassy_time::Duration::from_micros(1);

const DO_NOTHING: bool = false;

/// Inactivity Timeout until steppers are disabled
const STEPPER_INACTIVITY_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(5);

/// Precomputed micro-segment period in microseconds
const MICRO_SEGMENT_PERIOD_US: u32 = embassy_time::Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ).as_micros() as u32;
/// Precomputed micro-segment period in ticks
const MICRO_SEGMENT_PERIOD_TICKS: u32 = embassy_time::Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ).as_ticks() as u32;

/***
This task feeds watchdog to ensure no reset happen due high CPU starvation when feed rate is very high
 */
#[embassy_executor::task]
pub async fn task_stepper(
    motion_planner: hwa::controllers::MotionPlannerRef, _watchdog: hwa::WatchdogRef,
    task_master: TaskMaster,
) -> ! {
    let mut steppers_off = true;
    //let mut position_deviation: TVector<Real> = TVector::zero();

    motion_planner.start().await;

    let mut s = motion_planner.event_bus.subscriber().await;
    let u_segment_time: Real = Real::from_lit(embassy_time::Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ).as_micros() as i64,  6);


    hwa::info!("Micro-segment controller starting with {} us ({} ticks) micro-segment period and {} us step hold", MICRO_SEGMENT_PERIOD_US, MICRO_SEGMENT_PERIOD_TICKS, STEPPER_PULSE_WIDTH_US.as_micros());

    #[cfg(feature = "with-hotend")]
    hwa::info!("Extruder enabled");

    motion_planner.event_bus.publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)).await;

    cfg_if::cfg_if! {
        if #[cfg(feature="verbose-timings")] {
            let mut global_timer = embassy_time::Instant::now();
        }
    }

    loop {
        let mut wait_for_sysalarm = false;
        if !s.get_status().await.contains(EventFlags::ATX_ON) {
            task_master.semaphore.set(0);
            hwa::info!("task_stepper waiting for ATX_ON");
            // For safely, disable steppers
            cfg_if::cfg_if! {
                if #[cfg(not(feature="digital-probe-hack"))] {
                    motion_planner.motion_driver.lock().await.pins.disable_all_steppers();
                }
            }
            steppers_off = true;

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
            task_master.semaphore.set(0);
            cfg_if::cfg_if! {
                if #[cfg(not(feature="digital-probe-hack"))] {
                    motion_planner.motion_driver.lock().await.pins.disable_all_steppers();
                }
            }
            steppers_off = true;
            if s.ft_wait_while(EventFlags::SYS_ALARM).await.is_err() {
                panic!("Unexpected situation");
            }
        }
        match embassy_time::with_timeout(STEPPER_INACTIVITY_TIMEOUT, motion_planner.get_current_segment_data()).await {
            // Timeout
            Err(_) => {
                hwa::trace!("stepper_task timeout");
                task_master.semaphore.set(0);
                if !steppers_off {
                    hwa::info!("Timeout. Powering steppers off");
                    cfg_if::cfg_if! {
                        if #[cfg(not(feature="digital-probe-hack"))] {
                            motion_planner.motion_driver.lock().await.pins.disable_all_steppers();
                        }
                    }
                    steppers_off = true;
                }
            }
            // Process segment plan
            Ok(Some((segment, channel))) => {

                #[cfg(feature = "verbose-timings")]
                let tx = embassy_time::Instant::now();
                motion_planner.event_bus.publish_event(EventStatus::containing(EventFlags::MOVING)).await;

                let neutral_element = segment.segment_data.vdir.map_val(Real::zero());

                hwa::debug!("S {} * {}  v0={} v1={}",
                    segment.segment_data.vdir,
                    segment.segment_data.displacement_mm,
                    segment.segment_data.speed_enter_mms,
                    segment.segment_data.speed_exit_mms
                );

                let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
                    segment.segment_data.vdir.abs(),
                    (neutral_element + motion_planner.get_steps_per_mm_as_vector().await) * motion_planner.get_usteps_as_vector().await
                );

                let mut enabled = StepperChannel::empty();
                let mut dir_fwd = StepperChannel::ALL;
                segment.segment_data.vdir.apply_coords(|cs| {

                    if cs.0.contains(CoordSel::X) {
                        enabled.set(StepperChannel::X, true);
                        dir_fwd.set(StepperChannel::X, cs.1.is_defined_positive());
                    }
                    else if cs.0.contains(CoordSel::Y) {
                        enabled.set(StepperChannel::Y, true);
                        dir_fwd.set(StepperChannel::Y, cs.1.is_defined_positive());
                    }
                    else if cs.0.contains(CoordSel::Z) {
                        enabled.set(StepperChannel::Z, true);
                        dir_fwd.set(StepperChannel::Z, cs.1.is_defined_positive());
                    }
                    else {
                        #[cfg(feature = "with-hot-end")]
                        if cs.0.contains(CoordSel::E) {
                            enabled.set(StepperChannel::E, true);
                            dir_fwd.set(StepperChannel::E, cs.1.is_defined_positive());
                        }
                    }
                });

                if steppers_off {
                    hwa::info!("\tPowering steppers on");
                }
                steppers_off = false;

                task_master.sender.send(
                    TaskAction::PulseStart(TaskHeader {
                        enable: enabled,
                        dir_fwd,
                    })
                ).await;
                task_master.semaphore.set(1);

                #[cfg(feature = "verbose-timings")]
                let leap = global_timer.elapsed();

                let mut micro_segment_real_time_rel_pos = u_segment_time;

                let motion_profile = SCurveMotionProfile::compute(segment.segment_data.displacement_mm, segment.segment_data.speed_enter_mms, segment.segment_data.speed_exit_mms, &segment.segment_data.constraints, true).unwrap();
                let mut microsegment_iterator = SegmentIterator::new(&motion_profile, micro_segment_real_time_rel_pos);

                #[cfg(feature = "verbose-timings")]
                hwa::trace!("Calculation elapsed: {} us", tx.elapsed().as_micros());

                //// MICROSEGMENTS INTERP START
                hwa::debug!("Micro-segment interpolation START");

                // Micro-segments interpolation along segment

                cfg_if::cfg_if! {
                    if #[cfg(feature="verbose-timings")] {
                        let mut num_loops = 0;
                    }
                }

                loop {
                    micro_segment_real_time_rel_pos += u_segment_time;
                    // Microsegment start

                    cfg_if::cfg_if! {
                        if #[cfg(feature="verbose-timings")] {
                            num_loops += 1;
                        }
                    }
                    if DO_NOTHING {
                        break;
                    }
                    hwa::debug!("Micro-segment START");

                    if let Some(estimated_position) = microsegment_iterator.next(micro_segment_real_time_rel_pos) {

                        hwa::trace!("\tat t = {}: p = {}", micro_segment_real_time_rel_pos.rdp(3), estimated_position.rdp(3));

                        // Micro-segment logic
                        microsegment_interpolator.advance_to(estimated_position);
                        hwa::trace!("\t\t+Advanced: {} mm", microsegment_interpolator.advanced_mm());
                        hwa::trace!("\t\t+Advanced: {} steps", microsegment_interpolator.advanced_steps());

                        task_master.sender.send(
                            TaskAction::PulseTrain(TaskPayload {
                                state: microsegment_interpolator.state(),
                            })
                        ).await;
                    }
                    else { // No advance
                        break;
                    }
                    hwa::debug!("Micro-segment END");
                    // Microsegment end
                }
                // TODO: Big deviation on large segments
                hwa::debug!("\t\t+Advanced: {}", microsegment_interpolator.advanced_mm());
                hwa::debug!("segment advanced: {}", microsegment_interpolator.advanced_steps());

                task_master.sender.send(TaskAction::PulseEnd).await;
                /*
                match task_master.semaphore.acquire(1).await {
                    Ok(_o) => {
                        task_master.semaphore.release(1);
                    }
                    Err(_e) => {
                        panic!("acq err");
                    }
                }
                */

                ////
                //// MICROSEGMENTS INTERP END
                ////

                hwa::debug!("Micro-segment interpolation END");

                let _moves_left = motion_planner.consume_current_segment_data().await;
                motion_planner.defer_channel.send(DeferEvent::Completed(DeferAction::LinearMove, channel)).await;
                motion_planner.event_bus.publish_event(EventStatus::not_containing(EventFlags::MOVING)).await;

                //let adv_exp = segment.segment_data.vdir.abs() * segment.segment_data.displacement_mm;
                //let adv_real = microsegment_interpolator.advanced_mm();
                //position_deviation = position_deviation.apply(&(adv_exp - adv_real)).map_nan(&crate::math::ZERO);

                cfg_if::cfg_if! {
                    if #[cfg(feature="verbose-timings")] {
                        let segment_time_s = Real::from_lit(num_loops, 0) * u_segment_time;
                        let steps = microsegment_interpolator.advanced_steps().map_coords(|c| Some(Real::from_lit(c as i64, 0)) );
                        hwa::info!("\tSEGMENT at +{} us, v_0 = {}, v_lim = {}, v_1 = {}\n\t|disp| = {} took: {} s adv: [{}] steps [{}] mm spd: [{}] mm/s [{}] steps/s, {} loops",
                            leap.as_micros(),
                            segment.segment_data.speed_enter_mms.rdp(3).inner(),
                            motion_profile.v_lim.rdp(3).inner(),
                            segment.segment_data.speed_exit_mms.rdp(3).inner(),
                            segment.segment_data.displacement_mm.rdp(3).inner(),
                            segment_time_s,
                            microsegment_interpolator.advanced_steps(),
                            microsegment_interpolator.advanced_mm().rdp(2),
                            (microsegment_interpolator.advanced_mm() / segment_time_s).rdp(1),
                            (steps / segment_time_s).rdp(1),
                            num_loops,
                        );
                        hwa::info!("{} moves ahead, {} us leap", _moves_left, leap.as_micros());
                        //hwa::debug!("mm adv: expected: [{}] real: [{}] diff: [{}]", adv_exp, adv_real, position_deviation);
                        //hwa::debug!("st adv: expected: [{}] real: [{}]", adv_exp * microsegment_interpolator.usteps_per_mm, microsegment_interpolator.advanced_steps());

                    }
                    else {
                        hwa::debug!("\tSEGMENT v_0 = {}, v_lim = {}, v_1 = {}",
                            segment.segment_data.speed_enter_mms.rdp(3).inner(),
                            motion_profile.v_lim.rdp(3).inner(),
                            segment.segment_data.speed_exit_mms.rdp(3).inner()
                        );
                        hwa::debug!("{} moves ahead", _moves_left);
                    }
                }

                cfg_if::cfg_if! {
                    if #[cfg(feature="verbose-timings")] {
                        global_timer = embassy_time::Instant::now();
                    }
                }

                // segment end
            }
            // Homing
            Ok(None) => {
                hwa::debug!("Homing init");
                motion_planner.motion_driver.lock().await.pins.enable_all_steppers();
                if steppers_off {
                    hwa::info!("\tPowering steppers on");
                }
                steppers_off = false;
                cfg_if::cfg_if!{
                    if #[cfg(feature="debug-skip-homing")] {
                        // Do nothing
                    }
                    else {
                        if !motion_planner.do_homing().await.is_ok() {
                            // TODO
                        }
                    }
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

    /// Number of discrete steps along vector already advanced
    usteps_advanced: TVector<u32>,

    /// The number of discrete steps by axis already advanced
    axis_steps_advanced_precise: TVector<Real>,

    multi_timer: MultiTimer,
}

impl LinearMicrosegmentStepInterpolator {

    fn new(vdir_abs: TVector<Real>, usteps_per_mm: TVector<Real>) -> Self {

        Self {
            vdir_abs,
            usteps_per_mm,
            usteps_advanced: TVector::zero(),
            axis_steps_advanced_precise: TVector::zero(),
            multi_timer: MultiTimer::new(),
        }
    }

    fn advance_to(&mut self, estimated_position: Real) {
        let axial_pos: TVector<Real> = self.vdir_abs * estimated_position;
        let step_pos: TVector<Real> = axial_pos * self.usteps_per_mm;
        let steps_to_advance_precise: TVector<Real> = (step_pos - self.axis_steps_advanced_precise).round().clamp_min(TVector::zero());
        self.axis_steps_advanced_precise += steps_to_advance_precise;

        let tick_period_by_axis: TVector<u64> = (steps_to_advance_precise
            .map_val(Real::from_lit((MICRO_SEGMENT_PERIOD_TICKS) as i64, 0)) / (steps_to_advance_precise)
        ).round().map_coords(|cv|
            cv.to_i32().and_then(|c| Some(c as u64))
        );

        self.usteps_advanced += steps_to_advance_precise.map_coords(|cv|
            cv.to_i32().and_then(|c| Some(c as u32))
        );

        self.multi_timer.set_channel_ticks( StepperChannel::X, tick_period_by_axis.x);
        self.multi_timer.set_channel_ticks( StepperChannel::Y, tick_period_by_axis.y);
        self.multi_timer.set_channel_ticks( StepperChannel::Z, tick_period_by_axis.z);
        #[cfg(feature = "with-hot-end")]
        self.multi_timer.set_channel_ticks( StepperChannel::E, tick_period_by_axis.e);
    }

    fn state(&self) -> MultiTimer { self.multi_timer.clone() }

    #[inline]
    fn advanced_steps(&self) -> TVector<u32> {
        self.usteps_advanced
    }

    #[allow(unused)]
    fn advanced_mm(&self) -> TVector<Real> {
        self.advanced_steps().map_coords(|c| Some(Real::from_lit(c.into(), 0))) / self.usteps_per_mm
    }
}

pub struct TaskHeader {
    enable: StepperChannel,
    dir_fwd: StepperChannel,
}

pub struct TaskPayload {
    state: MultiTimer,
}


impl Default for TaskPayload {
    fn default() -> Self {
        TaskPayload {
            state: MultiTimer::new(),
        }
    }
}

pub enum TaskAction {
    PulseStart(TaskHeader),
    PulseTrain(TaskPayload),
    PulseEnd,
}

pub type TaskChannel = embassy_sync::channel::Channel<ControllerMutexType, TaskAction, 6>;
pub type TaskChannelReceiver = embassy_sync::channel::Receiver<'static, ControllerMutexType, TaskAction, 6>;
pub type TaskChannelSender = embassy_sync::channel::Sender<'static, ControllerMutexType, TaskAction, 6>;

pub type ParkingSemaphore = GreedySemaphore::<ControllerMutexType>;

pub struct TaskMaster {
    pub semaphore: &'static ParkingSemaphore,
    pub sender: TaskChannelSender,
}

pub struct TaskSlave {
    pub semaphore: &'static ParkingSemaphore,
    pub receiver: TaskChannelReceiver,
}

#[embassy_executor::task]
pub async fn task_stepper_worker(motion_planner: hwa::controllers::MotionPlannerRef, s: TaskSlave) -> !
{
    loop {
        let _ = s.semaphore.acquire(1).await;

        // The reference time. Chained between consecutive pulse trains
        let mut ref_time = embassy_time::Instant::now();

        // Controls when pulse train is consecutive
        let mut continuous_train = false;

        // The time spent on micro-segment
        let mut usegment_spent_time = embassy_time::Duration::from_ticks(0);
        let mut usteps_advanced = TVector::zero();

        let period = embassy_time::Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS.into());
        loop {
            match embassy_time::with_timeout(embassy_time::Duration::from_ticks((MICRO_SEGMENT_PERIOD_TICKS) as u64), s.receiver.receive()).await {
                #[allow(unused_assignments)]
                Err(_timeout) => {
                    if continuous_train {
                        hwa::debug!("Continuous train reset");
                        ref_time = embassy_time::Instant::now();
                        usegment_spent_time = embassy_time::Duration::from_ticks(0);
                        continuous_train = false;
                    }
                    break;
                }
                Ok(TaskAction::PulseStart(header)) => {

                    hwa::trace!("PulseStart at t={}, s={}",
                        ref_time.elapsed().as_micros(),
                        usegment_spent_time.as_micros());
                    usteps_advanced = TVector::zero();
                    let mut drv = motion_planner.motion_driver.lock().await;
                    drv.set_ena(header.enable);
                    drv.set_fwd(header.dir_fwd);
                }
                Ok(TaskAction::PulseEnd) => {
                    hwa::trace!("PulseEnd at {}. Advanced {} usteps",
                        ref_time.elapsed().as_micros(),
                        usteps_advanced
                    );
                }
                Ok(TaskAction::PulseTrain(mut u_segment)) => {
                    if !continuous_train {
                        ref_time = embassy_time::Instant::now();
                        usegment_spent_time = embassy_time::Duration::from_ticks(0);
                        continuous_train = true;
                    }

                    let mut drv = motion_planner.motion_driver.lock().await;

                    let mut awaited: embassy_time::Duration = embassy_time::Duration::from_ticks(0);
                    u_segment.state.reset(MICRO_SEGMENT_PERIOD_TICKS as u64);
                    //let t0 = embassy_time::Instant::now();
                    loop {
                        match u_segment.state.next() {
                            None => {
                                break;
                            },
                            Some((channel, _delay)) => {
                                let now = embassy_time::Instant::now();
                                // Compute times relatively to reference time to compensate deviations
                                let tick_instant = ref_time + usegment_spent_time + awaited + _delay;
                                if tick_instant < now {
                                    let deviation = now - tick_instant;
                                    //hwa::warn!("u-segment lagging {} us. ref_time displaced", deviation.as_micros() );
                                    // Displace reference time as it is not possible to time travel
                                    ref_time += deviation;
                                }
                                embassy_time::Timer::at(tick_instant).await;

                                awaited += _delay;

                                //drv.laser_controller.lock().await.set_power(1.0f32).await;
                                if channel.contains(StepperChannel::X) {
                                    usteps_advanced.increment(CoordSel::X, 1u32);
                                    drv.x_step_pin_high();
                                }
                                if channel.contains(StepperChannel::Y) {
                                    usteps_advanced.increment(CoordSel::Y, 1u32);
                                    drv.y_step_pin_high();
                                }
                                if channel.contains(StepperChannel::Z) {
                                    usteps_advanced.increment(CoordSel::Z, 1u32);
                                    drv.z_step_pin_high();
                                }
                                #[cfg(feature = "with-hot-end")]
                                if channel.contains(StepperChannel::E) {
                                    usteps_advanced.increment(CoordSel::E, 1u32);
                                    drv.e_step_pin_high();
                                }
                                embassy_time::Timer::after(STEPPER_PULSE_WIDTH_US).await;
                                // s_block_for(STEPPER_PULSE_WIDTH_US);
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
                            },
                        }

                    }
                    usegment_spent_time += period;
                    //hwa::debug!("Segment loop took {} us", t0.elapsed().as_micros())
                }
            }
        }
        s.semaphore.release(1);
    }
}
