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
//!
//! TODO: Refactor pending
//!
use core::cell::{RefCell};
use core::future::{Future, poll_fn};
use core::task::{Context, Poll};
use embassy_sync::waitqueue::WakerRegistration;
use crate::control::motion_planning::{StepperChannel};
#[allow(unused)]
use crate::math::{Real, ONE_MILLION, ONE_THOUSAND, ONE_HUNDRED};
#[allow(unused_imports)]
use crate::tgeo::{CoordSel, TVector};
#[allow(unused_imports)]
use crate::hwa;
use printhor_hwa_common::{DeferAction, DeferEvent};
#[allow(unused)]
use printhor_hwa_common::{EventStatus, EventFlags};
use hwa::controllers::motion_segment::SegmentIterator;
use crate::control::motion_planning::SCurveMotionProfile;
use crate::control::motion_planning::plan::MotionProfile;

use super::motion_timing::*;

const DO_NOTHING: bool = false;

/// Inactivity Timeout until steppers are disabled
const STEPPER_INACTIVITY_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(5);

const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 = embassy_time::Duration::from_hz(hwa::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY as u64).as_micros() as u32;
const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = embassy_time::Duration::from_hz(hwa::STEPPER_PLANNER_CLOCK_FREQUENCY as u64).as_micros() as u32;

// How many us there are in a microsegment period
const U_SEGMENT_US_BY_PERIOD: Real = Real::from_inner(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as f32);

const U_SEGMENT_TIME_SECS: Real = Real::from_inner(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as f32 / 1000000.0f32);

/***
This task feeds watchdog to ensure no reset happen due high CPU starvation when feed rate is very high
 */
#[embassy_executor::task]
pub async fn task_stepper(
    motion_planner: hwa::controllers::MotionPlannerRef, _watchdog: hwa::WatchdogRef,
) -> ! {
    let mut steppers_off = true;

    motion_planner.start().await;

    TM.setup(motion_planner.clone());

    let mut s = motion_planner.event_bus.subscriber().await;

    hwa::info!("Micro-segment controller starting with {} us micro-segment period at {} us resolution",
        STEPPER_PLANNER_MICROSEGMENT_PERIOD_US, STEPPER_PLANNER_CLOCK_PERIOD_US);

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
            hwa::info!("task_stepper waiting for ATX_ON");
            // For safely, disable steppers
            // TODO: Park
            motion_planner.motion_driver.lock().await.pins.disable_all_steppers();
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
            // TODO: Park
            motion_planner.motion_driver.lock().await.pins.disable_all_steppers();
            steppers_off = true;
            if s.ft_wait_while(EventFlags::SYS_ALARM).await.is_err() {
                panic!("Unexpected situation");
            }
        }
        match embassy_time::with_timeout(STEPPER_INACTIVITY_TIMEOUT, motion_planner.get_current_segment_data()).await {
            // Timeout
            Err(_) => {
                hwa::trace!("stepper_task timeout");
                if !steppers_off {
                    hwa::info!("Timeout. Powering steppers off");
                    motion_planner.motion_driver.lock().await.pins.disable_all_steppers();
                    steppers_off = true;
                }
            }
            // Process segment plan
            Ok(Some((segment, channel))) => {

                //#[cfg(feature = "verbose-timings")]
                let tx = embassy_time::Instant::now();
                motion_planner.event_bus.publish_event(EventStatus::containing(EventFlags::MOVING)).await;

                let neutral_element = segment.segment_data.vdir.map_val(Real::zero());

                hwa::trace!("S {} * {}  v0={} v1={}",
                    segment.segment_data.vdir,
                    segment.segment_data.displacement_mm,
                    segment.segment_data.speed_enter_mms,
                    segment.segment_data.speed_exit_mms
                );

                let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
                    segment.segment_data.vdir.abs(),
                    (neutral_element + motion_planner.get_steps_per_mm_as_vector().await) * motion_planner.get_usteps_as_vector().await
                );

                let mut stepper_enable_flags = StepperChannel::empty();
                let mut stepper_dir_fwd_flags = StepperChannel::ALL;
                segment.segment_data.vdir.apply_coords(|cs| {

                    if cs.0.contains(CoordSel::X) {
                        stepper_enable_flags.set(StepperChannel::X, true);
                        stepper_dir_fwd_flags.set(StepperChannel::X, cs.1.is_defined_positive());
                    }
                    else if cs.0.contains(CoordSel::Y) {
                        stepper_enable_flags.set(StepperChannel::Y, true);
                        stepper_dir_fwd_flags.set(StepperChannel::Y, cs.1.is_defined_positive());
                    }
                    else if cs.0.contains(CoordSel::Z) {
                        stepper_enable_flags.set(StepperChannel::Z, true);
                        stepper_dir_fwd_flags.set(StepperChannel::Z, cs.1.is_defined_positive());
                    }
                    else {
                        #[cfg(feature = "with-hot-end")]
                        if cs.0.contains(CoordSel::E) {
                            stepper_enable_flags.set(StepperChannel::E, true);
                            stepper_dir_fwd_flags.set(StepperChannel::E, cs.1.is_defined_positive());
                        }
                    }
                });

                if steppers_off {
                    hwa::info!("\tPowering steppers on");
                }
                steppers_off = false;

                #[cfg(feature = "verbose-timings")]
                let leap = global_timer.elapsed();

                let mut micro_segment_real_time_rel_pos = U_SEGMENT_TIME_SECS;

                let motion_profile = match SCurveMotionProfile::compute(segment.segment_data.displacement_mm, segment.segment_data.speed_enter_mms, segment.segment_data.speed_exit_mms, &segment.segment_data.constraints, true) {
                    Ok(_profile) => _profile,
                    _ => panic!("HODOR")
                };
                let mut microsegment_iterator = SegmentIterator::new(&motion_profile, math::ZERO);

                //#[cfg(feature = "verbose-timings")]
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
                        // TODO: push time lenght as well to premature complete leap segment if it is very short
                        TM.push(microsegment_interpolator.state().channels(),
                                microsegment_interpolator.state().max_count(),
                                stepper_enable_flags,
                                stepper_dir_fwd_flags,
                        ).await;
                    }
                    else { // No advance
                        break;
                    }
                    hwa::debug!("Micro-segment END");
                    micro_segment_real_time_rel_pos += U_SEGMENT_TIME_SECS;
                    // Microsegment end
                }
                hwa::debug!("\t\t+Advanced: {}", microsegment_interpolator.advanced_mm());
                hwa::debug!("segment advanced: {}", microsegment_interpolator.advanced_steps());

                ////
                //// MICROSEGMENTS INTERP END
                ////

                hwa::debug!("Micro-segment interpolation END");

                let _moves_left = motion_planner.consume_current_segment_data().await;
                motion_planner.defer_channel.send(DeferEvent::Completed(DeferAction::LinearMove, channel)).await;
                motion_planner.event_bus.publish_event(EventStatus::not_containing(EventFlags::MOVING)).await;

                hwa::info!("\tSEGMENT [v_0 = {}, v_lim = {}, v_1 = {}, t = {}]; {} moves left",
                    segment.segment_data.speed_enter_mms.rdp(3).inner(),
                    motion_profile.v_lim.rdp(3).inner(),
                    segment.segment_data.speed_exit_mms.rdp(3).inner(),
                    motion_profile.end(),
                    _moves_left
                );

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

        let tick_period_by_axis: TVector<u64> = (steps_to_advance_precise.map_val(U_SEGMENT_US_BY_PERIOD) / (steps_to_advance_precise)
        ).round().map_coords(|cv|
            cv.to_i32().and_then(|c| Some(c as u64))
        );

        let step_increment = steps_to_advance_precise.map_coords(|cv|
            cv.to_i32().and_then(|c| Some(c as u32))
        );

        self.usteps_advanced += step_increment;
        self.multi_timer.set_max_count(step_increment.vmax().unwrap_or(0));

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


use critical_section::Mutex as CsMutex;
use crate::hwa::controllers::MotionPlannerRef;
use crate::math;

const TIMER_QUEUE_SIZE: usize = 2;

struct SoftTimerDriver {
    current: StepPlanner,
    tdeque: embassy_time::Instant,
    state: State,
    tick_count: u32,
    pulse_count: u32,
    queue: [Option<StepPlanner>; TIMER_QUEUE_SIZE],
    head: usize,
    tail: usize,
    num_queued: usize,
    drv: Option<hwa::controllers::MotionPlannerRef>,
    waker: WakerRegistration,
    current_stepper_enable_flags: StepperChannel,
    current_stepper_dir_fwd_flags: StepperChannel,
}

#[derive(PartialEq)]
enum State {
    Idle,
    Duty,
}

impl SoftTimerDriver {

    fn on_interrupt(&mut self) {

        if self.state == State::Idle {
            if self.num_queued > 0 {
                if let Some(timer) = self.queue[self.head].take() {
                    self.current = timer;
                    self.tick_count = 0;
                    self.pulse_count = 0;
                    self.tdeque = embassy_time::Instant::now();
                    self.current.reset(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as u64);
                    match &self.drv {
                        None => {
                            unreachable!("hodor")
                        }
                        Some(_ref) => {
                            match _ref.motion_driver.try_lock() {
                                Ok(mut _drv) => {
                                    if self.current_stepper_enable_flags != self.current.stepper_enable_flags {
                                        _drv.set_ena(self.current.stepper_enable_flags);
                                        self.current_stepper_enable_flags = self.current.stepper_enable_flags;
                                    }
                                    if self.current_stepper_dir_fwd_flags != self.current.stepper_dir_fwd_flags {
                                        _drv.set_fwd(self.current.stepper_dir_fwd_flags);
                                        self.current_stepper_dir_fwd_flags = self.current.stepper_dir_fwd_flags;
                                    }
                                }
                                Err(_) => {
                                    unreachable!("hodor")
                                }
                            }
                        }
                    }
                    hwa::trace!("u-segment dequeued at {}", self.head);
                }
                else {
                    unreachable!("Unable to deque!!!")
                }
                self.state = State::Duty;
                self.head += 1;
                if self.head == self.queue.len() {
                    self.head = 0;
                }
                self.num_queued -= 1;
            }
            else {
                if self.num_queued < TIMER_QUEUE_SIZE {
                    self.waker.wake();
                }
                // Quickly return as there is no work to do
                return;
            }
        }

        self.tick_count += STEPPER_PLANNER_CLOCK_PERIOD_US;

        // In this point, state is either Duty or something were dequeued
        match self.current.next(STEPPER_PLANNER_CLOCK_PERIOD_US as u64) {
            None => {
                self.state = State::Idle;
                panic!("hodor");
            },
            Some(_ch) => {
                self.apply(_ch);
            }
        }

        if self.tick_count >= STEPPER_PLANNER_MICROSEGMENT_PERIOD_US {
            let te = self.tdeque.elapsed().as_micros();
            hwa::trace!("segment consumed with {} pulses in {} ticks taking {} us", self.pulse_count, self.tick_count, te);
            self.state = State::Idle;
            self.tick_count = 0;
            self.pulse_count = 0;
        }
        if self.num_queued < TIMER_QUEUE_SIZE {
            self.waker.wake();
        }
    }

    fn apply(&mut self, _channel: StepperChannel) -> bool {
        match &self.drv {
            None => {
                return false
            }
            Some(_ref) => {
                match _ref.motion_driver.try_lock() {
                    Ok(mut _drv) => {

                        if !_channel.is_empty() {
                            self.pulse_count += 1;
                        }

                        if _channel.contains(StepperChannel::X) {
                            //usteps_advanced.increment(CoordSel::X, 1u32);
                            #[cfg(feature = "pulsed")]
                            _drv.x_step_pin_high();
                            #[cfg(not(feature = "pulsed"))]
                            _drv.x_step_pin_toggle();
                        }
                        if _channel.contains(StepperChannel::Y) {
                            //usteps_advanced.increment(CoordSel::Y, 1u32);
                            #[cfg(feature = "pulsed")]
                            drv.y_step_pin_high();
                            #[cfg(not(feature = "pulsed"))]
                            _drv.y_step_pin_toggle();
                        }
                        if _channel.contains(StepperChannel::Z) {
                            //usteps_advanced.increment(CoordSel::Z, 1u32);
                            #[cfg(feature = "pulsed")]
                            _drv.z_step_pin_high();
                            #[cfg(not(feature = "pulsed"))]
                            _drv.z_step_pin_toggle();
                        }
                        #[cfg(feature = "with-hot-end")]
                        if _channel.contains(StepperChannel::E) {
                            //usteps_advanced.increment(CoordSel::E, 1u32);
                            #[cfg(feature = "pulsed")]
                            _drv.e_step_pin_high();
                            #[cfg(not(feature = "pulsed"))]
                            _drv.e_step_pin_toggle();
                        }
                        #[cfg(feature = "pulsed")]
                        {
                            embassy_time::Timer::after(STEPPER_PULSE_WIDTH_US).await;
                            //s_block_for(STEPPER_PULSE_WIDTH_US);
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
                        }
                        true
                    }
                    Err(_) => {
                        false
                    }
                }
            }
        }
    }
}

pub struct SoftTimer(CsMutex<RefCell<SoftTimerDriver>>);

impl SoftTimer {
    const fn new() -> Self {
        Self(CsMutex::new(RefCell::new(SoftTimerDriver{
            current: StepPlanner::new(),
            tdeque: embassy_time::Instant::from_ticks(0),
            state: State::Idle,
            queue: [None; 2],
            head: 0,
            tick_count: 0,
            pulse_count: 0,
            waker: WakerRegistration::new(),
            num_queued: 0,
            tail: 0,
            drv: None,
            current_stepper_enable_flags: StepperChannel::empty(),
            current_stepper_dir_fwd_flags: StepperChannel::empty(),
        })))
    }

    pub fn setup(&self, mp: MotionPlannerRef) {
        critical_section::with(|cs| {
             let mut r = self.0.borrow_ref_mut(cs);
             r.drv.replace(mp);
        })
     }

    pub fn push(&self, channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
                max_count: u32,
                stepper_enable_flags: StepperChannel,
                stepper_dir_fwd_flags: StepperChannel,
    ) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| self.poll_push(cx, channels, max_count, stepper_enable_flags, stepper_dir_fwd_flags))
    }

    fn poll_push(&self, cx: &mut Context<'_>, channels: [Option<ChannelStatus>; MULTITIMER_CHANNELS],
                 max_count: u32,
                 stepper_enable_flags: StepperChannel,
                 stepper_dir_fwd_flags: StepperChannel,
    ) -> Poll<()> {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            if r.num_queued >= r.queue.len() {
                r.waker.register(cx.waker());
                Poll::Pending
            }
            else {
                r.num_queued += 1;
                let current_tail = r.tail;
                hwa::trace!("u-segment queued at {}", current_tail);
                r.queue[current_tail] = Some(StepPlanner::from(channels, max_count, stepper_enable_flags, stepper_dir_fwd_flags));
                r.tail = if r.tail + 1  == r.queue.len() {
                    0
                }
                else {
                    r.tail + 1
                };
                Poll::Ready(())
            }
        })
    }

    #[allow(unused)]
    pub fn flush(&self) -> impl Future<Output = ()> + '_ {
        poll_fn(move |cx| self.poll_flush(cx))
    }

    fn poll_flush(&self, cx: &mut Context<'_>) -> Poll<()> {
        critical_section::with(|cs| {
            let mut r = self.0.borrow_ref_mut(cs);
            if r.num_queued > 0 {
                r.waker.register(cx.waker());
                Poll::Pending
            }
            else {
                Poll::Ready(())
            }
        })
    }
}

static TM: SoftTimer = SoftTimer::new();

#[no_mangle]
pub extern "Rust" fn do_tick() {
    let _t0 = embassy_time::Instant::now();
    critical_section::with(|cs| {
        let _t1 = embassy_time::Instant::now();
        let mut counter = TM.0.borrow_ref_mut(cs);
        counter.on_interrupt();
        let _te1 = _t1.elapsed();
        hwa::trace!("on_int took {}", _te1.as_micros());
    });
    let _te = _t0.elapsed();
    hwa::trace!("do_tick took {}", _te.as_micros());
}
