//! Interpolator Step Rate (ISR) algorithm and proper (high prio) task.
//!
//! The ISR algorithm works the following way:
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
//! Average Error Deviation in runtime: around 10us (Still pending to measure precisely)
use crate::control::planner::StepperChannel;
#[allow(unused)]
use crate::math::{Real, ONE_MILLION, ONE_THOUSAND};
#[allow(unused_imports)]
use crate::tgeo::{CoordSel, TVector};
#[allow(unused)]
use embassy_time::{Instant, Duration, with_timeout};
#[allow(unused_imports)]
use crate::{hwa, hwa::controllers::{DeferEvent, DeferType}};
#[allow(unused)]
use printhor_hwa_common::{EventStatus, EventFlags};
#[allow(unused_imports)]
use crate::math::ONE_HUNDRED;

use super::motion_timing::*;

/// Micro-segment sampling frequency in Hz
const MICRO_SEGMENT_PERIOD_HZ: u64 = 50;
/// Stepper pulse period in microseconds
const STEPPER_PULSE_WIDTH_US: Duration = Duration::from_micros(Duration::from_hz(embassy_time::TICK_HZ).as_micros());
/// Inactivity Timeout until steppers are disabled
const STEPPER_INACTIVITY_TIMEOUT: Duration = Duration::from_secs(20);

/// Precomputed micro-segment period in microseconds
const MICRO_SEGMENT_PERIOD_US: u32 = Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ).as_micros() as u32;
/// Precomputed micro-segment period in ticks
const MICRO_SEGMENT_PERIOD_TICKS: u32 = Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ).as_ticks() as u32;
/// Precomputed micro-segment period in milliseconds
const PERIOD_MS: i32 = (MICRO_SEGMENT_PERIOD_US / 1000) as i32;

struct Timings {
    // Segment reference time
    t_segment: Instant,
    // Micro-segment reference time
    t_u_segment: Instant,
    // Time to prepare a segment
    t_segment_preparation: Duration,
    t_segment_execution: Duration,
    // Aggregated computation for every micro-segment
    t_u_segment_computation_agg: Duration,
    // Aggregated stepping time for every micro-segment
    t_u_segment_stepping_agg: Duration,
    // Aggregated remaining time for every micro-segment
    t_u_segment_remaining_agg: Duration,
    // Min computation for every micro-segment
    t_u_segment_computation_min: Duration,
    // Min stepping time for every micro-segment
    t_u_segment_stepping_min: Duration,
    // Min remaining time for every micro-segment
    t_u_segment_remaining_min: Duration,
    // Max computation for every micro-segment
    t_u_segment_computation_max: Duration,
    // Max stepping time for every micro-segment
    t_u_segment_stepping_max: Duration,
    // Max remaining time for every micro-segment
    t_u_segment_remaining_max: Duration,
    t_segment_remaining: Duration,
}

impl Timings {
    fn new() -> Self {
        Self {
            t_segment: Instant::now(),
            t_u_segment: Instant::now(),
            t_segment_preparation: Duration::from_ticks(0),
            t_segment_execution: Duration::from_ticks(0),
            t_u_segment_computation_agg: Duration::from_ticks(0),
            t_u_segment_stepping_agg: Duration::from_ticks(0),
            t_u_segment_remaining_agg: Duration::from_ticks(0),
            t_u_segment_computation_min: Duration::from_ticks(999999999),
            t_u_segment_stepping_min: Duration::from_ticks(999999999),
            t_u_segment_remaining_min: Duration::from_ticks(999999999),
            t_u_segment_computation_max: Duration::from_ticks(0),
            t_u_segment_stepping_max: Duration::from_ticks(0),
            t_u_segment_remaining_max: Duration::from_ticks(0),
            t_segment_remaining: Duration::from_ticks(0),
        }
    }
    fn u_reset(&mut self) {
        self.t_u_segment = Instant::now();
    }

    fn set_prep(&mut self) {
        self.t_segment_preparation = self.t_segment.elapsed();
    }

    fn add_u_comp(&mut self) {
        let elapsed = self.t_u_segment.elapsed();
        self.t_u_segment_computation_min = self.t_u_segment_computation_min.min(elapsed);
        self.t_u_segment_computation_max = self.t_u_segment_computation_max.max(elapsed);
        self.t_u_segment_computation_agg += elapsed;
        self.u_reset();
    }

    fn add_u_stepping(&mut self) {
        let elapsed = self.t_u_segment.elapsed();
        self.t_u_segment_stepping_min = self.t_u_segment_stepping_min.min(elapsed);
        self.t_u_segment_stepping_max = self.t_u_segment_stepping_max.max(elapsed);
        self.t_u_segment_stepping_agg += elapsed;
        self.u_reset();
    }

    fn add_u_remaining(&mut self) {
        let elapsed = self.t_u_segment.elapsed();
        self.t_u_segment_remaining_min = self.t_u_segment_remaining_min.min(elapsed);
        self.t_u_segment_remaining_max = self.t_u_segment_remaining_max.max(elapsed);
        self.t_u_segment_remaining_agg += elapsed;
        self.u_reset();
    }

    fn set_execution(&mut self) {
        self.t_segment_execution = self.t_segment.elapsed();
    }

    fn set_remaining(&mut self) {
        self.t_segment_remaining = self.t_segment.elapsed() - self.t_segment_execution;
    }

    fn report(&self) {
        hwa::info!("Segment timings: {} + {} + {} = {}",
            self.t_segment_preparation.as_micros(),
            self.t_segment_execution.as_micros(),
            self.t_segment_remaining.as_micros(),
            (self.t_segment_preparation + self.t_segment_execution + self.t_segment_remaining).as_micros()
        );
        hwa::info!("\tMicro-segment timings: {} + {} + {} = {}",
            self.t_u_segment_computation_agg.as_micros(),
            self.t_u_segment_stepping_agg.as_micros(),
            self.t_u_segment_remaining_agg.as_micros(),
            (self.t_u_segment_computation_agg + self.t_u_segment_stepping_agg + self.t_u_segment_remaining_agg).as_micros()
        );
        hwa::info!("\tMicro-segment ranges: [{} .. {}] + [{} .. {}] + [{} .. {}] = [{} .. {}]",
            self.t_u_segment_computation_min.as_micros(),
            self.t_u_segment_computation_max.as_micros(),
            self.t_u_segment_stepping_min.as_micros(),
            self.t_u_segment_stepping_max.as_micros(),
            self.t_u_segment_remaining_min.as_micros(),
            self.t_u_segment_remaining_max.as_micros(),
            (self.t_u_segment_computation_min + self.t_u_segment_stepping_min + self.t_u_segment_remaining_min).as_micros(),
            (self.t_u_segment_computation_max + self.t_u_segment_stepping_max + self.t_u_segment_remaining_max).as_micros(),
        );
    }
}

/***
This task feeds watchdog to ensure no reset happen due high CPU starvation when feed rate is very high
 */
#[embassy_executor::task]
pub async fn stepper_task(
    motion_planner: hwa::controllers::MotionPlannerRef, watchdog: hwa::WatchdogRef)
{
    let mut steppers_off = true;

    motion_planner.start().await;

    let mut s = motion_planner.event_bus.subscriber().await;
    s.wait_while(EventFlags::HOMMING).await;

    hwa::info!("Micro-segment controller starting with {} us ({} ticks) micro-segment period and {} us step hold", MICRO_SEGMENT_PERIOD_US, MICRO_SEGMENT_PERIOD_TICKS, STEPPER_PULSE_WIDTH_US.as_micros());
    motion_planner.event_bus.publish_event(EventStatus::containing(EventFlags::MOV_QUEUE_EMPTY)).await;

    loop {
        match with_timeout(STEPPER_INACTIVITY_TIMEOUT, motion_planner.get_current_segment_data()).await {
            // Process segment plan
            Ok(Some(segment)) => {
                hwa::debug!("Segment init");

                let mut timings = Timings::new();

                // segment metronome
                //let mut absolute_ticker = embassy_time::Ticker::every(Duration::from_hz(MICRO_SEGMENT_PERIOD_HZ));
                let absolute_ticker_period = Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS.into());
                let mut absolute_ticker_start = now();
                let t_segment_start = now();
                // Annotate how many time in ticks the executor is duty in this segment
                let mut duty = Duration::from_ticks(0);

                duty += t_segment_start.elapsed();

                motion_planner.event_bus.publish_event(EventStatus::containing(EventFlags::MOVING)).await;

                let mut t0 = now();

                // TODO: Get from conf
                let to_ustep: Real = Real::from_lit(16, 0);

                let usteps_to_advance: TVector<u32> = (
                    (segment.segment_data.vdir.abs() * Real::from_lit(segment.segment_data.displacement_u as i64, 0) * to_ustep) / Real::from_lit(1000, 0)
                ).round().map_coords(|c| { Some(c.to_i32().unwrap_or(0) as u32) });

                hwa::debug!("usteps_to_advance = {}", usteps_to_advance);
                let mut usteps_advanced: TVector<u32> = TVector::zero();
                let mut tick_id = 1;

                hwa::debug!(">> Motion segment will advance {} mm at ~{} mm/sec ({} usteps/sec)",
                    (segment.segment_data.displacement_u as f64) / 1000.0f64,
                    segment.motion_profile.v_lim.rdp(4),
                    (segment.motion_profile.v_lim * to_ustep).rdp(4)
                );

                // global axis advance counter
                let mut axis_steps_advanced_precise: TVector<Real> = TVector::zero();

                let mut t_ref = t_segment_start;
                #[cfg(all(feature = "native", feature = "plot-timings"))]
                {
                    motion_planner.start_segment(t_segment_start, t_segment_start).await;
                }

                let mut t_tick = now();

                duty += t0.elapsed();

                motion_planner.motion_driver.lock().await.enable_and_set_dir(&segment.segment_data.vdir);
                if steppers_off {
                    hwa::info!("Powering steppers on");
                }
                steppers_off = false;

                timings.set_prep();

                /* WIP
                let offset = Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS.into());
                for _ps in segment.motion_profile.iterate(t_segment_start, offset) {
                    embassy_time::Timer::after_ticks(MICRO_SEGMENT_PERIOD_TICKS.into()).await;
                }
                */

                let mut eof = false;

                // Micro-segments interpolation
                loop {

                    // Feed watchdog because this high prio task could cause CPU starvation
                    let t_micro_segment_start = now();
                    timings.u_reset();

                    watchdog.lock().await.pet();
                    timings.add_u_comp();

                    t0 = now();

                    let time = Real::from_lit((t_tick + Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS as u64)).duration_since(t_segment_start).as_millis() as i64, 3);
                    hwa::debug!("\ttick #{} at t = {}", tick_id, time);
                    t_ref += Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS as u64);

                    // Interpolate as micro-segments

                    // TODO: Convert to an iterator to simplify logic
                    let next_position = if let Some(estimated_position) = segment.motion_profile.eval_position(time) {
                        Some(estimated_position)
                    } else {
                        if !eof && usteps_advanced.bounded_by(&usteps_to_advance) {
                            eof = true;
                            Some(Real::from_lit(segment.segment_data.displacement_u.into(), 3))
                        }
                        else {
                            None
                        }
                    };

                    if let Some(estimated_position) = next_position {

                        let axial_pos: TVector<Real> = segment.segment_data.vdir.abs() * estimated_position;

                        let step_pos: TVector<Real> = axial_pos * to_ustep;

                        let steps_to_advance_precise: TVector<Real> = (step_pos - axis_steps_advanced_precise).round();
                        axis_steps_advanced_precise += steps_to_advance_precise;

                        hwa::debug!("\ttick #{} \\Delta_pos {}, \\Delta_axis {} \\Delta_us: {} \\delta_us: {} {}", tick_id, estimated_position.rdp(4),
                        axial_pos.rdp(4), step_pos.rdp(4), steps_to_advance_precise.rdp(0), steps_to_advance_precise.rdp(4));

                        let tick_period_by_axis = (steps_to_advance_precise
                            .map_val(Real::from_lit((MICRO_SEGMENT_PERIOD_TICKS) as i64, 0)) / steps_to_advance_precise
                        ).round();


                        hwa::debug!("\ttick #{} ustep period {}", tick_id, tick_period_by_axis);
                        // The default rate is larger than a micro-segment period when there is not move in an axis, so no pulses are driven
                        let default_rate = (MICRO_SEGMENT_PERIOD_TICKS + MICRO_SEGMENT_PERIOD_TICKS) as u64;
                        let mut multi_timer = MultiTimer::new(
                            MICRO_SEGMENT_PERIOD_TICKS as u64,
                            [
                                ChannelStatus::new(StepperChannel::X, tick_period_by_axis.x.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default_rate)),
                                ChannelStatus::new(StepperChannel::Y, tick_period_by_axis.y.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default_rate)),
                                ChannelStatus::new(StepperChannel::Z, tick_period_by_axis.z.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default_rate)),
                                #[cfg(feature = "has-extruder")]
                                ChannelStatus::new(StepperChannel::E, tick_period_by_axis.e.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default_rate)),
                            ],
                        );

                        hwa::debug!("\tcomputing_time: {} us", t_micro_segment_start.elapsed().as_micros());
                        duty += t0.elapsed();

                        timings.add_u_comp();

                        let mut drv = motion_planner.motion_driver.lock().await;
                        t0 = now();
                        #[cfg(feature = "no-real-time")]
                        drv.update_clock(t_tick);
                        #[cfg(feature = "no-real-time")]
                        let mut t_micro_segment = t_tick;
                        timings.add_u_remaining();
                        loop {
                            match multi_timer.next() {
                                None => {
                                    break;
                                },
                                Some((channel, _delay)) => {
                                    #[cfg(feature = "native")]
                                    hwa::trace!("\t{:?} {}", channel, _delay.as_micros());

                                    #[cfg(feature = "no-real-time")]
                                    {
                                        t_micro_segment += Duration::from_ticks(_delay.as_ticks());
                                        drv.update_clock(t_micro_segment);
                                    }

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
                                    #[cfg(feature = "has-extruder")]
                                    if channel.contains(StepperChannel::E) {
                                        usteps_advanced.increment(CoordSel::E, 1u32);
                                        drv.e_step_pin_high();
                                    }

                                    #[cfg(feature = "no-real-time")]
                                    {
                                        t_micro_segment += STEPPER_PULSE_WIDTH_US;
                                        drv.update_clock(t_micro_segment);
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
                                    #[cfg(feature = "has-extruder")]
                                    if channel.contains(StepperChannel::E) {
                                        drv.e_step_pin_low();
                                    }
                                    s_block_for(STEPPER_PULSE_WIDTH_US);
                                    #[cfg(feature = "no-real-time")]
                                    {
                                        multi_timer.sync_clock(STEPPER_PULSE_WIDTH_US);
                                    }
                                },
                            }
                        }
                        timings.add_u_stepping();

                        //hwa::debug!("\tInterpolation task took {} ms", tx.elapsed().as_millis());
                        hwa::debug!("\ttick #{} usteps_exp: {} usteps_adv: {}", tick_id, usteps_to_advance, usteps_advanced);

                        if (eof && usteps_advanced.is_nan_or_zero()) || !usteps_advanced.bounded_by(&usteps_to_advance) {
                            //let _rpos = usteps_advanced.map_coords(|c| { Some(Real::from_lit(c as i64, 0)) }) / to_ustep;
                            //hwa::info!("<< Segment completed in {} ms. axial_pos: {} mm real_pos: {} mm", t_segment_start.elapsed().as_millis(), axial_pos.rdp(4), _rpos  );
                            //#[cfg(feature = "native")]
                            {
                                let t_sec = Real::from_lit(t_segment_start.elapsed().as_millis().try_into().unwrap_or(0), 3);
                                let s_mm = Real::from_lit(segment.segment_data.displacement_u.try_into().unwrap_or(0), 3);
                                let rate = if s_mm.is_zero() { Real::zero() } else {s_mm / t_sec};
                                hwa::info!("# Segment of length {} mm completed in {} sec ({} mm/s).", s_mm.rdp(4), t_sec.rdp(4), rate.rdp(3)  );
                            }
                            duty += t0.elapsed();
                            motion_planner.consume_current_segment_data().await;
                            motion_planner.defer_channel.send(DeferEvent::LinearMove(DeferType::Completed)).await;
                            break;
                        }
                        timings.add_u_remaining();
                    }
                    else {
                        // Reached end-of-segment, but still missing any step
                        //#[cfg(feature = "native")]
                        {
                            let t_sec = Real::from_lit(t_segment_start.elapsed().as_millis().try_into().unwrap(), 3);
                            let s_mm = Real::from_lit(segment.segment_data.displacement_u.try_into().unwrap(), 3);
                            let missing = (usteps_to_advance - usteps_advanced).map_coords(|c| if c > 0 { Some(c) } else { None });
                            let rate = if t_sec.is_zero() {Real::zero() } else {s_mm / t_sec};
                            hwa::warn!("# Segment of length {} mm truncated after {} sec ({} mm/s). Missing: {}", s_mm, t_sec, rate, missing  );
                        }
                        duty += t0.elapsed();
                        motion_planner.consume_current_segment_data().await;
                        motion_planner.defer_channel.send(DeferEvent::LinearMove(DeferType::Completed)).await;
                        timings.add_u_remaining();
                        break;
                    }

                    t0 = now();

                    let t_tick_elapsed = now().duration_since(t_micro_segment_start);
                    let rem = PERIOD_MS - t_tick_elapsed.as_millis() as i32;
                    hwa::info!("== t = {} ms took {} ms. pend = {} ms. tot = {} ms",
                        t_tick.as_millis(),
                        t_tick_elapsed.as_millis(),
                        rem,
                        (t_tick_elapsed).as_millis() as i32 + rem);
                    tick_id += 1;

                    t_tick += Duration::from_ticks(MICRO_SEGMENT_PERIOD_TICKS as u64);
                    #[cfg(feature = "no-real-time")]
                    {
                        let mut drv = motion_planner.motion_driver.lock().await;
                        drv.update_clock(t_tick);
                    }
                    duty += t0.elapsed();

                    //absolute_ticker.next().await;
                    let tn = now();
                    let elapsed = tn.checked_duration_since(absolute_ticker_start).unwrap_or(Duration::from_ticks(0));
/////////////////////////
// PREEMPTION START
/////////////////////////
                    if absolute_ticker_period > elapsed {
                        let pend = absolute_ticker_period - elapsed;
                        absolute_ticker_start = tn + pend;
                        embassy_time::Timer::after(pend).await;
                        hwa::info!("uSegment {} (not)waiting {} us (elapsed: {} us, period: {} us) eof: {}", tick_id, (absolute_ticker_period - elapsed).as_micros(), elapsed.as_micros(), absolute_ticker_period.as_micros(), eof);
                    }
                    else {
                        absolute_ticker_start = tn;
                        hwa::warn!("uSegment {} lagging {} us (elapsed: {}, period: {})", tick_id, (elapsed - absolute_ticker_period).as_micros(), elapsed.as_ticks(), absolute_ticker_period.as_ticks());
                    }
////////////////////////
// PREEMPTION END
////////////////////////
                    #[cfg(all(feature = "native", feature = "plot-timings"))]
                    motion_planner.mark_microsegment().await;
                    timings.add_u_remaining();
                }
                timings.set_execution();

                #[cfg(all(feature = "native", feature = "plot-timings"))]
                motion_planner.end_segment().await;
                hwa::debug!("Segment done");
                let segment_ms = Real::from_lit(t_segment_start.elapsed().as_micros().try_into().unwrap_or(0), 3).rdp(3);
                let duty_ms = Real::from_lit(duty.as_micros().try_into().unwrap_or(0), 3).rdp(3);
                let rate = if segment_ms.is_zero() { Real::zero() } else{ ( duty_ms * ONE_HUNDRED)  / segment_ms}.rdp(2);
                hwa::info!("ISR CPU LOAD: {}% (total: {} ms, taken_up: {} ms)", rate, segment_ms, duty_ms);
                motion_planner.event_bus.publish_event(EventStatus::not_containing(EventFlags::MOVING)).await;
                timings.set_remaining();
                timings.report();
            }
            // Homing
            Ok(None) => {
                hwa::debug!("Homing init");
                if !motion_planner.do_homing().await.is_ok() {
                    // TODO
                }
                motion_planner.consume_current_segment_data().await;
                hwa::debug!("Homing done");
            }
            // Timeout
            Err(_) => {
                if !steppers_off {
                    hwa::info!("Timeout. Powering steppers off");
                    let mut drv = motion_planner.motion_driver.lock().await;
                    drv.pins.disable_all_steppers();
                    steppers_off = true;
                }
            }
        }
    }
}
