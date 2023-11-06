//! TODO: INCOMPLETE! BUGGY! This feature is still very experimental
#[cfg(not(feature = "native"))]
use core::ops::Neg;
use embassy_time;
use embassy_time::{block_for, Duration, with_timeout};
#[cfg(feature = "with-motion")]
use crate::{hwa, hwa::controllers::{DeferEvent, DeferType}};
#[allow(unused)]
use crate::math::{Real, ONE_MILLION, ONE_THOUSAND};
use crate::tgeo::TVector;
use core::cmp::min;
use rust_decimal_macros::dec;
#[allow(unused)]
use printhor_hwa_common::{EventStatus, EventFlags};

const PERIOD_HZ: u64 = 100;
const PULSE_WIDTH_US: u32 = Duration::from_hz(PERIOD_HZ).as_micros() as u32;
const _MONITOR_RATE: u64 = 2 * PERIOD_HZ;

#[cfg(not(feature = "native"))]
const _DEVIATION_TOLERANCE: f64 = 0.05f64;

#[cfg(not(feature = "native"))]
#[inline]
#[allow(unused)]
fn fabs(f1: f64) -> f64
{
    match f1 > 0f64 {
        true => f1,
        false => f1.neg()
    }
}

/*

WORK-IN-PROGRES....

enum ChannelName {
    X,
    Y,
    Z,
    E
}

struct ChannelStatus<CN> {
    next_tick: Duration,
    name: CN,
}

pub struct MultiTimer<CN, const N: usize>
where CN: Copy
{
    channels: [ChannelStatus<CN>; N],
}

*/

/***
This task feeds watchdog to ensure no reset happen due high CPU starvation when feed rate is very high
 */
#[embassy_executor::task]
pub async fn stepper_task(
    motion_planner: hwa::controllers::MotionPlannerRef,
    watchdog: hwa::WatchdogRef)
{

    let one_ns = Duration::from_micros(1);
    let timeout = Duration::from_secs(10);
    let period_ms: i32 = (PULSE_WIDTH_US / 1000) as i32;
    let mut steppers_off = true;

    let mut _nticks = 0u64;
    let mut acc: embassy_time::Duration = embassy_time::Duration::from_micros(0);
    let mut _acc_busy: embassy_time::Duration = embassy_time::Duration::from_micros(0);
    let mut t0 = embassy_time::Instant::now();

    let mut _tcurr: u32 = 0;
    let mut _mov_id = 1u64;

    motion_planner.start().await;

    #[allow(unused)]
        #[allow(unused_mut)]
    let mut s = motion_planner.event_bus.subscriber().await;

    hwa::info!("Pulse controller starting with {} us period", PULSE_WIDTH_US);

    loop {
        acc += t0.elapsed();
        t0 = embassy_time::Instant::now();
        _nticks += 1;
        _tcurr = 0;

        //s.wait_until(EventStatus::containing(EventFlags::SYS_READY | EventFlags::ATX_ON).and_not_containing(EventFlags::SYS_ALARM)).await;

        match with_timeout(timeout, motion_planner.get_current_segment_data()).await {
            // Process segment plan
            Ok(Some(segment)) => {
                hwa::trace!("Go move segment");

                let mut tick_id = 1;

                let to_ustep = Real::from_fixed(dec!(16.0)) * Real::from_fixed(dec!(8.0));

                ////////////////////////////////////////
                let mut absolute_ticker = embassy_time::Ticker::every(Duration::from_hz(PERIOD_HZ));
                let t_ref = embassy_time::Instant::now() - Duration::from_micros(PULSE_WIDTH_US.into());
                // segment metronome

                hwa::debug!("Will advance {} steps at ~{} steps/sec {} usteps/sec",
                    segment.segment_data.total_steps,
                    segment.motion_profile.v_lim.rdp(4),
                    (segment.motion_profile.v_lim * to_ustep).rdp(4)
                );

                // global linear advance counter
                let mut advanced_steps = 0u32;

                // global axis advance counter
                let mut axis_steps_advanced_precise: TVector<Real> = TVector::zero();
                let mut axis_advanced: TVector<Real> = TVector::zero();
                let mut axis_delta: TVector<Real> = TVector::zero();

                let t_segment = embassy_time::Instant::now();

                loop { // Iterate on segment

                    let t_tick = embassy_time::Instant::now();

                    // Feed watchdog because this high prio task could cause CPU starvation
                    watchdog.lock().await.pet();

                    let time = Real::from_lit(t_ref.elapsed().as_millis() as i64, 3);

                    hwa::debug!("tick_id {} t = {} ms", tick_id, t_ref.elapsed().as_millis());

                    // Interpolate as microsegments
                    let current_position_precise = segment.motion_profile.eval_position(time);
                    let axial_pos = segment.segment_data.vdir * current_position_precise;
                    let step_pos = (axial_pos * to_ustep).rdp(0);

                    let steps_to_advance_precise: TVector<Real> = step_pos - axis_steps_advanced_precise;
                    axis_steps_advanced_precise += steps_to_advance_precise;

                    hwa::debug!("\tpos {}, axis {} step {}", current_position_precise.rdp(4),
                        axial_pos.rdp(4), steps_to_advance_precise.rdp(4));

                    // legacy
                    let current_position = current_position_precise.to_i64().unwrap_or(0) as u32;
                    let steps_to_advance = current_position - min(advanced_steps, current_position);
                    let axial_disp = segment.segment_data.vdir * Real::new(steps_to_advance as i64, 0);
                    axis_advanced += axial_disp;
                    let rdp_adv = axis_advanced.map_coords(|c| Some(c.floor()));
                    let _abs_adv = rdp_adv - axis_delta;
                    axis_delta = rdp_adv;
                    // legacy end

                    if let Some(max_steps_adv) = steps_to_advance_precise.max()
                        .and_then(|v| v.to_i32())
                        .and_then(|v| if v > 0 { Some(v) } else { None })
                    {
                        let mut drv = motion_planner.motion_driver.lock().await;
                        steppers_off = false;

                        drv.pins.x_enable_pin.set_low();
                        drv.pins.x_dir_pin.set_high();

                        //let tv = steps_to_advance_precise.with_coord(CoordSel::all(), Some(Real::from_lit((PULSE_WIDTH_US) as i64, 3)));
                        //hwa::info!("  -- tv = {}", tv);
                        let _pulses_by_us = (steps_to_advance_precise
                            .map_val(Real::from_lit((PULSE_WIDTH_US) as i64, 0)) / (steps_to_advance_precise + TVector::one())
                        ).rdp(0);
                        hwa::debug!("  -- max pulses {} =>  {} us/pulse", max_steps_adv, _pulses_by_us);
                        // FIXME:
                        let mut ustep_pulse_ticker = embassy_time::Ticker::every(Duration::from_micros(_pulses_by_us.x.unwrap_or(Real::zero()).to_i32().unwrap() as u64));
                        let tx = embassy_time::Instant::now();
                        for _i in 0 .. max_steps_adv {
                            ustep_pulse_ticker.next().await;
                            hwa::debug!("\tuStep {} at t+{} ms", _i + 1, tx.elapsed().as_millis());
                            drv.pins.x_step_pin.set_high();
                            block_for(one_ns);
                            drv.pins.x_step_pin.set_low();
                            //block_for(one_ns);
                        }
                        hwa::debug!("\tTask took {} ms", tx.elapsed().as_millis())

                        //let _single_pulse_us = PULSE_WIDTH_US / (max_steps_adv + 1) as u32;

                    }

                    _tcurr += PULSE_WIDTH_US;
                    advanced_steps += steps_to_advance;

                    if advanced_steps >= segment.segment_data.total_steps {
                        hwa::info!("Now at {} {} | {} ", t_segment.elapsed().as_millis(), advanced_steps, axis_advanced);
                        hwa::info!("    ++ axial {} | steps {} ", axial_pos.rdp(4), steps_to_advance_precise);
                        motion_planner.consume_current_segment_data().await;
                        motion_planner.defer_channel.send(DeferEvent::LinearMove(DeferType::Completed)).await;
                        _mov_id += 1;
                        break;
                    }
                    let elapsed = t_segment.elapsed();
                    let t_tick_elapsed = t_tick.elapsed();
                    let rem = period_ms - t_tick_elapsed.as_millis() as i32;
                    hwa::debug!("    at {} ms took {} ms. pend = {} ms. tot = {} ms", elapsed.as_millis(), t_tick_elapsed.as_millis(), rem, (t_tick_elapsed).as_millis() as i32 + rem);
                    tick_id += 1;
                    absolute_ticker.next().await;

                }
            }
            // Homing
            Ok(None) => {
                hwa::info!("Doing homing");
                if !motion_planner.do_homing().await.is_ok() {
                    // TODO
                }
                hwa::info!("Homing done");
                motion_planner.consume_current_segment_data().await;
            }
            // Timeout
            Err(_) => {
                if !steppers_off {
                    hwa::info!("Timeout. Powering steppers off");
                    let mut drv = motion_planner.motion_driver.lock().await;
                    drv.pins.x_enable_pin.set_high();
                    drv.pins.y_enable_pin.set_high();
                    drv.pins.z_enable_pin.set_high();
                    drv.pins.e_enable_pin.set_high();
                    steppers_off = true;
                }
            }
        }
    }
}
