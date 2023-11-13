//! TODO: INCOMPLETE! BUGGY! This feature is still very experimental
use embassy_time;
use embassy_time::{Duration, with_timeout};
#[cfg(feature = "with-motion")]
use crate::{hwa, hwa::controllers::{DeferEvent, DeferType}};
#[allow(unused)]
use crate::math::{Real, ONE_MILLION, ONE_THOUSAND};
use crate::tgeo::{CoordSel, TVector};
#[allow(unused)]
use printhor_hwa_common::{EventStatus, EventFlags};

const PERIOD_HZ: u64 = 100;
const PULSE_WIDTH_US: u32 = Duration::from_hz(PERIOD_HZ).as_micros() as u32;
const PULSE_WITDH_TICKS: u32 = Duration::from_hz(PERIOD_HZ).as_ticks() as u32;

#[derive(Clone, Copy)]
pub enum ChannelName {
    X,
    Y,
    Z,
    #[cfg(feature="has-extruder")]
    E
}

#[derive(Clone, Copy)]
pub struct ChannelStatus {
    next_tick: u64,
    width: u64,
    name: ChannelName,
}

impl ChannelStatus {
    pub const fn new(name: ChannelName, width: u64) -> Self {
        Self{
            next_tick: 0,
            width,
            name,
        }
    }
}

pub struct MultiTimer<const N: usize>
{
    timeout_instant: u64,
    channels: [ChannelStatus; N],
}

impl<const N: usize> MultiTimer<N> {
    pub fn new(interval: u64, mut s: [ChannelStatus; N]) -> Self {
        let t0 = embassy_time::Instant::now().as_ticks();
        for i in s.iter_mut() {
            i.next_tick = i.width + t0
        }
        Self {
            timeout_instant: t0 + interval,
            channels: s,
        }
    }

    pub fn next(&mut self) -> Option<ChannelName> {
        let mut next_tick = self.timeout_instant;
        let mut target_channel: Option<&mut ChannelStatus> = None;
        for c in self.channels.iter_mut() {
            if c.next_tick < next_tick && c.next_tick < (self.timeout_instant - (c.width / 2)) {
                next_tick = c.next_tick;
                target_channel = Some(c);
            }
        }
        if next_tick >= self.timeout_instant {
            return None
        }
        return if let Some(channel) = target_channel {
            while embassy_time::Instant::now().as_ticks() < channel.next_tick {}
            channel.next_tick += channel.width;
            Some(channel.name)
        } else {
            None
        };
    }
}

pub fn s_block_for(duration: Duration) {
    let expires_at = embassy_time::Instant::now() + duration;
    while embassy_time::Instant::now() < expires_at {}
}

pub struct Directions {
    pub x: bool,
    pub y: bool,
    pub z: bool,
    #[cfg(feature="has-extruder")]
    pub e: bool,
}

impl Directions {
    pub const fn new(x: bool, y: bool, z: bool,
                     #[cfg(feature="has-extruder")]
                     e: bool
    ) -> Self {
        Self {
            x,
            y,
            z,
            #[cfg(feature="has-extruder")]
            e,
        }
    }
}


/***
This task feeds watchdog to ensure no reset happen due high CPU starvation when feed rate is very high
 */
#[embassy_executor::task]
pub async fn stepper_task(
    motion_planner: hwa::controllers::MotionPlannerRef,
    watchdog: hwa::WatchdogRef)
{

    let one_us = Duration::from_micros(1);
    let timeout = Duration::from_secs(20);
    let period_ms: i32 = (PULSE_WIDTH_US / 1000) as i32;
    let mut steppers_off = true;

    motion_planner.start().await;

    #[allow(unused)]
        #[allow(unused_mut)]
    let mut s = motion_planner.event_bus.subscriber().await;

    hwa::info!("Pulse controller starting with {} us, {} ms period", PULSE_WIDTH_US, period_ms);

    loop {

        //s.wait_until(EventStatus::containing(EventFlags::SYS_READY | EventFlags::ATX_ON).and_not_containing(EventFlags::SYS_ALARM)).await;

        match with_timeout(timeout, motion_planner.get_current_segment_data()).await {
            // Process segment plan
            Ok(Some(segment)) => {

                let mut usteps_advanced: TVector<u32> = TVector::zero();
                let mut tick_id = 1;

                let to_ustep: Real = Real::from_lit(32, 0);
                let expected_advance = Real::from_lit(segment.segment_data.displacement_u as i64, 0) / Real::from_lit(1000, 0);

                // segment metronome
                hwa::info!(">> Motion segment will advance {} mm at ~{} mm/sec ({} usteps/sec)",
                    (segment.segment_data.displacement_u as f64) / 1000.0f64,
                    segment.motion_profile.v_lim.rdp(4),
                    (segment.motion_profile.v_lim * to_ustep).rdp(4)
                );

                ////////////////////////////////////////
                let mut absolute_ticker = embassy_time::Ticker::every(Duration::from_hz(PERIOD_HZ));
                let t_ref = embassy_time::Instant::now() - Duration::from_micros(PULSE_WIDTH_US.into());

                // global axis advance counter
                let mut axis_steps_advanced_precise: TVector<Real> = TVector::zero();

                let t_segment = embassy_time::Instant::now();

                loop { // Iterate on segment

                    let t_tick = embassy_time::Instant::now();

                    // Feed watchdog because this high prio task could cause CPU starvation
                    watchdog.lock().await.pet();

                    let time = Real::from_lit(t_ref.elapsed().as_millis() as i64, 3);

                    hwa::debug!("\ttick #{} at t = {} ms", tick_id, t_ref.elapsed().as_millis());

                    // Interpolate as microsegments
                    let estimated_position: Real = segment.motion_profile.eval_position(time);
                    let axial_pos: TVector<Real> = segment.segment_data.vdir * estimated_position;

                    let axial_dirs: Directions = {
                        let dir_ref = &segment.segment_data.vdir;
                        Directions::new(
                            dir_ref.x.and_then(|v| Some(v.is_positive())).unwrap_or(false),
                            dir_ref.y.and_then(|v| Some(v.is_positive())).unwrap_or(false),
                            dir_ref.z.and_then(|v| Some(v.is_positive())).unwrap_or(false),
                            #[cfg(feature="has-extruder")]
                            dir_ref.e.and_then(|v| Some(v.is_positive())).unwrap_or(false),
                        )
                    };
                    let step_pos: TVector<Real> = axial_pos * to_ustep;

                    let steps_to_advance_precise: TVector<Real> = (step_pos - axis_steps_advanced_precise).floor();
                    axis_steps_advanced_precise += steps_to_advance_precise;

                    hwa::debug!("\ttick #{} \\Delta_pos {}, \\Delta_axis {} \\Delta_us: {} \\delta_us: {} {}", tick_id, estimated_position.rdp(4),
                        axial_pos.rdp(4), step_pos.rdp(4), steps_to_advance_precise.rdp(0), steps_to_advance_precise.rdp(4));

                    let mut drv = motion_planner.motion_driver.lock().await;

                    steppers_off = false;

                    let _pulses_by_period = (steps_to_advance_precise
                        .map_val(Real::from_lit((PULSE_WITDH_TICKS) as i64, 0)) / (steps_to_advance_precise + TVector::one())
                    ).floor();
                    let v: TVector<Real> = _pulses_by_period;
                    hwa::debug!("\ttick #{} width {} ticks, ustep per {}", tick_id, PULSE_WITDH_TICKS, v);
                    let default = PULSE_WITDH_TICKS as u64 + 10;
                    let vx = [
                        ChannelStatus::new(ChannelName::X, v.x.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default)),
                        ChannelStatus::new(ChannelName::Y, v.y.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default)),
                        ChannelStatus::new(ChannelName::Z, v.z.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default)),
                        #[cfg(feature="has-extruder")]
                        ChannelStatus::new(ChannelName::E, v.e.and_then(|cv| cv.to_i64().and_then(|v| Some(v as u64))).unwrap_or(default)),
                    ];

                    let mut multi_timer = MultiTimer::new(
                        PULSE_WITDH_TICKS as u64,
                        vx,
                    );

                    let tx = embassy_time::Instant::now();
                    loop {
                        match multi_timer.next() {
                            None => {
                                break;
                            },
                            Some(ChannelName::X) => {
                                drv.pins.enable_x_stepper();
                                usteps_advanced.increment(CoordSel::X, 1u32);
                                match &axial_dirs.x {
                                    true => drv.pins.x_dir_pin.set_high(),
                                    false => drv.pins.x_dir_pin.set_low(),
                                }
                                drv.pins.x_step_pin.set_high();
                                s_block_for(one_us);
                                drv.pins.x_step_pin.set_low();
                                s_block_for(one_us);
                            },
                            Some(ChannelName::Y) => {
                                drv.pins.enable_y_stepper();
                                usteps_advanced.increment(CoordSel::Y, 1u32);
                                match &axial_dirs.y {
                                    true => drv.pins.y_dir_pin.set_high(),
                                    false => drv.pins.y_dir_pin.set_low(),
                                }
                                drv.pins.y_step_pin.set_high();
                                s_block_for(one_us);
                                drv.pins.y_step_pin.set_low();
                                s_block_for(one_us);
                            },
                            Some(ChannelName::Z) => {
                                drv.pins.enable_z_stepper();
                                usteps_advanced.increment(CoordSel::Z, 1u32);
                                match &axial_dirs.z {
                                    true => drv.pins.z_dir_pin.set_high(),
                                    false => drv.pins.z_dir_pin.set_low(),
                                }
                                drv.pins.z_step_pin.set_high();
                                s_block_for(one_us);
                                drv.pins.z_step_pin.set_low();
                                s_block_for(one_us);
                            },
                            #[cfg(feature="has-extruder")]
                            Some(ChannelName::E) => {
                                drv.pins.enable_e_stepper();
                                usteps_advanced.increment(CoordSel::E, 1u32);
                                match &axial_dirs.e {
                                    true => drv.pins.e_dir_pin.set_high(),
                                    false => drv.pins.e_dir_pin.set_low(),
                                }
                                drv.pins.e_step_pin.set_high();
                                s_block_for(one_us);
                                drv.pins.e_step_pin.set_low();
                                s_block_for(one_us);
                            },
                        }
                    }
                    hwa::debug!("\tInterpolation task took {} ms", tx.elapsed().as_millis());
                    hwa::debug!("\ttick #{} usteps_adv: {}", tick_id, usteps_advanced);

                    if estimated_position >= expected_advance {
                        let rpos = usteps_advanced.map_coords(|c| {Some(Real::from_lit(c as i64, 0))}) / to_ustep;
                        hwa::info!("<< Segment completed. axial_pos: {} mm real_pos: {} mm", axial_pos.rdp(4), rpos  );


                        motion_planner.consume_current_segment_data().await;
                        motion_planner.defer_channel.send(DeferEvent::LinearMove(DeferType::Completed)).await;
                        break;
                    }

                    let elapsed = t_segment.elapsed();
                    let t_tick_elapsed = t_tick.elapsed();
                    let rem = period_ms - t_tick_elapsed.as_millis() as i32;
                    hwa::debug!("^^ t = {} ms took {} ms. pend = {} ms. tot = {} ms", elapsed.as_millis(), t_tick_elapsed.as_millis(), rem, (t_tick_elapsed).as_millis() as i32 + rem);
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
                    drv.pins.disable_all_steppers();
                    steppers_off = true;
                }
            }
        }
    }
}
