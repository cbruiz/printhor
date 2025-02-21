fn main() {}
/*
cfg_if::cfg_if! {if #[cfg(feature="broken")] {
extern crate alloc;
extern crate core;

mod hwa;
mod control;
mod helpers;
mod math;


use math::Real;
use num_traits::float::FloatCore;
use num_traits::ToPrimitive;
use hwa::EventBusController;
use rust_decimal::Decimal;
use rust_decimal_macros::dec;

use crate::control::motion::*;
use crate::control::{GCodeCmd, GCodeValue, XYZF};
use crate::hwa::controllers::motion;
use crate::hwa::controllers::LinearMicrosegmentStepInterpolator;
use crate::hwa::controllers::SoftTimer;
use crate::hwa::controllers::SoftTimerDriver;
use crate::hwa::controllers::State;
use crate::hwa::controllers::StepPlanner;
use crate::hwa::device::MotionDevice;
use crate::hwa::drivers::{MotionDriver, MotionDriverParams};
use crate::hwa::CommChannel;
use crate::math::{TWO, ZERO};
use crate::prelude::hwa::controllers::ExecPlan;
use crate::math::{CoordSel, TVector};

const STEPPER_PLANNER_MICROSEGMENT_FREQUENCY: u32 = 500; // hwa::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
const STEPPER_PLANNER_CLOCK_FREQUENCY: u32 = 100_000; // hwa::STEPPER_PLANNER_CLOCK_FREQUENCY;

const STEPPER_PLANNER_MICROSEGMENT_PERIOD_US: u32 =
    1_000_000 / STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
const STEPPER_PLANNER_CLOCK_PERIOD_US: u32 = 1_000_000 / STEPPER_PLANNER_CLOCK_FREQUENCY;

struct DataPoints {
    pub last_time: Option<Real>,
    pub last_time_discrete: Option<Real>,
    pub last_pos: Real,
    pub last_pos_discrete: Real,
    pub last_spd: Real,
    pub last_spd_discrete: Real,
    pub time: Vec<f64>,
    pub time_seg: Vec<f64>,
    pub time_discrete: Vec<f64>,
    pub time_discrete_deriv: Vec<f64>,
    pub time_discrete_seg: Vec<f64>,
    pub time_discrete_acc: Vec<f64>,
    pub pos: Vec<f64>,
    pub seg_pos: Vec<f64>,
    pub pos_discrete: Vec<f64>,
    pub spd: Vec<f64>,
    pub spd_discrete: Vec<f64>,
    pub seg_discrete: Vec<f64>,

    pub acc_discrete: Vec<f64>,

    pub t0: Real,
    pub acc_time: Real,
    pub acc_disp: Real,
}
impl DataPoints {
    pub fn new(
        initial_pos: Real,
        initial_pos_discrete: Real,
        initial_spd: Real,
        initial_spd_discrete: Real,
    ) -> Self {
        let mut dp = Self {
            last_time: None,
            last_time_discrete: None,
            last_pos: initial_pos,
            last_pos_discrete: initial_pos_discrete,
            last_spd: initial_spd,
            last_spd_discrete: initial_spd_discrete,
            time: Vec::new(),
            time_seg: Vec::new(),
            time_discrete: Vec::new(),
            time_discrete_deriv: Vec::new(),
            time_discrete_seg: Vec::new(),
            time_discrete_acc: Vec::new(),
            pos: Vec::new(),
            seg_pos: Vec::new(),
            pos_discrete: Vec::new(),
            spd: Vec::new(),
            spd_discrete: Vec::new(),
            seg_discrete: Vec::new(),
            acc_discrete: Vec::new(),
            t0: Real::zero(),
            acc_time: Real::zero(),
            acc_disp: Real::zero(),
        };
        dp.time.push(0.0);
        dp.time_discrete.push(0.0);
        dp.time_discrete_deriv.push(0.0);
        dp.pos.push(dp.last_pos.to_f64());
        dp.pos_discrete.push(dp.last_pos_discrete.to_f64());
        dp.spd.push(dp.last_spd.to_f64());
        dp.spd_discrete.push(dp.last_spd_discrete.to_f64());
        dp
    }

    pub fn add_ideal(&mut self, time: Real, pos: Real) {
        self.time.push(time.to_f64());
        let spd = if let Some(t) = self.last_time {
            let dt = time - t;
            (pos - self.last_pos) / dt
        } else {
            self.last_spd
        };

        self.pos.push(pos.to_f64());
        self.spd.push(spd.to_f64());
        self.last_time = Some(time);
        self.last_pos = pos;
        self.last_spd = spd;
        //hwa::info!("Pos: {} Spd: {}", pos.rdp(3), spd.rdp(3));
    }

    pub fn seg_start(&mut self, time: Real, pos: Real, speed: Real) {
        self.time_seg.push(time.to_f64());
        self.seg_pos.push(pos.to_f64());
        self.time_discrete_seg.push(time.to_f64());
        self.seg_discrete.push(speed.to_f64());
    }

    pub fn acc_pred(&mut self, time0: Real, speed_p0: Real, time1: Real, speed_p1: Real) {
        self.time_discrete_acc.push(time0.to_f64());
        self.acc_discrete.push(speed_p0.to_f64());
        self.time_discrete_acc.push(time1.to_f64());
        self.acc_discrete.push(speed_p1.to_f64());
    }

    pub fn real_start(&mut self, time: Real) {
        self.t0 = time;
        self.acc_time = Real::zero();
        self.acc_disp = Real::zero();
        let last_spd_discrete = if self.last_spd_discrete.is_zero() {
            math::ZERO
        } else {
            self.last_spd_discrete
        };
        //self.time_discrete_deriv.push(time.to_f64());
        //self.spd_discrete.push(last_spd_discrete.to_f64());
    }

    pub fn real_end(&mut self, time: Real, s_id: u32) {
        let dt = self.acc_time;
        let spd = if dt.is_zero() {
            math::ZERO
        } else {
            self.acc_disp / dt
        };
        self.time_discrete_deriv
            .push((time - self.acc_time).to_f64());
        self.spd_discrete.push(spd.to_f64());
        self.time_discrete_deriv.push(time.to_f64());
        self.spd_discrete.push(spd.to_f64());
        self.last_spd_discrete = spd;
        hwa::trace!(
            "S[{}] Time: {} dt: {}, spd: {}",
            s_id,
            time.rdp(4),
            dt.rdp(4),
            spd.rdp(2)
        );
    }

    pub fn add_real(&mut self, time: Real, pos_discrete: Real, _s_id: u32) {
        self.time_discrete.push(time.to_f64());

        self.acc_time += time - self.last_time_discrete.unwrap_or(math::ZERO);
        self.acc_disp += pos_discrete - self.last_pos_discrete;

        self.pos_discrete.push(pos_discrete.to_f64());
        self.last_time_discrete = Some(time);
        self.last_pos_discrete = pos_discrete;
        //hwa::info!("RPos: {}", pos_discrete.rdp(3));
    }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    // How many us there are in a microsegment period
    let micro_segment_us_by_period: Real =
        Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 0);
    let micro_segment_period_secs: Real =
        Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 6);

    let sampling_time: Real = Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US as i64, 6);

    // First, prepare the underlying machinery
    env_logger::init();
    hwa::info!(
        "Micro-segment sampling freq: {} hz ({} us), stepper clock freq: {} hz ({} us)",
        STEPPER_PLANNER_MICROSEGMENT_FREQUENCY,
        micro_segment_us_by_period,
        STEPPER_PLANNER_CLOCK_FREQUENCY,
        STEPPER_PLANNER_CLOCK_PERIOD_US
    );
    let mut data_points = DataPoints::new(Real::zero(), Real::zero(), Real::zero(), Real::zero());
    let mut ref_time = Real::zero();

    let event_bus = EventBusController::new(
        hwa::make_static_controller!(
            "EventBusChannelController",
            hwa::EventbusMutexType,
            hwa::EventBusChannelController<hwa::EventBusChannelMutexType>,
            hwa::EventBusChannelController::new(
                hwa::make_static_ref!(
                    "EventBusChannel",
                    hwa::EventBusPubSubType<hwa::EventBusChannelMutexType>,
                    hwa::EventBusPubSubType::new()
                )
            )
        )
    );
    let defer_channel: hwa::GenericDeferChannel<hwa::DeferChannelMutexType> = hwa::GenericDeferChannel::new(
        hwa::make_static_ref!(
            "DeferChannel",
            hwa::DeferChannelChannelType<hwa::DeferChannelMutexType>,
            hwa::DeferChannelChannelType::new()
        )
    );

    let motion_config = hwa::make_static_controller!(
        "MotionConfig",
        hwa::MotionConfigMutexType,
        hwa::controllers::MotionConfig,
        hwa::controllers::MotionConfig::new()
    );

    let driver_device = hwa::drivers::MotionDriver{
        pins: hwa::device::StepActuator::new(),
    };

    let motion_driver = hwa::make_static_controller!(
            "MotionDriver",
            hwa::MotionDriverMutexType,
            hwa::drivers::MotionDriver,
            driver_device
        );

    let soft_timer = SoftTimer::new();
    soft_timer.setup(motion_driver.clone());
    let mut motion_planner =
        motion::MotionPlanner::new(defer_channel, motion_config, motion_driver);

    const V_MAX: u32 = 500;
    const A_MAX: u32 = 10000;
    const J_MAX: u32 = 20000;

    let constraints = Constraints {
        v_max: Real::from_lit(V_MAX.into(), 0),
        a_max: Real::from_lit(A_MAX.into(), 0),
        j_max: Real::from_lit(J_MAX.into(), 0),
    };

    // Physical Machine configuration

    motion_planner
        .set_max_speed(TVector::from_coords(
            Some(V_MAX),
            Some(V_MAX),
            Some(V_MAX),
            Some(V_MAX),
        ))
        .await;
    motion_planner
        .set_max_accel(TVector::from_coords(
            Some(A_MAX),
            Some(A_MAX),
            Some(A_MAX),
            Some(A_MAX),
        ))
        .await;
    motion_planner
        .set_max_jerk(TVector::from_coords(
            Some(J_MAX),
            Some(J_MAX),
            Some(J_MAX),
            Some(J_MAX),
        ))
        .await;
    motion_planner
        .set_steps_per_mm(
            math::Real::new(10, 0),
            math::Real::new(10, 0),
            math::Real::new(10, 0),
            math::Real::new(10, 0),
        )
        .await;
    motion_planner.set_usteps(16, 16, 16, 16).await;

    motion_planner.start(&event_bus).await;

    {
        let units_per_wu_s = motion_planner.get_steps_per_world_unit_as_vector().await;
        let mm_per_unit = units_per_mm * motion_planner.get_usteps_as_vector().await;
        let mm_per_step = TVector::one() / mm_per_unit;
        hwa::info!("* mm_per_step: [{}] (== min speed mm/s)", mm_per_step);
        hwa::info!("* sampling_time: {} s", sampling_time);
        hwa::info!(
            "* max speed: [{}] mm/s",
            TVector::one() / sampling_time * mm_per_step
        );
    }

    // Starting motion logic

    //hwa::info!("Set origin to [0, 0, 0]");
    motion_planner.set_last_planned_pos(&TVector::zero()).await;

    //hwa::info!("Segment from [0, 0, 0] to [0, 100, 0] at 200 mm/s");
    let _r00 = motion_planner
        .plan(
            CommChannel::Internal,
            &GCodeCmd::new(
                1,
                Some(1),
                GCodeValue::G0(XYZF {
                    x: None,
                    y: Some(Real::from_f32(100.0f32)),
                    z: None,
                    f: Some(Real::from_f32(2000.0f32)),
                }),
            ),
            false,
            &event_bus,
        )
        .await;

    let _r01 = motion_planner
        .plan(
            CommChannel::Internal,
            &GCodeCmd::new(
                2,
                Some(2),
                GCodeValue::G0(XYZF {
                    x: None,
                    y: Some(Real::from_f32(0.0f32)),
                    z: None,
                    f: Some(Real::from_f32(2000.0f32)),
                }),
            ),
            false,
            &event_bus,
        )
        .await;

    // Enqueue 5 consecutive moves
    //hwa::info!("Segment from [0, 0, 0] to [1, 0, 0] at 200 mm/s");

    let _r1 = motion_planner
        .plan(
            CommChannel::Internal,
            &GCodeCmd::new(
                3,
                Some(3),
                GCodeValue::G0(XYZF {
                    x: Some(Real::from_f32(1.0f32)),
                    y: None,
                    z: None,
                    f: Some(Real::from_f32(2000.0f32)),
                }),
            ),
            false,
            &event_bus,
        )
        .await;

    //hwa::info!("Segment from [1, 0, 0] to [2, 0, 0] at 200 mm/s");
    let _r1 = motion_planner
        .plan(
            CommChannel::Internal,
            &GCodeCmd::new(
                5,
                Some(5),
                GCodeValue::G0(XYZF {
                    x: Some(Real::from_f32(2.0f32)),
                    y: None,
                    z: None,
                    f: Some(Real::from_f32(2000.0f32)),
                }),
            ),
            false,
            &event_bus,
        )
        .await;

    //hwa::info!("Segment from [2, 0, 0] to [3.0, 0.0, 0] at 200 mm/s");
    let _r1 = motion_planner
        .plan(
            CommChannel::Internal,
            &GCodeCmd::new(
                6,
                Some(6),
                GCodeValue::G0(XYZF {
                    x: Some(Real::from_f32(3.0f32)),
                    y: None,
                    z: None,
                    f: Some(Real::from_f32(2000.0f32)),
                }),
            ),
            false,
            &event_bus,
        )
        .await;

    let _r1 = motion_planner
        .plan(
            CommChannel::Internal,
            &GCodeCmd::new(
                7,
                Some(7),
                GCodeValue::G0(XYZF {
                    x: Some(Real::from_f32(4.0f32)),
                    y: None,
                    z: None,
                    f: Some(Real::from_f32(2000.0f32)),
                }),
            ),
            false,
            &event_bus,
        )
        .await;

    let _r1 = motion_planner
        .plan(
            CommChannel::Internal,
            &GCodeCmd::new(
                8,
                Some(8),
                GCodeValue::G0(XYZF {
                    x: Some(Real::from_f32(5.0f32)),
                    y: None,
                    z: None,
                    f: Some(Real::from_f32(2000.0f32)),
                }),
            ),
            false,
            &event_bus,
        )
        .await;
    //hwa::info!("pos = {}", motion_planner.get_last_planned_pos().await.unwrap());

    let mut total_disp = Real::zero();
    let mut total_disp_discrete = Real::zero();
    let mut s_id = 0;
    loop {
        match motion_planner.next_plan(&event_bus).await {
            ExecPlan::Segment(segment, _) => {
                s_id += 1;
                hwa::debug!("Dequeuing move at t_ref={}", ref_time);
                data_points.seg_start(ref_time, total_disp, segment.segment_data.speed_enter_mms);
                let neutral_element = segment.segment_data.unit_vector_dir.map_val(&math::ZERO);

                match SCurveMotionProfile::compute(
                    segment.segment_data.displacement_mm,
                    segment.segment_data.speed_enter_mms,
                    segment.segment_data.speed_exit_mms,
                    &segment.segment_data.constraints,
                    false,
                ) {
                    Ok(motion_profile) => {
                        motion_profile.params_dump();
                        hwa::info!("S {}", s_id);

                        let units_per_mm =
                            (neutral_element + motion_planner.get_steps_per_mm_as_vector().await);

                        let mm_per_unit =
                            units_per_mm * motion_planner.get_usteps_as_vector().await;

                        let mm_per_step = TVector::one() / mm_per_unit;

                        let mut micro_segment_real_time_rel = micro_segment_period_secs;
                        let mut microsegment_iterator =
                            motion::SegmentIterator::new(&motion_profile, math::ZERO);

                        let mut microsegment_interpolator = LinearMicrosegmentStepInterpolator::new(
                            segment.segment_data.unit_vector_dir.abs(),
                            segment.segment_data.displacement_mm,
                            mm_per_unit,
                        );

                        let mut max_pos_reached = Real::zero();
                        let mut max_pos_reached_discrete = Real::zero();

                        let mut us_id = 0;
                        let mut prev_time = math::ZERO;
                        let mut p0 = math::ZERO;
                        // loop over the segment to interpolate micro-segments
                        loop {
                            us_id += 1;
                            if let Some((estimated_position, interval)) =
                                microsegment_iterator.next(micro_segment_real_time_rel)
                            {
                                // Got a micro-segment

                                let tprev = (micro_segment_real_time_rel - prev_time);
                                let tmax = motion_profile.i7_end() - prev_time;
                                let dt = tmax.min(tprev);

                                hwa::trace!(
                                    "at [{}] dt = {}",
                                    micro_segment_real_time_rel.rdp(4),
                                    dt.rdp(4)
                                );
                                let ds = estimated_position - p0;
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

                                let current_period_width = current_period_width_0; // (current_period_width_0 / sampling_time).ceil() * sampling_time;

                                hwa::trace!("  w = {}", current_period_width.rdp(6));

                                data_points.real_start(ref_time);

                                ref_time += current_period_width;
                                prev_time += current_period_width;
                                micro_segment_real_time_rel += current_period_width;

                                let w = (current_period_width * math::ONE_MILLION).round();

                                // Interpolate the micro-segment as regular pulse train along axes -> StepPlan
                                let has_more =
                                    microsegment_interpolator.advance_to(estimated_position, w);

                                hwa::trace!(
                                    "\tat t = {} [+{}]: p = {} [+ {} | {}] [i {}] v = {}",
                                    ref_time.rdp(4),
                                    current_period_width.rdp(4),
                                    (total_disp + estimated_position).rdp(6),
                                    estimated_position.rdp(6),
                                    microsegment_interpolator.advanced_steps(),
                                    interval,
                                    ((ds / w) * math::ONE_MILLION).rdp(6)
                                );

                                data_points.add_ideal(ref_time, total_disp + estimated_position);
                                max_pos_reached = estimated_position;

                                hwa::trace!(
                                    "\tAdvanced mm: {}",
                                    microsegment_interpolator.advanced_mm()
                                );

                                let mut step_planner = StepPlanner::from(
                                    microsegment_interpolator.state().clone(),
                                    hwa::StepperChannel::empty(),
                                    hwa::StepperChannel::empty(),
                                );

                                let mut pulse_offset = math::ZERO;
                                let mut tick_count = 0;

                                // Queue the StepPlan
                                soft_timer
                                    .push(
                                        microsegment_interpolator.state().clone(),
                                        hwa::StepperChannel::empty(),
                                        hwa::StepperChannel::empty(),
                                    )
                                    .await;

                                let mut stop = false;

                                // Directly perform the consumption of the queued StepPlan
                                // In real hardware, this is done by a dedicated handler invoked by ISR
                                loop {
                                    data_points.add_real(ref_time, total_disp_discrete, s_id);

                                    critical_section::with(|cs| {
                                        let _t1 = embassy_time::Instant::now();
                                        let mut counter: core::cell::RefMut<SoftTimerDriver> =
                                            soft_timer.0.borrow_ref_mut(cs);
                                        counter.on_interrupt();
                                        let _te1 = _t1.elapsed();
                                        hwa::trace!("on_int took {}", _te1.as_micros());

                                        let x = TVector::from_coords(
                                            Some(Real::from_lit(counter.pulses[0] as i64, 0)),
                                            Some(Real::from_lit(counter.pulses[1] as i64, 0)),
                                            Some(Real::from_lit(counter.pulses[2] as i64, 0)),
                                            Some(Real::from_lit(counter.pulses[3] as i64, 0)),
                                        ) / mm_per_unit;
                                        let norm = x.norm2().unwrap();

                                        data_points.add_real(
                                            ref_time,
                                            total_disp_discrete + norm,
                                            s_id,
                                        );

                                        if counter.state == State::Idle {
                                            total_disp_discrete += norm;
                                            stop = true;
                                        }
                                    });
                                    pulse_offset += sampling_time;
                                    if stop {
                                        break;
                                    }
                                }
                                soft_timer.flush().await;

                                data_points.real_end(ref_time, s_id);

                                if !has_more {
                                    // FIXME: Not sure if this is a good idea
                                    //break;
                                }
                            } else {
                                // No more micro-segments left: We advanced completely
                                break;
                            }
                        }
                        total_disp += max_pos_reached;
                        total_disp_discrete += max_pos_reached_discrete;
                    }
                    _ => unreachable!("Unable to compute motion plan"),
                };
                if motion_planner
                    .consume_current_segment_data(&event_bus)
                    .await
                    == 0
                {
                    data_points.seg_start(ref_time, total_disp, math::ZERO);
                    break;
                }
            }
            _ => {
                // Homing and dwell not expected to be handled
                // Hence, that means there is no more segments queued
                break;
            }
        }
    }

    hwa::info!(
        "Displ: {} mm, {} disc mm",
        total_disp.rdp(6),
        total_disp_discrete.rdp(6)
    );

    {
        use gnuplot::{
            AutoOption, MultiplotFillDirection::Downwards, MultiplotFillOrder::RowsFirst, Tick,
        };
        use gnuplot::{AxesCommon, Figure};
        use gnuplot::{DashType, PlotOption};

        let mut fg = Figure::new();

        cfg_if::cfg_if! {
            if #[cfg(feature="float-point-f32-impl")] {
                const MATH_PRECISION: &str = "[float point 32bits]";
            }
            else if #[cfg(feature="float-point-f64-impl")] {
                const MATH_PRECISION: &str = "[float point 64bits]";
            }
            else if #[cfg(feature="fixed-point-128-impl")] {
                const MATH_PRECISION: &str = "[fixed point 128bits]";
            }
            else {
                const MATH_PRECISION: &str = "[N/A]";
            }
        }

        fg.set_multiplot_layout(2, 1)
            .set_title(
                format!(
                    "{} Double S-Curve velocity profile\n[{} segments, {} mm displacement, {} hz interp, {} hz sampling]",
                    MATH_PRECISION,
                    s_id,
                    total_disp.rdp(6),
                    STEPPER_PLANNER_MICROSEGMENT_FREQUENCY,
                    STEPPER_PLANNER_CLOCK_FREQUENCY,
                ).as_str()
            )
            .set_scale(1.0, 1.0)
            .set_offset(0.0, 0.0)
            .set_multiplot_fill_order(RowsFirst, Downwards);

        fg.axes2d()
            .set_y_label("Position (mm)", &[])
            .points(
                data_points.time_seg.clone(),
                data_points.seg_pos.clone(),
                &[PlotOption::Color("black")],
            )
            .lines(
                data_points.time.clone(),
                data_points.pos.clone(),
                &[PlotOption::Color("blue")],
            )
            .lines(
                data_points.time_discrete.clone(),
                data_points.pos_discrete.clone(),
                &[PlotOption::Color("gray")],
            );
        fg.axes2d()
            .set_y_label("Velocity (mm/s)", &[])
            .points(
                data_points.time_discrete_seg.clone(),
                data_points.seg_discrete.clone(),
                &[PlotOption::Color("black")],
            )
            //.lines(data_points.time_discrete_acc.clone(), data_points.acc_discrete.clone(), &[PlotOption::Color("red")])
            .lines(
                data_points.time.clone(),
                data_points.spd.clone(),
                &[PlotOption::Color("green")],
            )
            .lines(
                data_points.time_discrete_deriv.clone(),
                data_points.spd_discrete.clone(),
                &[PlotOption::Color("gray")],
            );
        fg.show_and_keep_running().unwrap();
        fg.save_to_pdf("plot.pdf", 10.0f32, 10.0f32);
    }

    std::process::exit(0);
}

pub fn initialization_error() {
    let msg = "Unable to start because SYS_ALARM raised at startup. Giving up...";
    hwa::error!("{}", msg);
    panic!("{}", msg);
}
} else {
    fn main() {}
}
}
*/
