//! Auxiliary commandline program to study, research, analyze, investigate, evaluate and verify the motion pipelines
extern crate alloc;
extern crate core;

pub mod helpers;
pub mod hwa;
pub mod motion;
pub mod processing;
pub mod tasks;

mod instrumentation;

use hwa::HwiContract;

use hwa::CommChannel;
#[allow(unused)]
use hwa::RawHwiResource;
use hwa::controllers;
use motion::{MicroSegmentInterpolator, HybridMotionProfile, SegmentSampler};

// ── Scenario helper functions ─────────────────────────────────────────────────

#[allow(dead_code)]
fn gcode_collinear_x(segments: u32, step_mm: f32, speed_mm_min: Option<f32>) -> instrumentation::gcode::GCodeBuffer {
    let mut b = instrumentation::gcode::GCodeBuffer::new();
    let mut x = 0.0f32;
    for _ in 0..segments {
        x += step_mm;
        match speed_mm_min {
            Some(f) => b.append(&format!("G1 X{:.5} F{:.0}\n", x, f)),
            None => b.append(&format!("G0 X{:.5}\n", x)),
        }
    }
    b
}

#[allow(dead_code)]
fn gcode_reference_collinear(total_mm: f32, speed_mm_min: Option<f32>) -> instrumentation::gcode::GCodeBuffer {
    let mut b = instrumentation::gcode::GCodeBuffer::new();
    match speed_mm_min {
        Some(f) => b.append(&format!("G1 X{:.5} F{:.0}\n", total_mm, f)),
        None => b.append(&format!("G0 X{:.5}\n", total_mm)),
    }
    b
}

#[allow(dead_code)]
fn gcode_g92_diagonal() -> instrumentation::gcode::GCodeBuffer {
    let mut b = instrumentation::gcode::GCodeBuffer::new();
    b.append("G92 X100.18164 Y105\n");
    b.append("G1 X99.63526 Y104.99686 F7000 S0\n");
    b.append("G1 X99.09117 Y104.98748 F7000 S0\n");
    b.append("G1 X98.54937 Y104.97192 F7000 S0\n");
    b.append("G1 X98.00986 Y104.95024 F7000 S0\n");
    b
}

use tasks::task_stepper::{
    STEPPER_PLANNER_CLOCK_PERIOD_US, STEPPER_PLANNER_MICROSEGMENT_PERIOD_US,
};

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    printhor_main(spawner, false).await;
}

async fn printhor_main(spawner: embassy_executor::Spawner, _keep_feeding: bool) {
    hwa::Contract::init_logger();
    hwa::Contract::init_heap();

    let c = instrumentation::machinery::init_splot(spawner).await;

    let event_bus = c.event_bus;
    let motion_planner = c.motion_planner;

    motion_planner.start(&event_bus).await;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            // Max speed
            motion_planner.motion_config().set_max_speed(
                hwa::make_vector_real!(x=200.0, y=200.0, z=200.0)
                //hwa::Contract::DEFAULT_MAX_SPEED_PS
            );

            motion_planner.motion_config().set_max_accel(
                hwa::make_vector_real!(x=1500.0, y=1500.0, z=1500.0)
                //hwa::Contract::DEFAULT_MAX_ACCEL_PS
            );

            motion_planner.motion_config().set_max_jerk(
                hwa::make_vector_real!(x=9810.0, y=9810.0, z=9810.0) * hwa::make_real!(2.0)
                //hwa::Contract::DEFAULT_MAX_JERK_PS
            );

            motion_planner.motion_config().set_default_travel_speed(
                hwa::Contract::DEFAULT_TRAVEL_SPEED_PS
            );

            motion_planner.motion_config().set_space_units_per_world_unit(hwa::Contract::DEFAULT_UNITS_PER_WU);

            motion_planner.motion_config().set_micro_steps_per_axis(
                hwa::make_vector!(x=16, y=16, z=16)
                //hwa::Contract::DEFAULT_MICRO_STEPS_PER_AXIS
            );

            motion_planner.motion_config().set_world_center(
                hwa::Contract::DEFAULT_WORLD_CENTER_WU
            );

            motion_planner.motion_config().set_world_size(
                hwa::Contract::DEFAULT_WORLD_SIZE_WU
            );

            motion_planner.motion_config().set_nozzle_offset(
                hwa::make_vector_real!(x=0.0, y=0.0,z=0.0)
            );

            motion_planner.motion_config().set_probe_offset(
                hwa::make_vector_real!(x=0.0, y=0.0,z=0.0)
            );

            motion_planner.motion_config().set_flow_rate(100);
            motion_planner.motion_config().set_speed_rate(100);

            // Compute min speed. Really useful because of discretion effects
            motion_planner.motion_config().compute_min_speed();

            // Make homing unneeded
            hwa::info!("Virtually homing");
            {
                let position = hwa::controllers::Position::new_with_world_projection(
                    &hwa::Contract::DEFAULT_WORLD_HOMING_POINT_WU,
                );
                motion_planner
                    .motion_status()
                    .update_last_planned_position(0, &position);
                motion_planner
                    .motion_status()
                    .update_current_position(0, &position);
            }
            #[cfg(all(feature = "native", feature = "with-ps-on"))]
            {
                hwa::info!("Virtually powering on");
                event_bus.publish_event(hwa::EventStatus::containing(hwa::EventFlags::ATX_ON)).await;
            }
        }
    }

    // Starting motion logic

    {
        let mut gcode_buff = instrumentation::gcode::GCodeBuffer::new();

        // standard case: 
        //gcode_buff.append("G0 X50\n");
        //gcode_buff.append("G0 X100\n");

        // other case: <- WIP


        gcode_buff.append("G0 X10\n");
        gcode_buff.append("G0 X20\n");
        gcode_buff.append("G0 X30\n");
        gcode_buff.append("G0 X40\n");
        gcode_buff.append("G0 X50\n");


        //gcode_buff.append("G0 Z2.2000 F3000; pen up\n");
        //gcode_buff.append("G0 X100.18164 Y105\n");
        //gcode_buff.append("G0 Z1.2000 F3000; pen down\n");



        /*
        gcode_buff.append("G92 X100.18164 Y105\n");
        gcode_buff.append("G1 X99.63526 Y104.99686 F7000 S0\n");
        gcode_buff.append("G1 X99.09117 Y104.98748 F7000 S0\n");
        gcode_buff.append("G1 X98.54937 Y104.97192 F7000 S0\n");
        gcode_buff.append("G1 X98.00986 Y104.95024 F7000 S0\n");
*/


        gcode_buff.append("G1 X97.47261 Y104.92249 F7000 S0\n");
        gcode_buff.append("G1 X96.93764 Y104.88873 F7000 S0\n");
        gcode_buff.append("G1 X96.40494 Y104.84902 F7000 S0\n");
        gcode_buff.append("G1 X95.87449 Y104.80339 F7000 S0\n");
        gcode_buff.append("G1 X95.34630 Y104.75190 F7000 S0\n");
        gcode_buff.append("G1 X94.82036 Y104.69461 F7000 S0\n");
        gcode_buff.append("G1 X94.29131 Y104.63086 F7000 S0\n");
        gcode_buff.append("G1 X93.76473 Y104.56131 F7000 S0\n");
        gcode_buff.append("G1 X93.24062 Y104.48598 F7000 S0\n");
        gcode_buff.append("G1 X92.71898 Y104.40494 F7000 S0\n");
        gcode_buff.append("G1 X92.19980 Y104.31823 F7000 S0\n");
        gcode_buff.append("G1 X91.68308 Y104.22590 F7000 S0\n");
        gcode_buff.append("G1 X91.16881 Y104.12798 F7000 S0\n");
        gcode_buff.append("G1 X90.65699 Y104.02452 F7000 S0\n");
        gcode_buff.append("G1 X90.14763 Y103.91555 F7000 S0\n");
        gcode_buff.append("G1 X89.64071 Y103.80113 F7000 S0\n");
        gcode_buff.append("G1 X89.13232 Y103.68032 F7000 S0\n");
        gcode_buff.append("G1 X88.62661 Y103.55409 F7000 S0\n");
        gcode_buff.append("G1 X88.12357 Y103.42248 F7000 S0\n");
        gcode_buff.append("G1 X87.62321 Y103.28552 F7000 S0\n");


        let mut parser =
            processing::GCodeLineParser::new(instrumentation::gcode::BufferStream::new(gcode_buff));

        loop {
            match parser.next_gcode(CommChannel::Internal).await {
                Ok(gcode) => {
                    if motion_planner
                        .plan(CommChannel::Internal, &gcode, false, &event_bus)
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
                Err(_error) => {
                    break;
                }
            }
        }
    }

    let micro_segment_period_secs: hwa::math::Real =
        hwa::math::Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 6).rdp(6);

    let _sampling_time: hwa::math::Real =
        hwa::math::Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US as i64, 6).rdp(6);

    let mut data_points = instrumentation::data_points::DataPoints::new();
    loop {
        match motion_planner.next_plan(&event_bus).await {
            controllers::motion_control::ExecPlan::Segment(mut segment, _channel, _order_num) => {
                let current_real_pos = motion_planner.motion_status().get_current_position();
                let position_offset = segment.src_pos - current_real_pos.space_pos;
                #[cfg(feature = "debug-motion")]
                hwa::info!(
                        "[task_stepper] order_num:{:?} Correcting offset [{:?}] {}",
                        _order_num,
                        position_offset,
                        hwa::Contract::SPACE_UNIT_MAGNITUDE,
                    );
                segment.fix_deviation(
                    &position_offset,
                    motion_planner.motion_config().get_flow_rate_as_real(),
                );

                hwa::info!(
                        "[s_plot] order_num:{:?} pre-profile speeds: s={:?} ain={:?} aout={:?} vin={:?} vout={:?} vcin={:?} vcout={:?} target={:?}",
                        _order_num,
                        segment.displacement_su,
                        {
                            #[cfg(feature = "with-motion-chaining")]
                            { segment.accel_enter_su_s }
                            #[cfg(not(feature = "with-motion-chaining"))]
                            { hwa::math::ZERO }
                        },
                        {
                            #[cfg(feature = "with-motion-chaining")]
                            { segment.accel_exit_su_s }
                            #[cfg(not(feature = "with-motion-chaining"))]
                            { hwa::math::ZERO }
                        },
                        segment.speed_enter_su_s,
                        segment.speed_exit_su_s,
                        segment.speed_enter_constrained_su_s,
                        segment.speed_exit_constrained_su_s,
                        segment.speed_target_su_s,
                    );

                match HybridMotionProfile::compute(
                    segment.displacement_su,
                    segment.speed_enter_su_s,
                    segment.speed_exit_su_s,
                    #[cfg(feature = "with-motion-chaining")]
                    segment.accel_enter_su_s,
                    #[cfg(feature = "with-motion-chaining")]
                    segment.accel_exit_su_s,
                    &segment.constraints,
                    false,
                ) {
                    Ok(trajectory) => {
                        #[cfg(feature = "debug-motion-planning")]
                        hwa::info!("profile: {:?}", trajectory);
                        // The relevant coords (those that will move)
                        let mut relevant_coords = hwa::math::CoordSel::empty();
                        // The relevant coords that will move forward
                        let mut relevant_coords_dir_fwd = hwa::math::CoordSel::empty();
                        segment.unit_vector_dir.foreach_values(|coord, val| {
                            relevant_coords.set(coord, !val.is_negligible());
                            relevant_coords_dir_fwd.set(coord, val.is_defined_positive());
                        });
                        hwa::debug!("Relevant coords: [{:?}]", relevant_coords);
                        hwa::debug!("Relevant coords_dir_fwd: [{:?}]", relevant_coords_dir_fwd);

                        let steps_per_su = motion_planner
                            .motion_config()
                            .get_units_per_space_magnitude()
                            * motion_planner.motion_config().get_micro_steps_as_vector();

                        let mut segment_iterator =
                            SegmentSampler::new(&trajectory, micro_segment_period_secs);

                        let mut micro_segment_interpolator = MicroSegmentInterpolator::new(
                            segment
                                .unit_vector_dir
                                .with_coord(relevant_coords.complement(), None)
                                .abs(),
                            segment.displacement_su,
                            steps_per_su.with_coord(relevant_coords.complement(), None),
                        );
                        #[cfg(feature = "debug-motion")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory interpolation START",
                            _order_num
                        );

                        // Micro-segments interpolation along segment
                        data_points.segment_starts(&trajectory);
                        let mut v_window_start = {
                            use motion::MotionProfile;
                            trajectory.eval_velocity(hwa::math::ZERO)
                        };
                        loop {
                            // Micro-segment start

                            if let Some(estimated_position) = segment_iterator.next() {
                                data_points.sampling_tick(&segment_iterator);

                                let w = hwa::math::Real::from_lit(segment_iterator.dt_us().max(1) as i64, 0);
                                let v_window_end = segment_iterator.current_speed();
                                let _has_more = micro_segment_interpolator
                                    .advance_to_with_velocity(
                                        estimated_position,
                                        w,
                                        v_window_start,
                                        v_window_end,
                                        STEPPER_PLANNER_CLOCK_PERIOD_US,
                                    )
                                    .expect("bad advance");
                                v_window_start = v_window_end;

                                #[cfg(feature = "debug-motion")]
                                hwa::info!(
                                    "[task_stepper] segment:{:?}|{:?} Trajectory micro-segment advanced: {:?} [{:?}] {} [{:?}] steps",
                                    data_points.current_segment_id(),
                                    data_points.current_micro_segment_id(),
                                    segment_iterator.ds(),
                                    micro_segment_interpolator
                                        .advanced_units()
                                        .map_nan_coords(relevant_coords, &hwa::math::ZERO),
                                    hwa::Contract::SPACE_UNIT_MAGNITUDE,
                                    micro_segment_interpolator
                                        .advanced_steps()
                                        .map_nan_coords(relevant_coords, &0),
                                );

                                #[cfg(feature = "verbose-timings")]
                                let _t1 = embassy_time::Instant::now();

                                let _ = micro_segment_interpolator.state().clone();
                                let _ = relevant_coords;
                                data_points.micro_segment_ends();
                                if !_has_more {
                                    break;
                                }
                            } else {
                                // No advance
                                break;
                            }
                        }
                        data_points.segment_ends();
                        ////
                        //// TRAJECTORY INTERPOLATION END
                        ////
                        #[cfg(feature = "debug-motion")]
                        hwa::debug!(
                            "[task_stepper] order_num:{:?} Trajectory interpolation END",
                            _order_num
                        );

                        let _adv_steps = micro_segment_interpolator.advanced_steps();
                        let adv_pos = micro_segment_interpolator.advanced_units();
                        //#[cfg(feature = "debug-motion")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} [{:?}] Trajectory advanced. vector displacement space: {:?} [{:#?}] {}, velocity: <v_i={:?} v_lim={:?} v_o={:?}> {}/s, accel: <a_i={:?} a_o={:?}> {}/s^2",
                            _order_num,
                            trajectory.kind,
                            segment_iterator.current_position(),
                            adv_pos,
                            hwa::Contract::SPACE_UNIT_MAGNITUDE,
                            trajectory.v_i,
                            trajectory.v_lim,
                            trajectory.v_o,
                            hwa::Contract::SPACE_UNIT_MAGNITUDE,
                            trajectory.a_i,
                            trajectory.a_o,
                            hwa::Contract::SPACE_UNIT_MAGNITUDE
                        );
                        #[cfg(feature = "debug-motion")]
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory advanced. vector displacement space: [{:#?}] steps",
                            _order_num,
                            _adv_steps.excluding_negligible()
                        );

                        let adv_delta = segment.unit_vector_dir.sign() * adv_pos;
                        let next_real_pos = (current_real_pos.space_pos
                            + adv_delta.map_nan(&hwa::math::ZERO))
                        .rdp(6);

                        let mut pos = current_real_pos.clone();
                        pos.update_from_space_coordinates(&next_real_pos);
                        motion_planner
                            .motion_status()
                            .update_current_position(_order_num, &pos);
                    }
                    Err(_e) => {
                        hwa::error!("Unable to compute motion plan. Discarding. err: {:?}. Segment: {:?}", _e, segment);
                    }
                }
            }
            controllers::motion_control::ExecPlan::SetPosition(position, _channel, _order_num) => {
                motion_planner
                    .motion_status()
                    .update_current_position(_order_num, &position);
            }
            _ => {}
        }
        motion_planner.consume_current_plan(&event_bus).await;
        if motion_planner.num_queued().await == 0 {
            break;
        }
    }

    {
        #[allow(unused)]
        use gnuplot::{
            AutoOption, MultiplotFillDirection::Downwards, MultiplotFillOrder::RowsFirst, Tick,
        };
        use gnuplot::{AxesCommon, Figure};
        #[allow(unused)]
        use gnuplot::{DashType, PlotOption};

        hwa::info!("Writing data points to s_plot.csv...");
        let _ = data_points.write_csv("s_plot.csv");

        let mut fg = Figure::new();

        cfg_if::cfg_if! {
            if #[cfg(feature="float-point-f64-impl")] {
                const MATH_PRECISION: &str = "[float point 64bits]";
            }
            else if #[cfg(feature="fixed-point-128-impl")] {
                const MATH_PRECISION: &str = "[fixed point 128bits]";
            }
            else {
                const MATH_PRECISION: &str = "[float point 32bits]";
            }
        }
        let min_position = data_points.min_position().unwrap_or(0.0).floor();
        let max_position = data_points.max_position().unwrap_or(0.0).ceil();
        let position_margin = (max_position - min_position) * 0.10;
        let min_velocity = data_points.min_velocity().unwrap_or(0.0);
        let max_velocity = data_points.max_velocity().unwrap_or(0.0);
        let velocity_margin = (max_velocity - min_velocity) * 0.10;
        let min_acceleration = data_points.min_acceleration().unwrap_or(0.0);
        let max_acceleration = data_points.max_acceleration().unwrap_or(0.0);
        let acceleration_margin = (max_acceleration - min_acceleration) * 0.10;

        fg.set_multiplot_layout(3, 1)
            .set_title(
                format!(
                    "{} Hybrid S-Curve/Quintic velocity profile\n[{:?} segments, {:?} {} displacement, {:?} hz interp, {:?} hz sampling]",
                    MATH_PRECISION,
                    data_points.num_segments(),
                    data_points.total_displacement(),
                    hwa::Contract::SPACE_UNIT_MAGNITUDE,
                    hwa::Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY,
                    hwa::Contract::STEP_PLANNER_CLOCK_FREQUENCY,
                ).as_str()
            )
            .set_scale(1.0, 1.0)
            .set_offset(0.0, 0.0)
            .set_multiplot_fill_order(RowsFirst, Downwards);

        // Position panel
        {
            let position_panel = fg.axes2d();
            position_panel.set_y_label(
                format!("Position ({})", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[]
            );
            for instant in data_points.segment_marks.iter() {
                position_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_position - position_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_position + position_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("black".into()), PlotOption::ArrowSize(0.5)]);
            }
            for instant in data_points.interval_marks.iter() {
                position_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_position - position_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_position + position_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::ArrowSize(0.5)]);
            }
            position_panel.lines(
                data_points.sampled_positions.times.clone(), data_points.sampled_positions.points.clone(),
                &[PlotOption::Color("blue".into())]
            );
            position_panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(data_points.reference_time));
            position_panel.set_y_range(AutoOption::Fix(min_position - position_margin), AutoOption::Fix(max_position + position_margin));
        }

        // Velocity panel
        {
            let velocity_panel = fg.axes2d();
            velocity_panel.set_y_label(
                format!("Velocity ({}/s)", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[]
            );
            for instant in data_points.segment_marks.iter() {
                velocity_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_velocity - velocity_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_velocity + velocity_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("black".into()), PlotOption::ArrowSize(0.5)]);
            }
            for instant in data_points.interval_marks.iter() {
                velocity_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_velocity - velocity_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_velocity + velocity_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::ArrowSize(0.5)]);
            }
            velocity_panel.lines(data_points.sampled_velocities.times.clone(), data_points.sampled_velocities.points.clone(), &[PlotOption::Color("green".into())]);
            velocity_panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(data_points.reference_time));
            velocity_panel.set_y_range(AutoOption::Fix(min_velocity - velocity_margin), AutoOption::Fix(max_velocity + velocity_margin));
        }

        // Acceleration panel
        {
            let acceleration_panel = fg.axes2d();
            acceleration_panel.set_y_label(
                format!("Acceleration ({}/s^2)", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[]
            );
            for instant in data_points.segment_marks.iter() {
                acceleration_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_acceleration - acceleration_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_acceleration + acceleration_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("black".into()), PlotOption::ArrowSize(0.5)]);
            }
            for instant in data_points.interval_marks.iter() {
                acceleration_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_acceleration - acceleration_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_acceleration + acceleration_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::ArrowSize(0.5)]);
            }
            acceleration_panel.lines(data_points.sampled_accelerations.times.clone(), data_points.sampled_accelerations.points.clone(), &[PlotOption::Color("red".into())]);
            acceleration_panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(data_points.reference_time));
            acceleration_panel.set_y_range(AutoOption::Fix(min_acceleration - acceleration_margin), AutoOption::Fix(max_acceleration + acceleration_margin));
        }


        #[cfg(not(test))]
        match fg.show_and_keep_running() {
            Err(_err) => {hwa::error!("Unable to show plot: {:?}", _err)}
            _ => {}
        }
        _ = fg.save_to_pdf("plot.pdf", 10.0f32, 10.0f32);
    }

    #[cfg(not(test))]
    std::process::exit(0);
}

//#region "Machinery initialization"

pub fn initialization_error() {
    let msg = "Unable to start because SYS_ALARM raised at startup. Giving up...";
    hwa::error!("{}", msg);
    panic!("{}", msg);
}

//#endregion

//#region "Tests"

#[cfg(feature = "s-plot-bin")]
#[cfg(test)]
mod test {
    use super::*;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use printhor_hwa_common::PersistentState;
    use std::marker::PhantomData;
    use std::sync::{Condvar, Mutex};

    struct Signaler {
        mutex: Mutex<bool>,
        condvar: Condvar,
    }

    impl Signaler {
        fn new() -> Self {
            Self {
                mutex: Mutex::new(false),
                condvar: Condvar::new(),
            }
        }

        fn wait(&self) {
            let mut signaled = self.mutex.lock().unwrap();
            while !*signaled {
                signaled = self.condvar.wait(signaled).unwrap();
            }
            *signaled = false;
        }
    }

    pub static SPLOT_STATE: PersistentState<CriticalSectionRawMutex, bool> = PersistentState::new();

    pub struct MockedExecutor {
        inner: embassy_executor::raw::Executor,
        not_send: PhantomData<*mut ()>,
        signaler: &'static Signaler,
    }

    impl MockedExecutor {
        /// Create a new Executor.
        pub fn new() -> Self {
            let signaler = Box::leak(Box::new(Signaler::new()));
            Self {
                inner: embassy_executor::raw::Executor::new(signaler as *mut Signaler as *mut ()),
                not_send: PhantomData,
                signaler,
            }
        }

        pub fn run(&'static mut self, init: impl FnOnce(embassy_executor::Spawner)) {
            init(self.inner.spawner());

            loop {
                unsafe { self.inner.poll() };
                if SPLOT_STATE.signaled() {
                    break;
                }
                self.signaler.wait()
            }
        }
    }

    /// The integration test entry-point
    #[embassy_executor::task(pool_size = 1)]
    async fn mocked_splot_main(spawner: embassy_executor::Spawner) {
        printhor_main(spawner, false).await;
        SPLOT_STATE.signal(true);
    }

    #[test]
    fn splot_test() {
        let executor = hwa::make_static_ref!("Executor", MockedExecutor, MockedExecutor::new());
        // 2. Spawn the [mocked_main] task
        executor.run(|spawner| {
            let _tk = spawner.spawn(mocked_splot_main(spawner).unwrap());
        });
    }
}

//#endregion
