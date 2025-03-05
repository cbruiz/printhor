extern crate alloc;
extern crate core;

pub mod control;
pub mod helpers;
pub mod hwa;

mod instrumentation;

use hwa::HwiContract;

#[allow(unused)]
use hwa::RawHwiResource;

use control::motion;
use motion::SCurveMotionProfile;
use control::task_stepper::{
    STEPPER_PLANNER_CLOCK_PERIOD_US, STEPPER_PLANNER_MICROSEGMENT_PERIOD_US,
};
use hwa::controllers;
use controllers::{LinearMicrosegmentStepInterpolator, SegmentIterator};
use printhor_hwa_common::CommChannel;

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
            motion_planner.motion_config().set_max_speed(hwa::Contract::DEFAULT_MAX_SPEED_PS);

            motion_planner.motion_config().set_max_accel(hwa::Contract::DEFAULT_MAX_ACCEL_PS);

            motion_planner.motion_config().set_max_jerk(hwa::Contract::DEFAULT_MAX_JERK_PS);

            motion_planner.motion_config().set_default_travel_speed(hwa::Contract::DEFAULT_TRAVEL_SPEED_PS);

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
        gcode_buff.append("G0 Z2.2000 F3000; pen up\n");
        gcode_buff.append("G0 X100.18164 Y105\n");
        gcode_buff.append("G0 Z1.2000 F3000; pen down\n");
        gcode_buff.append("G1 X99.63526 Y104.99686 F7000 S0\n");
        gcode_buff.append("G1 X99.09117 Y104.98748 F7000 S0\n");
        gcode_buff.append("G1 X98.54937 Y104.97192 F7000 S0\n");
        gcode_buff.append("G1 X98.00986 Y104.95024 F7000 S0\n");
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

        let mut parser = control::GCodeLineParser::new(
            instrumentation::gcode::BufferStream::new(gcode_buff)
        );

        loop {
            match parser.next_gcode(CommChannel::Internal).await {
                Ok(gcode) => {
                    if motion_planner.plan(CommChannel::Internal, &gcode, false, &event_bus).await.is_err() {
                        break
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

    let mut data_points = instrumentation::datapoints::DataPoints::new();
    loop {
        match motion_planner.next_plan(&event_bus).await {
            controllers::motion::ExecPlan::Segment(mut segment, _channel, _order_num) => {
                let current_real_pos = motion_planner.motion_status().get_current_position();
                let position_offset = segment.src_pos - current_real_pos.space_pos;
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
                match SCurveMotionProfile::compute(
                    segment.displacement_su,
                    segment.speed_enter_su_s,
                    segment.speed_exit_su_s,
                    &segment.constraints,
                    false,
                ) {
                    Ok(trajectory) => {
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
                            SegmentIterator::new(&trajectory, micro_segment_period_secs);

                        let mut micro_segment_interpolator =
                            LinearMicrosegmentStepInterpolator::new(
                                segment
                                    .unit_vector_dir
                                    .with_coord(relevant_coords.complement(), None)
                                    .abs(),
                                segment.displacement_su,
                                steps_per_su.with_coord(relevant_coords.complement(), None),
                            );
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory interpolation START",
                            _order_num
                        );

                        // Micro-segments interpolation along segment
                        data_points.segment_starts(&trajectory);
                        loop {
                            // Micro-segment start

                            if let Some(estimated_position) = segment_iterator.next() {
                                data_points.interpolation_tick(&segment_iterator);

                                let w = (segment_iterator.dt() * hwa::math::ONE_MILLION).round();
                                if w.is_negligible() {
                                    hwa::info!("giving up for any reason");
                                    break;
                                }
                                let _has_more =
                                    micro_segment_interpolator.advance_to(estimated_position, w);

                                hwa::debug!(
                                    "[task_stepper] segment:{:?}|{:?} Trajectory micro-segment advanced: {:?} {} {:?} steps",
                                    data_points.current_segment_id(),
                                    data_points.current_micro_segment_id(),
                                    micro_segment_interpolator
                                        .advanced_units()
                                        .map_nan_coords(relevant_coords, &hwa::math::ZERO),
                                    hwa::Contract::SPACE_UNIT_MAGNITUDE,
                                    micro_segment_interpolator
                                        .advanced_steps()
                                        .map_nan_coords(relevant_coords, &0),
                                );

                                //total_disp += micro_segment_interpolator.advanced_units().norm2().unwrap();
                                //total_disp_discrete += micro_segment_interpolator.advanced_steps().norm2().unwrap();

                                #[cfg(feature = "verbose-timings")]
                                let t1 = embassy_time::Instant::now();

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
                        hwa::info!(
                            "[task_stepper] order_num:{:?} Trajectory advanced. vector displacement space: [{:#?}] {}, vlim: {:?} {}/s",
                            _order_num,
                            adv_pos,
                            hwa::Contract::SPACE_UNIT_MAGNITUDE,
                            trajectory.v_lim,
                            hwa::Contract::SPACE_UNIT_MAGNITUDE
                        );

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
                        motion_planner.motion_status().update_current_position(_order_num, &pos);
                    }
                    _ => {
                        hwa::error!("Unable to compute motion plan. Discarding...");
                    }
                }
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

        fg.set_multiplot_layout(2, 1)
            .set_title(
                format!(
                    "{} Double S-Curve velocity profile\n[{:?} segments, {:?} {} displacement, {:?} hz interp, {:?} hz sampling]",
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

        fg.axes2d()
            .set_y_label(format!("Position ({})", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[])
            .points(data_points.segment_position_marks.times, data_points.segment_position_marks.points, &[PlotOption::Color("black")])
            .lines(data_points.interpolated_positions.times, data_points.interpolated_positions.points, &[PlotOption::Color("blue")])
            //.lines(data_points.time_discrete, data_points.pos_discrete, &[PlotOption::Color("gray")])
        ;
        fg.axes2d()
            .set_y_label(format!("Velocity ({}/s)", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[])
            .points(data_points.segment_velocity_marks.times, data_points.segment_velocity_marks.points, &[PlotOption::Color("black")])
            //.lines(data_points.time_discrete_acc, data_points.acc_discrete, &[PlotOption::Color("red")])
            //.lines(data_points.time, data_points.spd, &[PlotOption::Color("green")])
            //.lines(data_points.time_discrete_deriv, data_points.spd_discrete, &[PlotOption::Color("gray")])
        ;
        #[cfg(not(test))]
        fg.show_and_keep_running().unwrap();
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
            let _tk = spawner.must_spawn(mocked_splot_main(spawner));
        });
    }
}

//#endregion
