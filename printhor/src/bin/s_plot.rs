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

    let event_bus = c.event_bus.clone();
    let motion_planner = c.motion_planner.clone();

    motion_planner.start(&event_bus).await;

    cfg_if::cfg_if! {
        if #[cfg(feature = "with-motion")] {
            // The motion defaults of the SKR Mini E3 V3 as an FDM printer (`skr_mini_e3_v3_fdm`, the
            // cartesian branch of hwi-boards/printhor-hwi_skr_mini_e3/src/board_stm32g0/mod.rs: keep
            // them in sync), so that the plot shows what that machine executes. The X/Y limits can
            // be overridden to tune them: S_PLOT_SPEED_XY (mm/s), S_PLOT_ACCEL_XY (mm/s²) and
            // S_PLOT_JERK_XY (mm/s³); the jerk is also set by the job (M800 J). S_PLOT_CORNER_DV
            // (mm/s) overrides the largest velocity jump at a junction (M205 X).
            // To reproduce the board exactly, build with its motion constants, axes and chaining:
            //   SEGMENT_QUEUE_SIZE=40 MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY=100 \
            //   STEP_PLANNER_CLOCK_FREQUENCY=50000 cargo run --profile release --bin s_plot \
            //   --no-default-features --features native,cartessian-fdm,s-plot-bin,with-motion-chaining
            let env_f32 = |name: &str, default: f32| -> f32 {
                std::env::var(name).ok().and_then(|v| v.parse::<f32>().ok()).unwrap_or(default)
            };
            let (speed_xy, accel_xy, jerk_xy) = (
                env_f32("S_PLOT_SPEED_XY", 600.0),
                env_f32("S_PLOT_ACCEL_XY", 3000.0),
                env_f32("S_PLOT_JERK_XY", 40000.0),
            );
            hwa::info!("[s_plot] X/Y limits: {} mm/s, {} mm/s², {} mm/s³", speed_xy, accel_xy, jerk_xy);
            let with_xy = |mut v: hwa::math::TVector<hwa::math::Real>, xy: f32| {
                v.set_coord(hwa::CoordSel::X | hwa::CoordSel::Y, Some(hwa::math::Real::from_f32(xy)));
                v
            };
            let cfg = motion_planner.motion_config();

            cfg.set_max_speed(with_xy(hwa::make_vector_real!(x=600.0, y=600.0, z=20.0, e=100.0), speed_xy));
            cfg.set_max_accel(with_xy(hwa::make_vector_real!(x=3000.0, y=3000.0, z=500.0, e=3000.0), accel_xy));
            cfg.set_max_jerk(with_xy(hwa::make_vector_real!(x=40000.0, y=40000.0, z=20000.0, e=200000.0), jerk_xy));
            cfg.set_corner_speed_change(hwa::math::Real::from_f32(env_f32("S_PLOT_CORNER_DV", 7.0)));
            cfg.set_default_travel_speed(hwa::make_real!(300.0));

            // Full steps per mm and micro-steps: 80 steps/mm on X and Y, 400 on Z and E
            cfg.set_space_units_per_world_unit(hwa::make_vector_real!(x=10.0, y=10.0, z=50.0, e=50.0));
            cfg.set_micro_steps_per_axis(hwa::make_vector!(x=8, y=8, z=8, e=8));

            cfg.set_world_center(hwa::make_vector_real!(x=105.0, y=105.0, z=105.0));
            cfg.set_world_size(hwa::make_vector_real!(x=237.3, y=223.0, z=220.0));
            cfg.set_nozzle_offset(hwa::make_vector_real!(x=0.0, y=0.0, z=0.0));
            cfg.set_probe_offset(hwa::make_vector_real!(x=33.0, y=-4.0, z=-3.6));

            cfg.set_flow_rate(100);
            cfg.set_speed_rate(100);

            // Compute min speed. Really useful because of discretion effects
            cfg.compute_min_speed();

            // Make homing unneeded: at the homing point of the board (X -17.3, Y -21, Z 1)
            hwa::info!("Virtually homing");
            {
                let position = hwa::controllers::Position::new_with_world_projection(
                    &hwa::make_vector_real!(x=-17.3, y=-21.0, z=1.0),
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

    // The G-code to plot: the whole pen-test job by default, or the file in S_PLOT_GCODE
    let gcode_buff = {
        let path = std::env::var("S_PLOT_GCODE")
            .unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/../pen-test.gcode").to_string());
        let mut b = instrumentation::gcode::GCodeBuffer::new();
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                hwa::info!("[s_plot] G-code: {}", path.as_str());
                b.append(&text);
            }
            Err(_e) => hwa::error!("[s_plot] Unable to read {}: {:?}", path.as_str(), _e),
        }
        b
    };

    // Producer: parses and plans the job while the consumer below executes it (the motion queue
    // holds a few segments only). The commands that change the trajectory are applied as the
    // firmware does (M206 shifts the coordinates, M800 J sets the jerk); the others are skipped.
    let producer_done = core::sync::atomic::AtomicBool::new(false);
    // The producer waits for the queue to drain (M206): the consumer must not wait for it to fill
    let producer_draining = core::sync::atomic::AtomicBool::new(false);
    let producer = async {
        let mut parser =
            processing::GCodeLineParser::new(instrumentation::gcode::BufferStream::new(gcode_buff));
        let mut _planned = 0u32;
        loop {
            match parser.next_gcode(CommChannel::Internal).await {
                Ok(gcode) => match &gcode.value {
                    processing::GCodeValue::M206(offset) => {
                        let cfg = motion_planner.motion_config();
                        let v: hwa::math::TVector<hwa::math::Real> = offset.into();
                        let before = cfg.get_home_offset();
                        cfg.update_home_offset(&v);
                        let delta = cfg.get_home_offset() - before;
                        // After the queued motion, as the firmware
                        producer_draining.store(true, core::sync::atomic::Ordering::Release);
                        while motion_planner.num_queued().await > 0 {
                            embassy_time::Timer::after_millis(1).await;
                        }
                        producer_draining.store(false, core::sync::atomic::Ordering::Release);
                        let status = motion_planner.motion_status();
                        let shift = |p: controllers::Position| {
                            let mut world = p.world_pos;
                            delta.foreach_values(|c, d| {
                                if let Some(v) = world.get_coord(c) {
                                    world.set_coord(c, Some(v + *d));
                                }
                            });
                            controllers::Position::new_with_world_projection(&world)
                        };
                        status.update_last_planned_position(gcode.order_num, &shift(status.get_last_planned_position()));
                        status.update_current_position(gcode.order_num, &shift(status.get_current_position()));
                    }
                    processing::GCodeValue::M800(param) => {
                        if let Some(j) = param.j.filter(|j| j.is_defined_positive()) {
                            let cfg = motion_planner.motion_config();
                            let mut jerk = cfg.get_max_jerk();
                            jerk.assign_if_set(
                                hwa::CoordSel::motion_relevant_axis(),
                                &jerk.map_values(|_, _| Some(j)),
                            );
                            cfg.set_max_jerk(jerk);
                        }
                    }
                    _ => {
                        // Blocking: waits for room in the queue. Non-motion commands are rejected
                        if motion_planner
                            .plan(CommChannel::Internal, &gcode, true, &event_bus)
                            .await
                            .is_ok()
                        {
                            _planned += 1;
                        }
                    }
                },
                Err(processing::GCodeLineParserError::EOF) => break,
                Err(_e) => hwa::warn!("[s_plot] G-code skipped: {:?}", _e),
            }
        }
        hwa::info!("[s_plot] {} commands planned", _planned);
        producer_done.store(true, core::sync::atomic::Ordering::Release);
        // A final dwell, planned after the flag: the consumer always sees the end
        let mut tail = instrumentation::gcode::GCodeBuffer::new();
        tail.append("G4 P0\n");
        let mut parser = processing::GCodeLineParser::new(instrumentation::gcode::BufferStream::new(tail));
        if let Ok(gcode) = parser.next_gcode(CommChannel::Internal).await {
            let _ = motion_planner.plan(CommChannel::Internal, &gcode, true, &event_bus).await;
        }
    };

    let micro_segment_period_secs: hwa::math::Real =
        hwa::math::Real::from_lit(STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as i64, 6).rdp(6);

    let _sampling_time: hwa::math::Real =
        hwa::math::Real::from_lit(STEPPER_PLANNER_CLOCK_PERIOD_US as i64, 6).rdp(6);

    let mut data_points = instrumentation::data_points::DataPoints::new();
    // Velocity of each axis at every window end: the motors follow it linearly inside the window
    // (second-order DDA), the input of the resonance analysis (see [Resonance])
    let mut window_ends: Vec<(f64, [f64; 3])> = Vec::new();
    // Finer sampling of the axes CSV (see below)
    let axes_dt_s: Option<f64> = std::env::var("S_PLOT_AXES_DT_US")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0)
        .map(|v| v * 1e-6);
    let mut deviation = StepDeviation::default();
    let consumer = async {
    loop {
        // As a machine whose planner runs ahead of the motion: the next segment is only taken with
        // the queue full (or the job planned). Without it, the consumer (no real time here) would
        // drain the whole queue at once and execute its tail, always planned to stop
        while !producer_done.load(core::sync::atomic::Ordering::Acquire)
            && !producer_draining.load(core::sync::atomic::Ordering::Acquire)
            && motion_planner.num_queued().await + 1 < hwa::Contract::SEGMENT_QUEUE_SIZE
        {
            embassy_futures::yield_now().await;
        }
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
                segment.fix_deviation(&position_offset);

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
                                // The same instant in machine axes: the segment origin plus its
                                // direction times the displacement, speed and acceleration. With
                                // S_PLOT_AXES_DT_US, the profile is also evaluated every that many
                                // µs inside the window (analysis only: the motion is unchanged)
                                let (t_end, dt) = (
                                    segment_iterator.current_time().to_f64(),
                                    segment_iterator.dt().to_f64(),
                                );
                                let n_sub = match axes_dt_s {
                                    Some(sub) if dt > sub => (dt / sub).ceil() as usize,
                                    _ => 1,
                                };
                                for k in 1..=n_sub {
                                    use hwa::CoordSel;
                                    use motion::MotionProfile;
                                    let frac = k as f64 / n_sub as f64;
                                    let (s, v, a) = if k == n_sub {
                                        (
                                            segment_iterator.current_position().to_f64(),
                                            segment_iterator.current_speed().to_f64(),
                                            segment_iterator.current_acceleration().to_f64(),
                                        )
                                    } else {
                                        let t = hwa::math::Real::from_f32((t_end - dt * (1.0 - frac)) as f32);
                                        (
                                            trajectory.eval_position(t).to_f64(),
                                            trajectory.eval_velocity(t).to_f64(),
                                            trajectory.eval_acceleration(t).to_f64(),
                                        )
                                    };
                                    let mut p = [0.0f64; 3];
                                    let (mut vv, mut aa) = ([0.0f64; 3], [0.0f64; 3]);
                                    for (k, c) in [CoordSel::X, CoordSel::Y, CoordSel::Z].into_iter().enumerate() {
                                        // The segment origin of the moving axes, the real
                                        // position of the others (not part of the segment)
                                        let o = segment
                                            .src_pos
                                            .get_coord(c)
                                            .or(current_real_pos.space_pos.get_coord(c))
                                            .map_or(0.0, |x| x.to_f64());
                                        let d = segment.unit_vector_dir.get_coord(c).map_or(0.0, |x| x.to_f64());
                                        p[k] = o + d * s;
                                        vv[k] = d * v;
                                        aa[k] = d * a;
                                    }
                                    data_points.axes.push(data_points.reference_time - dt * (1.0 - frac), p, vv, aa);
                                    if k == n_sub {
                                        window_ends.push((data_points.reference_time, vv));
                                    }
                                }

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

                        // Deviation of the real (stepped) position from the ideal end of the segment,
                        // in steps of each axis that moved. The real position must also stay on
                        // the step grid (no drift of the float accumulation)
                        {
                            use hwa::CoordSel;
                            for (k, c) in [CoordSel::X, CoordSel::Y, CoordSel::Z].into_iter().enumerate() {
                                if let (Some(real), Some(ideal), Some(spu)) = (
                                    next_real_pos.get_coord(c),
                                    segment.dest_pos.get_coord(c),
                                    steps_per_su.get_coord(c),
                                ) {
                                    let spu = spu.to_f64();
                                    let real_steps = real.to_f64() * spu;
                                    deviation.record(
                                        k,
                                        real_steps - ideal.to_f64() * spu,
                                        real_steps - real_steps.round(),
                                    );
                                }
                            }
                        }

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
            controllers::motion_control::ExecPlan::Homing(_channel, _order_num) => {
                // Virtual homing: where the firmware ends it (see the stepper task)
                let position = controllers::Position::new_with_world_projection(
                    &motion_planner.motion_config().get_post_homing_position(),
                );
                motion_planner
                    .motion_status()
                    .update_current_position(_order_num, &position);
            }
            _ => {}
        }
        motion_planner.consume_current_plan(&event_bus).await;
        if producer_done.load(core::sync::atomic::Ordering::Acquire)
            && motion_planner.num_queued().await == 0
        {
            break;
        }
    }
    };
    // Drains the deferred command events (the firmware's task_defer): otherwise the channel fills
    // up and both the planner and the consumer block on it
    let defer_drain = async {
        while !producer_done.load(core::sync::atomic::Ordering::Acquire)
            || motion_planner.num_queued().await > 0
        {
            let _ = embassy_futures::select::select(
                c.defer_channel.receive(),
                embassy_time::Timer::after_millis(10),
            )
            .await;
        }
        while c.defer_channel.try_receive().is_ok() {}
    };
    embassy_futures::join::join3(producer, consumer, defer_drain).await;
    deviation.report();
    let resonance = Resonance::analyze(&window_ends, Resonance::band_from_env());
    match &resonance {
        Some(r) => r.report(),
        None => hwa::warn!("[s_plot] Resonance: not enough motion to analyze"),
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
        // Segment and interval marks. gnuplot draws them (arrows) in front of the curves: where
        // the segments are short (a circle of 0.55 mm segments) they made a solid block that hid
        // the curves. Light gray, and only those at least 0.25% of the duration apart
        let sparse = |marks: &[f64]| -> Vec<f64> {
            let min_gap = data_points.reference_time * 0.0025;
            let mut kept: Vec<f64> = Vec::new();
            for m in marks {
                if kept.last().is_none_or(|last| m - last >= min_gap) {
                    kept.push(*m);
                }
            }
            kept
        };
        let segment_marks = sparse(&data_points.segment_marks);
        let interval_marks = sparse(&data_points.interval_marks);
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
            for instant in segment_marks.iter() {
                position_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_position - position_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_position + position_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray60".into()), PlotOption::ArrowSize(0.5)]);
            }
            for instant in interval_marks.iter() {
                position_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_position - position_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_position + position_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray85".into()), PlotOption::ArrowSize(0.5)]);
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
            for instant in segment_marks.iter() {
                velocity_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_velocity - velocity_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_velocity + velocity_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray60".into()), PlotOption::ArrowSize(0.5)]);
            }
            for instant in interval_marks.iter() {
                velocity_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_velocity - velocity_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_velocity + velocity_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray85".into()), PlotOption::ArrowSize(0.5)]);
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
            for instant in segment_marks.iter() {
                acceleration_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_acceleration - acceleration_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_acceleration + acceleration_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray60".into()), PlotOption::ArrowSize(0.5)]);
            }
            for instant in interval_marks.iter() {
                acceleration_panel.arrow(
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(min_acceleration - acceleration_margin),
                    gnuplot::Coordinate::Axis(*instant),
                    gnuplot::Coordinate::Axis(max_acceleration + acceleration_margin),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray85".into()), PlotOption::ArrowSize(0.5)]);
            }
            acceleration_panel.lines(data_points.sampled_accelerations.times.clone(), data_points.sampled_accelerations.points.clone(), &[PlotOption::Color("red".into())]);
            acceleration_panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(data_points.reference_time));
            acceleration_panel.set_y_range(AutoOption::Fix(min_acceleration - acceleration_margin), AutoOption::Fix(max_acceleration + acceleration_margin));
        }


        // S_PLOT_NO_SHOW: only the files (batch runs, e.g. tuning sweeps)
        #[cfg(not(test))]
        if std::env::var("S_PLOT_NO_SHOW").is_err()
        {
            match fg.show_and_keep_running() {
                Err(_err) => {hwa::error!("Unable to show plot: {:?}", _err)}
                _ => {}
            }
        }
        _ = fg.save_to_pdf("plot.pdf", 10.0f32, 10.0f32);

        // Second figure: the motion in machine axes, and its vector magnitudes
        hwa::info!("Writing axes data points to s_plot_axes.csv...");
        let _ = data_points.axes.write_csv("s_plot_axes.csv");
        let axes = &data_points.axes;
        let range = |series: &[&Vec<f64>]| {
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            for v in series.iter().flat_map(|s| s.iter()) {
                lo = lo.min(*v);
                hi = hi.max(*v);
            }
            if lo > hi { (0.0, 1.0) } else { let m = (hi - lo).max(1e-9) * 0.1; (lo - m, hi + m) }
        };
        let names = ["X", "Y", "Z"];
        let colors = ["blue", "red", "green"];
        let mut fa = Figure::new();
        let resonance_title = match &resonance {
            Some(r) => format!(
                ", resonance band {:.0}-{:.0} Hz: {:.1} {}/s^2 RMS (X {:.1}, Y {:.1})",
                r.band.0, r.band.1, r.rms_xy, hwa::Contract::SPACE_UNIT_MAGNITUDE, r.rms[0], r.rms[1]
            ),
            None => String::new(),
        };
        fa.set_multiplot_layout(if resonance.is_some() { 4 } else { 3 }, 1)
            .set_title(
                format!(
                    "{} Motion in machine axes\n[{:?} segments, {:.1} s{}]",
                    MATH_PRECISION,
                    data_points.num_segments(),
                    data_points.reference_time,
                    resonance_title,
                ).as_str()
            )
            .set_scale(1.0, 1.0)
            .set_offset(0.0, 0.0)
            .set_multiplot_fill_order(RowsFirst, Downwards);
        {
            let panel = fa.axes2d();
            panel.set_y_label(format!("Position ({})", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[]);
            for k in 0..3 {
                panel.lines(axes.times.clone(), axes.positions[k].clone(),
                    &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
            }
            let (lo, hi) = range(&[&axes.positions[0], &axes.positions[1], &axes.positions[2]]);
            panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(data_points.reference_time));
            panel.set_y_range(AutoOption::Fix(lo), AutoOption::Fix(hi));
        }
        {
            let panel = fa.axes2d();
            panel.set_y_label(format!("Velocity ({}/s)", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[]);
            for k in 0..3 {
                panel.lines(axes.times.clone(), axes.velocities[k].clone(),
                    &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
            }
            panel.lines(axes.times.clone(), axes.speed.clone(),
                &[PlotOption::Color("black".into()), PlotOption::Caption("|v|")]);
            let (lo, hi) = range(&[&axes.velocities[0], &axes.velocities[1], &axes.velocities[2], &axes.speed]);
            panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(data_points.reference_time));
            panel.set_y_range(AutoOption::Fix(lo), AutoOption::Fix(hi));
        }
        {
            let panel = fa.axes2d();
            panel.set_y_label(format!("Acceleration ({}/s^2)", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[]);
            for k in 0..3 {
                panel.lines(axes.times.clone(), axes.accelerations[k].clone(),
                    &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
            }
            panel.lines(axes.times.clone(), axes.total_acceleration.clone(),
                &[PlotOption::Color("black".into()), PlotOption::Caption("|dv/dt| (with direction changes)")]);
            let (lo, hi) = range(&[&axes.accelerations[0], &axes.accelerations[1], &axes.accelerations[2], &axes.total_acceleration]);
            panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(data_points.reference_time));
            panel.set_y_range(AutoOption::Fix(lo), AutoOption::Fix(hi));
        }
        // Spectrum of the acceleration the motors follow, with the resonance band
        if let Some(r) = &resonance {
            let panel = fa.axes2d();
            panel.set_x_label("Frequency (Hz)", &[]);
            panel.set_y_label(
                format!("Accel. spectrum ({}/s^2)", hwa::Contract::SPACE_UNIT_MAGNITUDE).as_str(), &[]
            );
            let peak = r.amplitude.iter().take(2).flat_map(|a| a.iter()).fold(0.0f64, |m, v| m.max(*v));
            let top = (peak * 1.1).max(1e-6);
            for f in [r.band.0, r.band.1] {
                panel.arrow(
                    gnuplot::Coordinate::Axis(f), gnuplot::Coordinate::Axis(0.0),
                    gnuplot::Coordinate::Axis(f), gnuplot::Coordinate::Axis(top),
                    &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray".into())]);
            }
            for k in 0..2 {
                panel.lines(r.freqs.clone(), r.amplitude[k].clone(),
                    &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
            }
            panel.set_x_range(AutoOption::Fix(0.0), AutoOption::Fix(r.freqs.last().copied().unwrap_or(1.0)));
            panel.set_y_range(AutoOption::Fix(0.0), AutoOption::Fix(top));
        }
        #[cfg(not(test))]
        if std::env::var("S_PLOT_NO_SHOW").is_err() {
            match fa.show_and_keep_running() {
                Err(_err) => {hwa::error!("Unable to show plot: {:?}", _err)}
                _ => {}
            }
        }
        _ = fa.save_to_pdf("plot_axes.pdf", 10.0f32, 10.0f32);
    }

    #[cfg(not(test))]
    std::process::exit(0);
}

/// Frame excitation of the motion: the acceleration the motors follow (constant inside each
/// window, the velocity ramps linearly in it), and its RMS in the resonance band of the machine
/// frame (25-100 Hz by default, an Ender-3 like frame; `S_PLOT_RES_BAND=lo,hi` changes it).
/// Lower is smoother: fewer vibrations excited. It accounts for the planned profile (jerk,
/// acceleration), the velocity jumps at the corners and the staircase of the window rate
/// (`MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY`).
struct Resonance {
    band: (f64, f64),
    /// RMS in the band of X, Y and Z
    rms: [f64; 3],
    /// RMS in the band of the XY plane
    rms_xy: f64,
    /// Spectrum (amplitude of each frequency) of X, Y and Z, up to [Self::MAX_FREQ]
    freqs: Vec<f64>,
    amplitude: [Vec<f64>; 3],
}

impl Resonance {
    /// The acceleration is resampled at this period
    const DT: f64 = 0.001;
    /// Highest frequency of the plotted spectrum
    const MAX_FREQ: f64 = 150.0;

    fn band_from_env() -> (f64, f64) {
        std::env::var("S_PLOT_RES_BAND")
            .ok()
            .and_then(|v| {
                let (lo, hi) = v.split_once(',')?;
                Some((lo.trim().parse::<f64>().ok()?, hi.trim().parse::<f64>().ok()?))
            })
            .filter(|(lo, hi)| *lo >= 0.0 && hi > lo)
            .unwrap_or((25.0, 100.0))
    }

    fn analyze(window_ends: &[(f64, [f64; 3])], band: (f64, f64)) -> Option<Self> {
        if window_ends.len() < 3 {
            return None;
        }
        let (t0, t1) = (window_ends[0].0, window_ends[window_ends.len() - 1].0);
        let n = ((t1 - t0) / Self::DT) as usize;
        if n < 16 {
            return None;
        }
        // Constant acceleration of the window containing each instant
        let mut accel: [Vec<f64>; 3] = [vec![0.0; n], vec![0.0; n], vec![0.0; n]];
        let mut w = 1usize;
        for i in 0..n {
            let t = t0 + i as f64 * Self::DT;
            while w + 1 < window_ends.len() && window_ends[w].0 <= t {
                w += 1;
            }
            let (ta, va) = window_ends[w - 1];
            let (tb, vb) = window_ends[w];
            let span = (tb - ta).max(1e-9);
            for k in 0..3 {
                accel[k][i] = (vb[k] - va[k]) / span;
            }
        }
        // Hann window (its amplitude gain is 1/2)
        let hann: Vec<f64> = (0..n)
            .map(|i| 0.5 - 0.5 * (2.0 * core::f64::consts::PI * i as f64 / (n - 1) as f64).cos())
            .collect();
        let df = 1.0 / (n as f64 * Self::DT);
        let last_bin = ((Self::MAX_FREQ.max(band.1) / df) as usize).min(n / 2);
        let mut freqs = Vec::with_capacity(last_bin);
        let mut amplitude: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        let mut band_power = [0.0f64; 3];
        let signals: Vec<Vec<f64>> = accel
            .iter()
            .map(|a| {
                let mean = a.iter().sum::<f64>() / n as f64;
                a.iter().zip(hann.iter()).map(|(x, h)| (x - mean) * h).collect()
            })
            .collect();
        for bin in 1..=last_bin {
            let f = bin as f64 * df;
            // DFT of this bin (a rotating phasor: no trigonometry per sample)
            let (c, s) = ((2.0 * core::f64::consts::PI * bin as f64 / n as f64).cos(),
                          (2.0 * core::f64::consts::PI * bin as f64 / n as f64).sin());
            for k in 0..3 {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                let (mut pr, mut pi) = (1.0f64, 0.0f64);
                for x in signals[k].iter() {
                    re += x * pr;
                    im -= x * pi;
                    let npr = pr * c - pi * s;
                    pi = pr * s + pi * c;
                    pr = npr;
                }
                let power = re * re + im * im;
                if f >= band.0 && f < band.1 {
                    band_power[k] += power;
                }
                if f <= Self::MAX_FREQ {
                    // Single-sided amplitude, corrected by the Hann gain
                    amplitude[k].push(2.0 * power.sqrt() / n as f64 * 2.0);
                }
            }
            if f <= Self::MAX_FREQ {
                freqs.push(f);
            }
        }
        // Band-limited RMS (Parseval, single-sided), corrected by the Hann gain
        let rms = band_power.map(|p| (2.0 * p).sqrt() / n as f64 * 2.0);
        Some(Self { band, rms, rms_xy: rms[0].hypot(rms[1]), freqs, amplitude })
    }

    fn report(&self) {
        hwa::info!(
            "[s_plot] Resonance band {:.0}-{:.0} Hz: XY {:.1} {}/s² RMS (X {:.1}, Y {:.1}, Z {:.1}). Lower is smoother",
            self.band.0,
            self.band.1,
            self.rms_xy,
            hwa::Contract::SPACE_UNIT_MAGNITUDE,
            self.rms[0],
            self.rms[1],
            self.rms[2]
        );
    }
}

/// Deviation of the real (stepped) position from the ideal one at the end of every segment, in
/// steps of each axis (X, Y, Z). Rounding to the nearest step bounds it to half a step: anything
/// above is a deviation that the motion pipeline lets accumulate
#[derive(Default)]
struct StepDeviation {
    samples: [u32; 3],
    max_abs: [f64; 3],
    sum_abs: [f64; 3],
    last: [f64; 3],
    above_half_step: [u32; 3],
    /// Distance of the real position to the step grid (float drift)
    max_off_grid: [f64; 3],
}

impl StepDeviation {
    fn record(&mut self, axis: usize, deviation: f64, off_grid: f64) {
        self.samples[axis] += 1;
        self.max_abs[axis] = self.max_abs[axis].max(deviation.abs());
        self.sum_abs[axis] += deviation.abs();
        self.last[axis] = deviation;
        if deviation.abs() > 0.5 + 1e-6 {
            self.above_half_step[axis] += 1;
        }
        self.max_off_grid[axis] = self.max_off_grid[axis].max(off_grid.abs());
    }

    fn report(&self) {
        for (k, name) in ["X", "Y", "Z"].iter().enumerate() {
            if self.samples[k] == 0 {
                continue;
            }
            hwa::info!(
                "[s_plot] Step deviation {}: {} segment ends, max {:.4} steps, mean {:.4}, final {:.4}, above half a step: {}, max off the step grid: {:.2e}",
                name,
                self.samples[k],
                self.max_abs[k],
                self.sum_abs[k] / self.samples[k] as f64,
                self.last[k],
                self.above_half_step[k],
                self.max_off_grid[k],
            );
        }
    }
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
