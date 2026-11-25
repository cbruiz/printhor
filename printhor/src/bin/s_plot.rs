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

    configure(&motion_planner);
    #[cfg(all(feature = "with-motion", feature = "native", feature = "with-ps-on"))]
    {
        hwa::info!("Virtually powering on");
        event_bus.publish_event(hwa::EventStatus::containing(hwa::EventFlags::ATX_ON)).await;
    }
    let gcode = load_gcode();

    #[cfg(not(feature = "with-input-shaping"))]
    {
        let run = run_job(&c, &gcode).await;
        let _ = report(&run, "", "off (feature disabled)", None, None);
    }
    // Two runs of the job: without input shaping, and with a shaper tuned from the ringing of the
    // first one (or the one forced by S_PLOT_SHAPER). The files of each one are suffixed with
    // -without-shaping and -with-input-shaping
    #[cfg(feature = "with-input-shaping")]
    {
        let cfg = motion_planner.motion_config();
        hwa::info!("[s_plot] Run 1/2: without input shaping");
        cfg.set_input_shaping([motion::ShaperConfig::disabled(); 2]);
        let without = run_job(&c, &gcode).await;
        let (ringing_without, zoom) = report(&without, "-without-shaping", "off", None, None);
        let shaper = match forced_shaper() {
            Some(shaper) => {
                hwa::info!("[s_plot] Shaper forced by S_PLOT_SHAPER");
                shaper
            }
            None => match ringing_without.as_ref().and_then(tune) {
                Some(shaper) => shaper,
                None => {
                    hwa::warn!("[s_plot] No ringing to tune the shaper from: Marlin's defaults");
                    MARLIN_SHAPER
                }
            },
        };
        let title = shaper_title(&shaper);
        hwa::info!("[s_plot] Run 2/2: with input shaping {}", title.as_str());
        configure(&motion_planner);
        cfg.set_input_shaping([shaper; 2]);
        let with = run_job(&c, &gcode).await;
        let response = match shaper.frequency.is_zero() {
            true => None,
            false => Some(shaper_response(&shaper, Ringing::model_from_env().1)),
        };
        let (ringing_with, _) = report(&with, "-with-input-shaping", &title, response, zoom);
        if let (Some(a), Some(b)) = (&ringing_without, &ringing_with) {
            let change = |a: f64, b: f64| if a > 0.0 { (b - a) / a * 100.0 } else { 0.0 };
            hwa::info!(
                "[s_plot] Input shaping {}: residual ringing {:.6} -> {:.6} {} RMS ({:+.1}%), peak {:.6} -> {:.6} ({:+.1}%); ringing {:.5} -> {:.5} RMS ({:+.1}%)",
                title.as_str(),
                a.residual_rms_xy,
                b.residual_rms_xy,
                hwa::Contract::SPACE_UNIT_MAGNITUDE,
                change(a.residual_rms_xy, b.residual_rms_xy),
                a.residual_peak[0].max(a.residual_peak[1]),
                b.residual_peak[0].max(b.residual_peak[1]),
                change(a.residual_peak[0].max(a.residual_peak[1]), b.residual_peak[0].max(b.residual_peak[1])),
                a.rms_xy,
                b.rms_xy,
                change(a.rms_xy, b.rms_xy)
            );
        }
    }

    #[cfg(not(test))]
    std::process::exit(0);
}

/// The machine: the motion defaults of the board, at its homing point. Also before every run
fn configure(motion_planner: &hwa::controllers::MotionPlanner) {
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
            cfg.set_probe_offset(hwa::make_vector_real!(x=33.0, y=-4.0, z=-3.9));

            cfg.set_flow_rate(100);
            cfg.set_speed_rate(100);

            // Compute min speed. Really useful because of discretion effects
            cfg.compute_min_speed();

            // No home offsets (a job may leave them set: M206)
            cfg.update_home_offset(&hwa::math::TVector::new_with_coord(
                hwa::CoordSel::motion_relevant_axis(),
                Some(hwa::math::ZERO),
            ));

            // Make homing unneeded: at the homing point of the board (X -17.3, Y -21, Z 0)
            hwa::info!("Virtually homing");
            {
                let position = hwa::controllers::Position::new_with_world_projection(
                    &hwa::make_vector_real!(x=-17.3, y=-21.0, z=0.0),
                );
                motion_planner
                    .motion_status()
                    .update_last_planned_position(0, &position);
                motion_planner
                    .motion_status()
                    .update_current_position(0, &position);
            }
        }
    }

}

/// The G-code to plot: the whole pen-test job by default, or the file in `S_PLOT_GCODE`
fn load_gcode() -> String {
    let path = std::env::var("S_PLOT_GCODE")
        .unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/../pen-test.gcode").to_string());
    match std::fs::read_to_string(&path) {
        Ok(text) => {
            hwa::info!("[s_plot] G-code: {}", path.as_str());
            text
        }
        Err(_e) => {
            hwa::error!("[s_plot] Unable to read {}: {:?}", path.as_str(), _e);
            String::new()
        }
    }
}

/// The finer sampling of the axes data (`S_PLOT_AXES_DT_US`), seconds
fn axes_dt() -> Option<f64> {
    std::env::var("S_PLOT_AXES_DT_US")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0)
        .map(|v| v * 1e-6)
}

/// What a run of the job produced (see [run_job])
struct JobRun {
    data_points: instrumentation::data_points::DataPoints,
    /// Velocity of each axis at every window end: the input of the resonance analysis
    window_ends: Vec<(f64, [f64; 3])>,
    /// The X and Y motors at every window end, in a continuous frame: the input of the ringing
    /// analysis
    motor_windows: Vec<(f64, [f64; 2], [f64; 2])>,
    deviation: StepDeviation,
}

/// Plans and executes the job (as the firmware does, with the shaper set in the configuration),
/// recording the motion
async fn run_job(c: &instrumentation::machinery::SplotContext, gcode: &str) -> JobRun {
    let event_bus = c.event_bus.clone();
    let motion_planner = c.motion_planner.clone();
    let gcode_buff = {
        let mut b = instrumentation::gcode::GCodeBuffer::new();
        b.append(gcode);
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
    let axes_dt_s = axes_dt();
    let mut deviation = StepDeviation::default();
    // The motion of the X and Y motors at every window end, in a continuous frame (the planned
    // displacements, so that a G92 does not move it): instant, position and velocity. The input
    // of the ringing analysis (see [Ringing]): shaped with input shaping
    let mut motor_windows: Vec<(f64, [f64; 2], [f64; 2])> = Vec::new();
    let mut frame_xy = [0.0f64; 2];
    #[cfg(feature = "with-input-shaping")]
    let mut shaped = ShapedXY::new(&motion_planner.motion_config().get_input_shaping());
    // The last commanded position (X, Y, Z) and the offset of the frame to it
    #[allow(unused_mut, unused_variables)]
    let (mut last_p, mut frame_offset) = ([0.0f64; 3], [0.0f64; 2]);
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
                        // The continuous frame of X and Y: where the segment starts, and its
                        // displacement per unit of path (as executed: the frame does not move with
                        // the coordinates, e.g. the home offsets or a G92)
                        let frame_start = frame_xy;
                        let frame_ratio = [hwa::CoordSel::X, hwa::CoordSel::Y]
                            .map(|c| segment.unit_vector_dir.get_coord(c).map_or(0.0, |u| u.to_f64()));
                        #[cfg(feature = "with-input-shaping")]
                        {
                            let v0 = v_window_start.to_f64();
                            shaped.segment_starts(frame_start, frame_ratio.map(|r| r * v0));
                            // The frame to the plotted (absolute) positions
                            for (k, c) in [hwa::CoordSel::X, hwa::CoordSel::Y].into_iter().enumerate() {
                                let o = segment
                                    .src_pos
                                    .get_coord(c)
                                    .or(current_real_pos.space_pos.get_coord(c))
                                    .map_or(0.0, |x| x.to_f64());
                                frame_offset[k] = o - frame_start[k];
                            }
                        }
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
                                // The motors at the window end, in the frame
                                let (s_end, v_end) = (
                                    segment_iterator.current_position().to_f64(),
                                    segment_iterator.current_speed().to_f64(),
                                );
                                #[allow(unused_mut)]
                                let mut motor = (
                                    [0, 1].map(|k| frame_start[k] + frame_ratio[k] * s_end),
                                    frame_ratio.map(|r| r * v_end),
                                );
                                #[cfg(feature = "with-input-shaping")]
                                shaped.window_ends(segment_iterator.dt_us() as u32, motor.0, motor.1);
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
                                    // Input shaping: X and Y as the motors follow them
                                    #[cfg(feature = "with-input-shaping")]
                                    if shaped.is_active() {
                                        let before_us = ((dt * (1.0 - frac)) * 1e6).round() as u32;
                                        for j in 0..2 {
                                            let (ps, vs, as_) = shaped.shaped(j, before_us);
                                            (p[j], vv[j], aa[j]) = (ps + frame_offset[j], vs, as_);
                                            if k == n_sub {
                                                (motor.0[j], motor.1[j]) = (ps, vs);
                                            }
                                        }
                                    }
                                    data_points.axes.push(data_points.reference_time - dt * (1.0 - frac), p, vv, aa);
                                    if k == n_sub {
                                        window_ends.push((data_points.reference_time, vv));
                                        motor_windows.push((data_points.reference_time, motor.0, motor.1));
                                        last_p = p;
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
                        frame_xy = [0, 1].map(|k| frame_start[k] + frame_ratio[k] * segment.displacement_su.to_f64());
                        #[cfg(feature = "with-input-shaping")]
                        {
                            last_p[0] = frame_xy[0] + frame_offset[0];
                            last_p[1] = frame_xy[1] + frame_offset[1];
                        }
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
            controllers::motion_control::ExecPlan::Homing(_channel, _order_num, plan) => {
                // The head stops: the shaped motion is completed first (as the firmware)
                #[cfg(feature = "with-input-shaping")]
                shaped.complete(&mut data_points, &mut window_ends, &mut motor_windows, last_p, frame_offset, axes_dt_s);
                // Virtual homing: where the firmware ends it (see the stepper task)
                let position = motion_planner.homing_end_position(
                    &motion_planner.motion_status().get_current_position(),
                    &plan,
                );
                motion_planner
                    .motion_status()
                    .update_current_position(_order_num, &position);
            }
            #[cfg(feature = "with-input-shaping")]
            controllers::motion_control::ExecPlan::Dwell(..) => {
                shaped.complete(&mut data_points, &mut window_ends, &mut motor_windows, last_p, frame_offset, axes_dt_s);
            }
            _ => {}
        }
        let _consumed = motion_planner.consume_current_plan(&event_bus).await;
        // The motion stops (nothing queued behind): the shaped motion is completed (as the firmware)
        #[cfg(feature = "with-input-shaping")]
        if _consumed.moves_left == 0 {
            shaped.complete(&mut data_points, &mut window_ends, &mut motor_windows, last_p, frame_offset, axes_dt_s);
        }
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
    JobRun {
        data_points,
        window_ends,
        motor_windows,
        deviation,
    }
}

/// Reports and plots a run (its files named with `suffix`): the step deviation, the resonance band
/// excitation, the ringing of the frame and the figures. `shaping_title`: the input shaping of the
/// run; `shaper_response`: the vibration its shaper leaves at each frequency, when it has one;
/// `zoom`: the interval (s) of the ringing zoom, around the strongest ringing when not given.
/// Returns the ringing and the zoom interval
fn report(
    run: &JobRun,
    suffix: &str,
    shaping_title: &str,
    shaper_response: Option<(Vec<f64>, Vec<f64>)>,
    zoom: Option<(f64, f64)>,
) -> (Option<Ringing>, Option<(f64, f64)>) {
    let data_points = &run.data_points;
    let axes_dt_s = axes_dt();
    run.deviation.report();
    let resonance = Resonance::analyze(&run.window_ends, Resonance::band_from_env());
    match &resonance {
        Some(r) => r.report(),
        None => hwa::warn!("[s_plot] Resonance: not enough motion to analyze"),
    }
    let ringing = Ringing::analyze(&run.motor_windows, Ringing::model_from_env());
    match &ringing {
        Some(r) => {
            r.report();
            let _ = r.write_csv(&format!("s_plot_vibration{}.csv", suffix));
        }
        None => hwa::warn!("[s_plot] Ringing: not enough motion to analyze"),
    }
    // The ringing zoom: 150 ms from 20 ms before the strongest ringing
    let zoom = zoom.or_else(|| {
        let r = ringing.as_ref()?;
        let strongest = (0..r.times.len()).max_by(|a, b| {
            let m = |i: usize| r.ringing[0][i].abs().max(r.ringing[1][i].abs());
            m(*a).total_cmp(&m(*b))
        })?;
        let from = (r.times[strongest] - 0.02).max(0.0);
        Some((from, from + 0.15))
    });
    hwa::info!("[s_plot] Input shaping: {}", shaping_title);

    {
        use gnuplot::{AutoOption::Fix, AxesCommon, Coordinate::Axis, Figure, PlotOption};

        hwa::info!("Writing data points to s_plot{}.csv...", suffix);
        let _ = data_points.write_csv(&format!("s_plot{}.csv", suffix));

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
        let unit = hwa::Contract::SPACE_UNIT_MAGNITUDE;
        let t_end = data_points.reference_time;
        // Segment and interval marks. gnuplot draws them (arrows) in front of the curves: where
        // the segments are short (a circle of 0.55 mm segments) they made a solid block that hid
        // the curves. Light gray, and only those at least 0.25% of the duration apart
        let sparse = |marks: &[f64]| -> Vec<f64> {
            let min_gap = t_end * 0.0025;
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
        // The range of some series, with a 10% margin
        let range = |series: &[&Vec<f64>]| {
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            for v in series.iter().flat_map(|s| s.iter()) {
                lo = lo.min(*v);
                hi = hi.max(*v);
            }
            if lo > hi { (0.0, 1.0) } else { let m = (hi - lo).max(1e-9) * 0.1; (lo - m, hi + m) }
        };

        // First figure: the profile along the path
        let path_figure = |layout: &PlotLayout| {
            let mut fg = Figure::new();
            fg.set_multiplot_layout(layout.rows(), layout.columns()).set_title(&format!(
                "{} Hybrid S-Curve/Quintic velocity profile along the path\n{} segments, {:.2} {} in {:.2} s. {} Hz step interpolation, {} Hz sampling. Segment ends in gray, profile phases in light gray",
                MATH_PRECISION,
                data_points.num_segments(),
                data_points.total_displacement().to_f64(),
                unit,
                t_end,
                // Interpolation: the step clock; sampling: the micro-segments of the profile
                hwa::Contract::STEP_PLANNER_CLOCK_FREQUENCY,
                hwa::Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY,
            ));
            // The extremes of each series, with a 10% margin (the positions to whole units)
            let with_margin = |lo: Option<f64>, hi: Option<f64>| {
                let (lo, hi) = (lo.unwrap_or(0.0), hi.unwrap_or(0.0));
                let m = (hi - lo) * 0.1;
                (lo - m, hi + m)
            };
            let panels = [
                (
                    "Position along the path",
                    format!("Position ({})", unit),
                    &data_points.sampled_positions,
                    "blue",
                    with_margin(data_points.min_position().map(f64::floor), data_points.max_position().map(f64::ceil)),
                ),
                (
                    "Velocity along the path",
                    format!("Velocity ({}/s)", unit),
                    &data_points.sampled_velocities,
                    "dark-green",
                    with_margin(data_points.min_velocity(), data_points.max_velocity()),
                ),
                (
                    "Acceleration along the path",
                    format!("Acceleration ({}/s²)", unit),
                    &data_points.sampled_accelerations,
                    "red",
                    with_margin(data_points.min_acceleration(), data_points.max_acceleration()),
                ),
            ];
            for (row, (title, y_label, series, color, (lo, hi))) in panels.iter().enumerate() {
                let (lo, hi) = (*lo, *hi);
                let panel = fg.axes2d();
                layout.place(panel, row, title, y_label, "Time (s)");
                for (marks, gray) in [(&segment_marks, "gray60"), (&interval_marks, "gray85")] {
                    for instant in marks.iter() {
                        panel.arrow(
                            Axis(*instant), Axis(lo), Axis(*instant), Axis(hi),
                            &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color((*gray).into()), PlotOption::ArrowSize(0.5)],
                        );
                    }
                }
                panel.lines(series.times.clone(), series.points.clone(), &[PlotOption::Color((*color).into())]);
                panel.set_x_range(Fix(0.0), Fix(t_end));
                layout.y_range(panel, lo, hi);
            }
            fg
        };
        PlotLayout::output(path_figure, 3, false, &format!("plot{}.pdf", suffix));

        // Second figure: the motion in machine axes, its vector magnitudes and its analyses
        hwa::info!("Writing axes data points to s_plot_axes{}.csv...", suffix);
        let _ = data_points.axes.write_csv(&format!("s_plot_axes{}.csv", suffix));
        let axes = &data_points.axes;
        let names = ["X", "Y", "Z"];
        let colors = ["blue", "red", "dark-green"];
        // The motion plan: the velocity profile, the shape of its jerk and whether the moves are
        // chained (junction speeds) or each one starts and ends at rest
        let motion_plan = format!(
            "Hybrid S-Curve/Quintic velocity profile, {:?} jerk, {}",
            motion::DEFAULT_JERK_SHAPE,
            match cfg!(feature = "with-motion-chaining") {
                true => "chained moves",
                false => "moves from rest to rest",
            },
        );
        // The rates of the firmware (see MotionConfig::compute_min_speed): the profile is sampled
        // into micro-segments at MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY and the steps interpolated
        // at STEP_PLANNER_CLOCK_FREQUENCY. The axes data may be finer (S_PLOT_AXES_DT_US)
        let axes_data_step = match axes_dt_s {
            Some(dt) => format!(", axes data every {:.0} us", dt * 1e6),
            None => String::new(),
        };
        let rows = 3 + resonance.is_some() as usize + 2 * ringing.is_some() as usize + shaper_response.is_some() as usize;
        let axes_figure = |layout: &PlotLayout| {
            let mut fa = Figure::new();
            fa.set_multiplot_layout(layout.rows(), layout.columns()).set_title(&format!(
                "{} Motion in machine axes: {}\n{} segments, {:.2} {} in {:.2} s. {} Hz step interpolation, {} Hz sampling{}. Input shaping: {}",
                MATH_PRECISION,
                motion_plan,
                data_points.num_segments(),
                data_points.total_displacement().to_f64(),
                unit,
                t_end,
                hwa::Contract::STEP_PLANNER_CLOCK_FREQUENCY,
                hwa::Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY,
                axes_data_step,
                shaping_title,
            ));
            let mut row = 0usize;
            // Position, velocity and acceleration of the axes, with the magnitude of the vectors
            let speed_caption = "|v|";
            let accel_caption = "|dv/dt|";
            let axis_panels = [
                ("Position of the axes", format!("Position ({})", unit), &axes.positions, None),
                ("Velocity of the axes, and the speed |v|", format!("Velocity ({}/s)", unit), &axes.velocities, Some((&axes.speed, speed_caption))),
                (
                    "Acceleration of the axes, and |dv/dt|: the acceleration of the velocity vector (its direction changes included)",
                    format!("Acceleration ({}/s²)", unit),
                    &axes.accelerations,
                    Some((&axes.total_acceleration, accel_caption)),
                ),
            ];
            for (title, y_label, series, magnitude) in axis_panels.iter() {
                let panel = fa.axes2d();
                layout.place(panel, row, title, y_label, "Time (s)");
                row += 1;
                let mut all: Vec<&Vec<f64>> = series.iter().collect();
                for k in 0..3 {
                    panel.lines(axes.times.clone(), series[k].clone(),
                        &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
                }
                if let Some((values, caption)) = magnitude {
                    panel.lines(axes.times.clone(), (*values).clone(),
                        &[PlotOption::Color("black".into()), PlotOption::Caption(caption)]);
                    all.push(values);
                }
                let (lo, hi) = range(&all);
                panel.set_x_range(Fix(0.0), Fix(t_end));
                layout.y_range(panel, lo, hi);
            }
            // Spectrum of the acceleration the motors follow, with the resonance band
            if let Some(r) = &resonance {
                let panel = fa.axes2d();
                layout.place(
                    panel,
                    row,
                    &format!(
                        "Spectrum of the acceleration the motors follow. {:.0}-{:.0} Hz band (gray): {:.1} {}/s² RMS (X {:.1}, Y {:.1})",
                        r.band.0, r.band.1, r.rms_xy, unit, r.rms[0], r.rms[1]
                    ),
                    &format!("Amplitude ({}/s²)", unit),
                    "Frequency (Hz)",
                );
                row += 1;
                let peak = r.amplitude.iter().take(2).flat_map(|a| a.iter()).fold(0.0f64, |m, v| m.max(*v));
                let top = (peak * 1.1).max(1e-6);
                for f in [r.band.0, r.band.1] {
                    panel.arrow(Axis(f), Axis(0.0), Axis(f), Axis(top),
                        &[PlotOption::ArrowType(gnuplot::NoArrow), PlotOption::Color("gray".into())]);
                }
                for k in 0..2 {
                    panel.lines(r.freqs.clone(), r.amplitude[k].clone(),
                        &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
                }
                panel.set_x_range(Fix(0.0), Fix(r.freqs.last().copied().unwrap_or(1.0)));
                layout.y_range(panel, 0.0, top);
            }
            // The ringing of the head: its oscillation about the deflection of the frame under the
            // acceleration (what input shaping cancels), over the whole job and around the strongest
            // ringing (the same interval in the runs compared)
            if let Some(r) = &ringing {
                let panel = fa.axes2d();
                layout.place(
                    panel,
                    row,
                    &format!(
                        "Ringing of the head on a {:.0} Hz, ζ {:.2} frame: {:.4} {} RMS. Residual (steady motion): {:.5} {} RMS, peak {:.5} {}",
                        r.model.0, r.model.1, r.rms_xy, unit, r.residual_rms_xy, unit,
                        r.residual_peak[0].max(r.residual_peak[1]), unit
                    ),
                    &format!("Ringing ({})", unit),
                    "Time (s)",
                );
                row += 1;
                for k in 0..2 {
                    panel.lines(r.times.clone(), r.ringing[k].clone(),
                        &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
                }
                let (lo, hi) = range(&[&r.ringing[0], &r.ringing[1]]);
                panel.set_x_range(Fix(0.0), Fix(r.times.last().copied().unwrap_or(1.0)));
                layout.y_range(panel, lo, hi);

                let panel = fa.axes2d();
                layout.place(panel, row, "Ringing around the strongest corner (the same interval in the runs compared)",
                    &format!("Ringing ({})", unit), "Time (s)");
                row += 1;
                if let Some((from, to)) = zoom {
                    let inside: Vec<usize> = (0..r.times.len()).filter(|i| r.times[*i] >= from && r.times[*i] <= to).collect();
                    let times: Vec<f64> = inside.iter().map(|i| r.times[*i]).collect();
                    let series = [0, 1].map(|k| inside.iter().map(|i| r.ringing[k][*i]).collect::<Vec<f64>>());
                    for k in 0..2 {
                        panel.lines(times.clone(), series[k].clone(),
                            &[PlotOption::Color(colors[k].into()), PlotOption::Caption(names[k])]);
                    }
                    let (lo, hi) = range(&[&series[0], &series[1]]);
                    panel.set_x_range(Fix(from), Fix(to));
                    layout.y_range(panel, lo, hi);
                }
            }
            // The vibration the shaper leaves at each frequency, relative to none
            if let Some((freqs, response)) = &shaper_response {
                let panel = fa.axes2d();
                layout.place(
                    panel,
                    row,
                    &format!("Shaper {}: vibration left in a frame of each frequency (ζ {:.2}), relative to none",
                        shaping_title, Ringing::model_from_env().1),
                    "Vibration left",
                    "Frequency (Hz)",
                );
                panel.lines(freqs.clone(), response.clone(), &[PlotOption::Color("black".into())]);
                panel.set_x_range(Fix(0.0), Fix(freqs.last().copied().unwrap_or(1.0)));
                layout.y_range(panel, 0.0, 1.05);
            }
            fa
        };
        PlotLayout::output(axes_figure, rows, true, &format!("plot_axes{}.pdf", suffix));
    }

    (ringing, zoom)
}

/// The page of a figure of `panels` panels, all aligned (the same edges in each column): a title
/// above each one, room below for the tick and axis labels, on the left for the y label and on the
/// right for the legend, outside, where it never covers the curves.
///
/// The room for the text is fixed in points (it does not shrink with the page), so the panels
/// get whatever is left. Two pages:
/// * the PDF: one column, its height growing with the panels (2.3 in each), 9 pt text;
/// * the window (gnuplot's interactive terminal): `S_PLOT_WINDOW=<width>x<height>` pixels
///   (1600x900 by default), 9 pt text and, with more than three panels, two columns, so that a
///   figure fits a screen without squeezing its panels.
///
/// The panel titles are wrapped to the width of their panel (two lines at most in the narrow
/// columns of a window, the room above every panel of the row: they stay aligned)
struct PlotLayout {
    panels: usize,
    columns: usize,
    /// Whether the panels have a legend (room on the right)
    legend: bool,
    /// Size of the page, in its units (points for the PDF, pixels for the window)
    width: f32,
    height: f32,
    /// Units of the page per point of text
    scale: f32,
}

impl PlotLayout {
    /// The title of the figure (two lines)
    const TITLE_PT: f32 = 40.0;
    /// Above each panel: its title (one line), and each other line of it
    const ABOVE_PT: f32 = 22.0;
    const LINE_PT: f32 = 11.0;
    /// Mean width of a character of the 9 pt text (sans serif), to wrap the titles
    const CHAR_PT: f32 = 4.8;
    /// Below each panel: the tick labels and the axis label
    const BELOW_PT: f32 = 35.0;
    /// Left of each panel: the tick labels and the (rotated) y label
    const LEFT_PT: f32 = 60.0;
    /// Right of each panel: the legend, or a margin without legend
    const LEGEND_PT: f32 = 90.0;
    const RIGHT_PT: f32 = 20.0;
    /// Width (pt) and height of a panel (pt) of the PDF
    const PDF_WIDTH_PT: f32 = 792.0;
    const PDF_PANEL_PT: f32 = 166.0;
    /// Pixels of the window per point of text (gnuplot's interactive terminals render ~96 dpi)
    #[cfg(not(test))]
    const WINDOW_PX_PER_PT: f32 = 1.33;

    fn pdf(panels: usize, legend: bool) -> Self {
        let panels = panels.max(1);
        Self {
            panels,
            columns: 1,
            legend,
            width: Self::PDF_WIDTH_PT,
            height: Self::TITLE_PT + panels as f32 * Self::PDF_PANEL_PT,
            scale: 1.0,
        }
    }

    #[cfg(not(test))]
    fn window(panels: usize, legend: bool) -> Self {
        let (width, height) = std::env::var("S_PLOT_WINDOW")
            .ok()
            .and_then(|v| {
                let (w, h) = v.split_once('x')?;
                Some((w.trim().parse::<f32>().ok()?, h.trim().parse::<f32>().ok()?))
            })
            .filter(|(w, h)| *w >= 400.0 && *h >= 300.0)
            .unwrap_or((1600.0, 900.0));
        let panels = panels.max(1);
        Self {
            panels,
            columns: if panels > 3 { 2 } else { 1 },
            legend,
            width,
            height,
            scale: Self::WINDOW_PX_PER_PT,
        }
    }

    fn columns(&self) -> usize {
        self.columns
    }

    fn rows(&self) -> usize {
        self.panels.div_ceil(self.columns)
    }

    /// Lines of the panel titles: two in the narrow columns of a window
    fn title_lines(&self) -> usize {
        if self.columns > 1 { 2 } else { 1 }
    }

    /// `text` wrapped at its spaces into lines of `max_chars` at most, and `max_lines` (the last one
    /// cut with an ellipsis if it does not fit)
    fn wrap(text: &str, max_chars: usize, max_lines: usize) -> String {
        let max_chars = max_chars.max(8);
        let mut lines: Vec<String> = Vec::new();
        let mut line = String::new();
        for word in text.split(' ') {
            if !line.is_empty() && line.chars().count() + 1 + word.chars().count() > max_chars {
                lines.push(core::mem::take(&mut line));
            }
            if !line.is_empty() {
                line.push(' ');
            }
            line.push_str(word);
        }
        lines.push(line);
        if lines.len() > max_lines {
            let rest = lines.split_off(max_lines.max(1) - 1).join(" ");
            let mut last: String = rest.chars().take(max_chars - 1).collect();
            last.push('…');
            lines.push(last);
        }
        lines.join("\n")
    }

    /// Places the panel `index` (row by row), with its title, axis labels, grid and legend
    fn place(&self, panel: &mut gnuplot::Axes2D, index: usize, title: &str, y_label: &str, x_label: &str) {
        use gnuplot::{AlignType::*, AxesCommon, Coordinate::Graph, DashType, LegendOption, MarginSide::*, PlotOption};
        let (row, column) = (index / self.columns, index % self.columns);
        let title_h = Self::TITLE_PT * self.scale;
        let panel_h = (self.height - title_h) / self.rows() as f32;
        let column_w = self.width / self.columns as f32;
        let right = match self.legend {
            true => Self::LEGEND_PT,
            false => Self::RIGHT_PT,
        };
        // The title, within the width of the panel
        let width_pt = column_w / self.scale - Self::LEFT_PT - right;
        let title = Self::wrap(title, (width_pt / Self::CHAR_PT) as usize, self.title_lines());
        let above = Self::ABOVE_PT + (self.title_lines() - 1) as f32 * Self::LINE_PT;
        panel
            .set_margins(&[
                MarginLeft((column as f32 * column_w + Self::LEFT_PT * self.scale) / self.width),
                MarginRight(((column + 1) as f32 * column_w - right * self.scale) / self.width),
                MarginTop(1.0 - (title_h + row as f32 * panel_h + above * self.scale) / self.height),
                MarginBottom(1.0 - (title_h + (row + 1) as f32 * panel_h - Self::BELOW_PT * self.scale) / self.height),
            ])
            .set_title(&title, &[])
            .set_y_label(y_label, &[])
            .set_x_label(x_label, &[])
            .set_legend(Graph(1.01), Graph(1.0), &[LegendOption::Placement(AlignLeft, AlignTop)], &[])
            .set_grid_options(false, &[PlotOption::Color("gray85".into()), PlotOption::LineStyle(DashType::Dot)])
            .set_x_grid(true)
            .set_y_grid(true);
    }

    /// Height (pt) of the plot area of the panels
    fn plot_height_pt(&self) -> f32 {
        let panel_h = (self.height - Self::TITLE_PT * self.scale) / self.rows() as f32;
        let above = Self::ABOVE_PT + (self.title_lines() - 1) as f32 * Self::LINE_PT;
        panel_h / self.scale - above - Self::BELOW_PT
    }

    /// Sets the y range of a panel, with as many tick labels as its height holds (one every 18 pt
    /// at most, at a 1, 2 or 5 step): gnuplot chooses them from the range only, and in a short
    /// panel they overlap
    fn y_range(&self, panel: &mut gnuplot::Axes2D, lo: f64, hi: f64) {
        use gnuplot::{AutoOption::Fix, AxesCommon};
        let max_ticks = ((self.plot_height_pt() / 18.0) as usize).max(2);
        let span = (hi - lo).abs();
        panel.set_y_range(Fix(lo), Fix(hi));
        if !(span > 0.0) {
            return;
        }
        let magnitude = 10f64.powf((span / max_ticks as f64).log10().floor());
        let step = [1.0, 2.0, 5.0, 10.0, 20.0]
            .iter()
            .map(|m| m * magnitude)
            .find(|step| span / step <= max_ticks as f64)
            .unwrap_or(span);
        panel.set_y_ticks(Some((Fix(step), 0)), &[], &[]);
    }

    /// Saves the figure that `build` makes for a page as a PDF, and shows it in a window (unless
    /// `S_PLOT_NO_SHOW`), each one laid out for its own page
    fn output(build: impl Fn(&PlotLayout) -> gnuplot::Figure, panels: usize, legend: bool, pdf: &str) {
        let page = Self::pdf(panels, legend);
        let mut figure = build(&page);
        figure.set_terminal(
            &format!("pdfcairo size {}in,{}in font ',9'", page.width / 72.0, page.height / 72.0),
            pdf,
        );
        if let Err(_err) = figure.show() {
            hwa::error!("Unable to save {}: {:?}", pdf, _err)
        }
        // S_PLOT_NO_SHOW: only the files (batch runs, e.g. tuning sweeps)
        #[cfg(not(test))]
        if std::env::var("S_PLOT_NO_SHOW").is_err() {
            let window = Self::window(panels, legend);
            let mut figure = build(&window);
            // The interactive terminal of this gnuplot, at the size of the layout
            figure.set_pre_commands(&format!(
                "eval sprintf(\"set terminal %s size {},{} font ',9'\", GPVAL_TERM)",
                window.width, window.height
            ));
            if let Err(_err) = figure.show_and_keep_running() {
                hwa::error!("Unable to show plot: {:?}", _err)
            }
        }
    }
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

/// The ringing of the head on the frame, modeled as an oscillator (`S_PLOT_VIBRATION=<Hz>,<damping>`,
/// 40 Hz and 0.15 by default: the frame Marlin's input shaping defaults are tuned to) driven by the
/// X and Y motors as the step executor drives them: in each window the velocity ramps between the
/// rates at its ends, offset so that the window covers exactly its displacement (the second-order
/// DDA). The ringing is the oscillation of the head about the deflection of the frame under the
/// acceleration (`−a/ω²`, which no shaper removes). After the motion, it is followed for 0.2 s.
///
/// Two measures: over the whole motion, and the residual one (the ghosting of the prints): while
/// the motion is steady (constant acceleration of both axes, no velocity jump) since `1.2/f` at
/// least, longer than any shaper tuned at or above `f`. Right after a change no shaper can prevent
/// the first oscillation (its first impulse), only cancel it with the next ones: the residual
/// ringing is what input shaping (feature `with-input-shaping`) removes. Lower is better
struct Ringing {
    /// Frequency (Hz) and damping ratio of the frame
    model: (f64, f64),
    /// RMS and peak of X and Y
    rms: [f64; 2],
    peak: [f64; 2],
    rms_xy: f64,
    /// Residual (steady motion) RMS and peak of X and Y, and the fraction of the time it covers
    residual_rms: [f64; 2],
    residual_peak: [f64; 2],
    residual_rms_xy: f64,
    steady_fraction: f64,
    /// The free oscillation of the head where it rings undisturbed (the axis cruising, after its
    /// last velocity jump), as read on a print (ghosting): the half periods (s) and the ratios of
    /// successive half-cycle peaks, each weighted by its amplitude
    half_periods: Vec<(f64, f64)>,
    decays: Vec<(f64, f64)>,
    /// Every [Self::DT]
    times: Vec<f64>,
    ringing: [Vec<f64>; 2],
}

impl Ringing {
    /// Integration step
    const H: f64 = 1e-5;
    /// Sampling of the series
    const DT: f64 = 1e-3;

    fn model_from_env() -> (f64, f64) {
        std::env::var("S_PLOT_VIBRATION")
            .ok()
            .and_then(|v| {
                let (f, z) = v.split_once(',')?;
                Some((f.trim().parse::<f64>().ok()?, z.trim().parse::<f64>().ok()?))
            })
            .filter(|(f, z)| *f > 0.0 && (0.0..1.0).contains(z))
            .unwrap_or((40.0, 0.15))
    }

    fn analyze(windows: &[(f64, [f64; 2], [f64; 2])], model: (f64, f64)) -> Option<Self> {
        if windows.len() < 3 {
            return None;
        }
        let (w, zeta) = (2.0 * core::f64::consts::PI * model.0, model.1);
        let t_end = windows[windows.len() - 1].0 + 0.2;
        let n = (t_end / Self::DT) as usize + 1;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * Self::DT).collect();
        let mut out = Self {
            model,
            rms: [0.0; 2],
            peak: [0.0; 2],
            rms_xy: 0.0,
            residual_rms: [0.0; 2],
            residual_peak: [0.0; 2],
            residual_rms_xy: 0.0,
            steady_fraction: 0.0,
            half_periods: Vec::new(),
            decays: Vec::new(),
            times,
            ringing: [vec![0.0; n], vec![0.0; n]],
        };
        // The velocity of each window as the DDA executes it (its mean, the exact displacement,
        // ± half its ramp): (acceleration, start and end velocity) of X and Y
        let ramps: Vec<[(f64, f64, f64); 2]> = windows
            .windows(2)
            .map(|pair| {
                let ((t0, p0, v0), (t1, p1, v1)) = (pair[0], pair[1]);
                let dt = (t1 - t0).max(1e-9);
                [0, 1].map(|k| {
                    let a = (v1[k] - v0[k]) / dt;
                    let mean = (p1[k] - p0[k]) / dt;
                    (a, mean - a * dt / 2.0, mean + a * dt / 2.0)
                })
            })
            .collect();
        // When the motion of each window is steady from: the acceleration of the axes unchanged
        // (and no velocity jump) since `1.2/f`
        let settle = 1.2 / model.0;
        let mut quiet_since = windows[0].0;
        let steady_from: Vec<f64> = ramps
            .iter()
            .enumerate()
            .map(|(i, ramp)| {
                let changed = i == 0
                    || (0..2).any(|k| {
                        let prev = ramps[i - 1][k];
                        (ramp[k].0 - prev.0).abs() > 5.0 || (ramp[k].1 - prev.2).abs() > 0.05
                    });
                if changed {
                    quiet_since = windows[i].0;
                }
                quiet_since + settle
            })
            .collect();
        let (mut steady_time, mut total_time) = (0.0f64, 0.0f64);
        for k in 0..2 {
            // z'' + 2ζω z' + ω² z = −a: the head relative to the motor
            let (mut z, mut zv, mut v_end) = (0.0f64, 0.0f64, windows[0].2[k]);
            let (mut sum, mut samples, mut residual_sum, mut residual_samples) = (0.0f64, 0usize, 0.0f64, 0usize);
            let mut next = 0usize;
            // The free oscillation being followed: last zero crossing, peak since it, previous
            // half-cycle peak
            let mut free: Option<(Option<f64>, f64, Option<f64>)> = None;
            let last = windows[windows.len() - 1].0;
            // The windows, and the stop (0.2 s still)
            let spans = ramps
                .iter()
                .enumerate()
                .map(|(i, ramp)| (windows[i].0, windows[i + 1].0, ramp[k], steady_from[i]))
                .chain(core::iter::once((last, t_end, (0.0, 0.0, 0.0), last + settle)));
            for (t0, t1, (a, v_start, v_window_end), steady_from) in spans {
                let jump = v_start - v_end;
                zv -= jump;
                v_end = v_window_end;
                // Free while the axis cruises, from its last velocity jump
                if a.abs() > 1e-9 || jump.abs() > 1e-9 {
                    free = None;
                }
                if a.abs() <= 1e-9 && free.is_none() {
                    free = Some((None, 0.0, None));
                }
                let m = ((t1 - t0) / Self::H).ceil().max(1.0) as usize;
                let h = (t1 - t0) / m as f64;
                for i in 1..=m {
                    let z_prev = z;
                    zv += h * (-a - 2.0 * zeta * w * zv - w * w * z);
                    z += h * zv;
                    let ring = z + a / (w * w);
                    sum += ring * ring;
                    samples += 1;
                    out.peak[k] = out.peak[k].max(ring.abs());
                    let t = t0 + h * i as f64;
                    if t >= steady_from {
                        residual_sum += ring * ring;
                        residual_samples += 1;
                        out.residual_peak[k] = out.residual_peak[k].max(ring.abs());
                    }
                    while next < n && out.times[next] <= t {
                        out.ringing[k][next] = ring;
                        next += 1;
                    }
                    if let Some((crossing, peak, previous)) = free.as_mut() {
                        *peak = peak.max(z.abs());
                        if z_prev != 0.0 && (z > 0.0) != (z_prev > 0.0) {
                            let at = t - h + h * z_prev.abs() / (z_prev.abs() + z.abs());
                            // A complete half cycle, of an oscillation not lost in the precision
                            if let Some(c) = *crossing {
                                if *peak > 1e-7 {
                                    out.half_periods.push((*peak, at - c));
                                    if let Some(p) = *previous {
                                        out.decays.push((p, *peak / p));
                                    }
                                    *previous = Some(*peak);
                                }
                            }
                            *crossing = Some(at);
                            *peak = 0.0;
                        }
                    }
                }
            }
            out.rms[k] = (sum / samples.max(1) as f64).sqrt();
            out.residual_rms[k] = (residual_sum / residual_samples.max(1) as f64).sqrt();
            (steady_time, total_time) = (residual_samples as f64, samples as f64);
        }
        out.rms_xy = out.rms[0].hypot(out.rms[1]);
        out.residual_rms_xy = out.residual_rms[0].hypot(out.residual_rms[1]);
        out.steady_fraction = steady_time / total_time.max(1.0);
        Some(out)
    }

    fn report(&self) {
        hwa::info!(
            "[s_plot] Ringing of a {:.1} Hz, {:.3} damping frame: XY {:.5} {} RMS (X {:.5}, Y {:.5}), peak X {:.5} Y {:.5}. Lower is better",
            self.model.0,
            self.model.1,
            self.rms_xy,
            hwa::Contract::SPACE_UNIT_MAGNITUDE,
            self.rms[0],
            self.rms[1],
            self.peak[0],
            self.peak[1]
        );
        hwa::info!(
            "[s_plot] Residual ringing (steady motion, {:.0}% of the time): XY {:.6} {} RMS (X {:.6}, Y {:.6}), peak X {:.6} Y {:.6}. Lower is better",
            self.steady_fraction * 100.0,
            self.residual_rms_xy,
            hwa::Contract::SPACE_UNIT_MAGNITUDE,
            self.residual_rms[0],
            self.residual_rms[1],
            self.residual_peak[0],
            self.residual_peak[1]
        );
    }

    /// The resonance of the frame, from the free oscillation of the head (see [Self::half_periods]):
    /// natural frequency (Hz), damping ratio and the half cycles it was measured on. As one would
    /// measure it on the ghosting of a print: its period, and how fast it fades
    #[cfg_attr(not(feature = "with-input-shaping"), allow(dead_code))]
    fn identify(&self) -> Option<(f64, f64, usize)> {
        let mean = |pairs: &[(f64, f64)]| {
            let weight: f64 = pairs.iter().map(|(w, _)| w).sum();
            (weight > 0.0).then(|| pairs.iter().map(|(w, v)| w * v).sum::<f64>() / weight)
        };
        if self.half_periods.len() < 4 || self.decays.len() < 3 {
            return None;
        }
        let half_period = mean(&self.half_periods)?;
        let ratio = mean(&self.decays)?;
        if !(half_period > 0.0 && ratio > 0.0 && ratio < 1.0) {
            return None;
        }
        // Successive half-cycle peaks: e^(−πζ/√(1−ζ²))
        let delta = -ratio.ln();
        let zeta = delta / (core::f64::consts::PI * core::f64::consts::PI + delta * delta).sqrt();
        let f_damped = 0.5 / half_period;
        Some((f_damped / (1.0 - zeta * zeta).sqrt(), zeta, self.half_periods.len()))
    }

    fn write_csv(&self, path: &str) -> std::io::Result<()> {
        use std::io::Write;
        let mut file = std::fs::File::create(path)?;
        writeln!(file, "time,ringing_x,ringing_y")?;
        for i in 0..self.times.len() {
            writeln!(file, "{:.3},{:.6},{:.6}", self.times[i], self.ringing[0][i], self.ringing[1][i])?;
        }
        Ok(())
    }
}

/// Marlin's default shaper: ZV, 40 Hz, damping 0.15
#[cfg(feature = "with-input-shaping")]
const MARLIN_SHAPER: motion::ShaperConfig = motion::ShaperConfig {
    kind: motion::ShaperKind::Zv,
    frequency: hwa::math::Real::from_f32(40.0),
    damping: hwa::math::Real::from_f32(0.15),
};

/// The shaper forced by `S_PLOT_SHAPER=<type>,<frequency Hz>,<damping>` (type zv, ei, mzv, zvd or
/// 0-3; `off` disables it). `None` when not set: the shaper is tuned from the run without it
#[cfg(feature = "with-input-shaping")]
fn forced_shaper() -> Option<motion::ShaperConfig> {
    let spec = std::env::var("S_PLOT_SHAPER").ok()?;
    if spec.trim().eq_ignore_ascii_case("off") {
        return Some(motion::ShaperConfig::disabled());
    }
    let mut words = spec.split(',').map(str::trim);
    let kind = match words.next().map(|w| w.to_ascii_lowercase()) {
        Some(w) if w == "zv" || w == "0" => motion::ShaperKind::Zv,
        Some(w) if w == "ei" || w == "1" => motion::ShaperKind::Ei,
        Some(w) if w == "mzv" || w == "2" => motion::ShaperKind::Mzv,
        Some(w) if w == "zvd" || w == "3" => motion::ShaperKind::Zvd,
        _ => MARLIN_SHAPER.kind,
    };
    let mut number = |fallback: hwa::math::Real| {
        words.next().and_then(|w| w.parse::<f32>().ok()).map_or(fallback, hwa::math::Real::from_f32)
    };
    let frequency = number(MARLIN_SHAPER.frequency);
    let damping = number(MARLIN_SHAPER.damping);
    match motion::ShaperConfig::is_valid_frequency(frequency) && motion::ShaperConfig::is_valid_damping(damping) {
        true => Some(motion::ShaperConfig { kind, frequency, damping }),
        false => {
            hwa::warn!("[s_plot] S_PLOT_SHAPER out of range ({}): Marlin's defaults", spec.as_str());
            Some(MARLIN_SHAPER)
        }
    }
}

/// The title of a shaper
#[cfg(feature = "with-input-shaping")]
fn shaper_title(shaper: &motion::ShaperConfig) -> String {
    match shaper.frequency.is_zero() {
        true => String::from("off"),
        false => format!(
            "{:?} {:.2} Hz, damping {:.3}",
            shaper.kind,
            shaper.frequency.to_f64(),
            shaper.damping.to_f64()
        ),
    }
}

/// The impulses of a shaper: amplitude and delay (s), the first one included
#[cfg(feature = "with-input-shaping")]
fn impulses(config: &motion::ShaperConfig) -> Vec<(f64, f64)> {
    let shaper = motion::Shaper::new(config);
    let rest: f64 = shaper.delayed().iter().map(|(a, _)| a.to_f64()).sum();
    let mut impulses = vec![(1.0 - rest, 0.0)];
    impulses.extend(shaper.delayed().iter().map(|(a, t)| (a.to_f64(), *t as f64 * 1e-6)));
    impulses
}

/// The vibration the impulses leave in a frame of frequency `f` (Hz) and damping `zeta`, relative
/// to none (the classic V(ω, ζ) of the shaper theory)
#[cfg(feature = "with-input-shaping")]
fn residual_vibration(impulses: &[(f64, f64)], f: f64, zeta: f64) -> f64 {
    let t_n = impulses.last().map_or(0.0, |(_, t)| *t);
    let w = 2.0 * core::f64::consts::PI * f;
    let wd = w * (1.0 - zeta * zeta).sqrt();
    let (mut c, mut s) = (0.0, 0.0);
    for (a, t) in impulses.iter() {
        let e = (zeta * w * t).exp();
        c += a * e * (wd * t).cos();
        s += a * e * (wd * t).sin();
    }
    (-zeta * w * t_n).exp() * (c * c + s * s).sqrt()
}

/// The vibration a shaper leaves in a frame of each frequency (0-150 Hz) with the given damping,
/// relative to none
#[cfg(feature = "with-input-shaping")]
fn shaper_response(config: &motion::ShaperConfig, zeta: f64) -> (Vec<f64>, Vec<f64>) {
    let impulses = impulses(config);
    let freqs: Vec<f64> = (1..=300).map(|i| i as f64 * 0.5).collect();
    let response = freqs.iter().map(|f| residual_vibration(&impulses, *f, zeta)).collect();
    (freqs, response)
}

/// The shaper for the frame that rang in a run without input shaping: tuned to the resonance
/// identified from its free oscillation (see [Ringing::identify]); of the type with the best
/// compromise between the vibration it leaves, over ±10% of the frequency (the frame is not that
/// constant: the load, the position), and the smoothing of the corners (the variance of its
/// impulses in time). The score is Klipper's: `smoothing · (v^1.5 + 0.2·v + 0.01)`
#[cfg(feature = "with-input-shaping")]
fn tune(ringing: &Ringing) -> Option<motion::ShaperConfig> {
    let (f, zeta, half_cycles) = ringing.identify()?;
    hwa::info!(
        "[s_plot] Frame resonance identified from the ringing without shaping ({} half cycles of free oscillation): {:.2} Hz, damping {:.3}",
        half_cycles,
        f,
        zeta
    );
    let f = f.max(motion::MIN_FREQUENCY as f64);
    let zeta = zeta.clamp(0.0, motion::MAX_DAMPING as f64);
    let mut best: Option<(f64, motion::ShaperConfig)> = None;
    for kind in [motion::ShaperKind::Zv, motion::ShaperKind::Mzv, motion::ShaperKind::Ei, motion::ShaperKind::Zvd] {
        let config = motion::ShaperConfig {
            kind,
            frequency: hwa::math::Real::from_f32(f as f32),
            damping: hwa::math::Real::from_f32(zeta as f32),
        };
        let impulses = impulses(&config);
        let vibration = ((0..=20)
            .map(|i| residual_vibration(&impulses, f * (0.9 + 0.01 * i as f64), zeta).powi(2))
            .sum::<f64>()
            / 21.0)
            .sqrt();
        let centroid: f64 = impulses.iter().map(|(a, t)| a * t).sum();
        let smoothing: f64 = impulses.iter().map(|(a, t)| a * (t - centroid).powi(2)).sum();
        let score = smoothing * (vibration.powf(1.5) + 0.2 * vibration + 0.01);
        hwa::info!(
            "[s_plot]   {:?}: vibration left {:.1}% (±10% of the frequency), smoothing {:.1} ms², score {:.3e}",
            kind,
            vibration * 100.0,
            smoothing * 1e6,
            score
        );
        if best.is_none_or(|(s, _)| score < s) {
            best = Some((score, config));
        }
    }
    best.map(|(_, config)| config)
}

/// The motion of X and Y with input shaping: the shapers and the history of the firmware (see
/// `motion::input_shaping`) on the commanded motion (the continuous frame, mm)
#[cfg(feature = "with-input-shaping")]
struct ShapedXY {
    shapers: [motion::Shaper; 2],
    history: motion::ShaperHistory,
    now_us: u32,
    moving: bool,
}

#[cfg(feature = "with-input-shaping")]
impl ShapedXY {
    fn new(config: &[motion::ShaperConfig; 2]) -> Self {
        Self {
            shapers: [motion::Shaper::new(&config[0]), motion::Shaper::new(&config[1])],
            history: motion::ShaperHistory::new(),
            now_us: 0,
            moving: false,
        }
    }

    fn is_active(&self) -> bool {
        !self.shapers.iter().all(motion::Shaper::is_identity)
    }

    fn real(v: [f64; 2]) -> [hwa::math::Real; 2] {
        v.map(|x| hwa::math::Real::from_f32(x as f32))
    }

    /// A segment starts at `p`, leaving it with velocity `v`
    fn segment_starts(&mut self, p: [f64; 2], v: [f64; 2]) {
        match self.moving {
            true => self.history.junction(Self::real(v)),
            false => {
                self.moving = true;
                self.now_us = 0;
                self.history.start(0, Self::real(p), Self::real(v));
            }
        }
    }

    /// A window `dt_us` wide ends at `p` with velocity `v`
    fn window_ends(&mut self, dt_us: u32, p: [f64; 2], v: [f64; 2]) {
        self.now_us += dt_us;
        self.history.push(self.now_us, Self::real(p), Self::real(v));
    }

    /// The shaped position, velocity and acceleration of an axis `before_us` before the end of the
    /// last window
    fn shaped(&self, k: usize, before_us: u32) -> (f64, f64, f64) {
        let (r, v, a) = self.history.eval_with_acceleration(self.now_us, before_us, k);
        let (r, v, a) = (r.to_f64(), v.to_f64(), a.to_f64());
        let (mut ps, mut vs, mut as_) = (r, v, a);
        for (amplitude, delay) in self.shapers[k].delayed() {
            let (rd, vd, ad) = self.history.eval_with_acceleration(self.now_us, before_us + delay, k);
            let amplitude = amplitude.to_f64();
            ps += amplitude * (rd.to_f64() - r);
            vs += amplitude * (vd.to_f64() - v);
            as_ += amplitude * (ad.to_f64() - a);
        }
        (ps, vs, as_)
    }

    /// The commanded motion stopped: its shaped motion is completed (as the firmware does), with
    /// windows of the micro-segment period. Z (and the other data) stays where it is
    #[allow(clippy::too_many_arguments)]
    fn complete(
        &mut self,
        data_points: &mut instrumentation::data_points::DataPoints,
        window_ends: &mut Vec<(f64, [f64; 3])>,
        motor_windows: &mut Vec<(f64, [f64; 2], [f64; 2])>,
        last_p: [f64; 3],
        frame_offset: [f64; 2],
        axes_dt_s: Option<f64>,
    ) {
        if !self.moving {
            return;
        }
        self.history.junction([hwa::math::ZERO; 2]);
        let stopped = self.now_us;
        let duration = self.shapers[0].duration_us().max(self.shapers[1].duration_us());
        let frame = [last_p[0] - frame_offset[0], last_p[1] - frame_offset[1]];
        let dt_us = STEPPER_PLANNER_MICROSEGMENT_PERIOD_US;
        let dt = dt_us as f64 * 1e-6;
        while self.now_us - stopped < duration {
            self.window_ends(dt_us, frame, [0.0; 2]);
            data_points.reference_time += dt;
            let n_sub = match axes_dt_s {
                Some(sub) if dt > sub => (dt / sub).ceil() as usize,
                _ => 1,
            };
            for k in 1..=n_sub {
                let frac = k as f64 / n_sub as f64;
                let before_us = ((dt * (1.0 - frac)) * 1e6).round() as u32;
                let (mut p, mut v, mut a) = (last_p, [0.0f64; 3], [0.0f64; 3]);
                for j in 0..2 {
                    let (ps, vs, as_) = self.shaped(j, before_us);
                    (p[j], v[j], a[j]) = (ps + frame_offset[j], vs, as_);
                }
                data_points.axes.push(data_points.reference_time - dt * (1.0 - frac), p, v, a);
                if k == n_sub {
                    window_ends.push((data_points.reference_time, v));
                    motor_windows.push((data_points.reference_time, [p[0] - frame_offset[0], p[1] - frame_offset[1]], [v[0], v[1]]));
                }
            }
        }
        self.moving = false;
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
