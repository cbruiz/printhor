//! Micro-benchmarks of the motion hot paths (host). Compiled only with the `perf-bench` feature;
//! run with:
//!
//! ```text
//! cargo test --release --features cartessian-cnc,perf-bench --bin printhor perf_bench -- --nocapture --test-threads=1
//! ```
//!
//! Absolute host timings are not MCU timings, but they are comparable between versions and
//! reveal where the time goes.
use crate::{hwa, motion};
use hwa::controllers::StepPlan;
use hwa::math::{CoordSel, Real};
#[allow(unused_imports)]
use hwa::HwiContract;
use motion::{Constraints, HybridMotionProfile, MicroSegmentInterpolator, MotionProfile, SegmentSampler};
use std::hint::black_box;
use std::time::Instant;

fn rnd(seed: &mut u64) -> f32 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*seed >> 40) as f32) / ((1u64 << 24) as f32)
}

fn constraints() -> Constraints {
    Constraints {
        v_max: Real::from_f32(200.0),
        a_max: Real::from_f32(3000.0),
        j_max: Real::from_f32(60000.0),
    }
}

fn compute(s: Real, vi: Real, vo: Real, _ai: Real, _ao: Real) -> Option<HybridMotionProfile> {
    HybridMotionProfile::compute(
        s,
        vi,
        vo,
        #[cfg(feature = "with-motion-chaining")]
        _ai,
        #[cfg(feature = "with-motion-chaining")]
        _ao,
        &constraints(),
        false,
    )
    .ok()
}

fn report(name: &str, ns_per_op: f64) {
    println!("PERF {:<44} {:>10.1} ns/op", name, ns_per_op);
}

#[test]
fn perf_bench_profile_compute() {
    let mut seed = 7u64;
    let cases: Vec<(Real, Real, Real)> = (0..2000)
        .map(|_| {
            (
                Real::from_f32(0.2 + 100.0 * rnd(&mut seed) * rnd(&mut seed)),
                Real::from_f32(150.0 * rnd(&mut seed)),
                Real::from_f32(150.0 * rnd(&mut seed)),
            )
        })
        .collect();
    let reps = 100;
    let t = Instant::now();
    for _ in 0..reps {
        for &(s, vi, vo) in &cases {
            black_box(compute(black_box(s), vi, vo, hwa::math::ZERO, hwa::math::ZERO));
        }
    }
    report(
        "profile compute (a_i = a_o = 0)",
        t.elapsed().as_nanos() as f64 / (reps * cases.len()) as f64,
    );

    #[cfg(feature = "with-motion-chaining")]
    {
        let cases: Vec<(Real, Real, Real, Real, Real)> = (0..2000)
            .map(|_| {
                let vi = 20.0 + 100.0 * rnd(&mut seed);
                let vo = 20.0 + 100.0 * rnd(&mut seed);
                (
                    Real::from_f32(1.0 + 10.0 * rnd(&mut seed)),
                    Real::from_f32(vi),
                    Real::from_f32(vo),
                    Real::from_f32(500.0 * (rnd(&mut seed) - 0.5)),
                    Real::from_f32(500.0 * (rnd(&mut seed) - 0.5)),
                )
            })
            .collect();
        let t = Instant::now();
        for _ in 0..reps {
            for &(s, vi, vo, ai, ao) in &cases {
                black_box(compute(black_box(s), vi, vo, ai, ao));
            }
        }
        report(
            "profile compute (a_i, a_o != 0: blend)",
            t.elapsed().as_nanos() as f64 / (reps * cases.len()) as f64,
        );
    }
}

/// One micro-segment window: sampler step + step interpolation (+ StepPlan build)
#[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))]
#[test]
fn perf_bench_window() {
    let profile = compute(
        Real::from_f32(50.0),
        hwa::math::ZERO,
        hwa::math::ZERO,
        hwa::math::ZERO,
        hwa::math::ZERO,
    )
    .unwrap();
    let period = Real::from_lit(2500, 6);
    let dir = hwa::make_vector_real!(x = 0.6, y = 0.8);
    let spm = hwa::make_vector_real!(x = 80.0, y = 80.0);
    let reps = 2000;
    let mut windows = 0u64;
    let t = Instant::now();
    for _ in 0..reps {
        let mut sampler = SegmentSampler::new(&profile, period);
        let mut lin = MicroSegmentInterpolator::new(dir, profile.end_pos(), spm);
        while let Some(p) = sampler.next() {
            let w = (sampler.dt() * hwa::math::ONE_MILLION).round();
            let _ = black_box(lin.advance_to(p, w));
            black_box(StepPlan::from(
                0,
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
                *lin.state(),
                CoordSel::X | CoordSel::Y,
                CoordSel::X | CoordSel::Y,
            ));
            windows += 1;
        }
    }
    report(
        "window: sample + interpolate (uniform)",
        t.elapsed().as_nanos() as f64 / windows as f64,
    );
    let mut windows = 0u64;
    let t = Instant::now();
    for _ in 0..reps {
        let mut sampler = SegmentSampler::new(&profile, period);
        let mut lin = MicroSegmentInterpolator::new(dir, profile.end_pos(), spm);
        let mut v0 = profile.eval_velocity(hwa::math::ZERO);
        while let Some(p) = sampler.next() {
            let w = Real::from_lit(sampler.dt_us() as i64, 0);
            let v1 = sampler.current_speed();
            let _ = black_box(lin.advance_to_with_velocity(p, w, v0, v1, 10));
            v0 = v1;
            black_box(StepPlan::from(
                0,
                #[cfg(feature = "with-motion-broadcast")]
                hwa::MotionDelta::new(),
                *lin.state(),
                CoordSel::X | CoordSel::Y,
                CoordSel::X | CoordSel::Y,
            ));
            windows += 1;
        }
    }
    report(
        "window: sample + interpolate (2nd-order DDA)",
        t.elapsed().as_nanos() as f64 / windows as f64,
    );
}

/// ISR cost: one StepPlan tick with two active channels
#[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))]
#[test]
fn perf_bench_isr_tick() {
    let dir = hwa::make_vector_real!(x = 0.6, y = 0.8);
    let spm = hwa::make_vector_real!(x = 80.0, y = 80.0);
    let mut lin = MicroSegmentInterpolator::new(dir, Real::from_f32(100.0), spm);
    lin.advance_to(Real::from_f32(0.4), Real::from_f32(2500.0)).unwrap();
    let mt = *lin.state();
    let reps = 40_000u64;
    let ticks = 250u64;
    let mut fired = 0u64;
    let t = Instant::now();
    for _ in 0..reps {
        let mut plan = StepPlan::from(
            0,
            #[cfg(feature = "with-motion-broadcast")]
            hwa::MotionDelta::new(),
            mt,
            CoordSel::X | CoordSel::Y,
            CoordSel::X | CoordSel::Y,
        );
        for _ in 0..ticks {
            if let Some(ch) = plan.next(black_box(10)) {
                fired += (!ch.is_empty()) as u64;
            }
        }
    }
    black_box(fired);
    report(
        "ISR tick (StepPlan::next, 2 channels)",
        t.elapsed().as_nanos() as f64 / (reps * ticks) as f64,
    );
}

/// Planner: perform_motion_chaining on a queue of 20 segments of a tessellated curve
#[cfg(feature = "with-motion-chaining")]
#[futures_test::test]
async fn perf_bench_chaining() {
    use hwa::controllers::motion_control::motion_ring_buffer::RingBuffer;
    use hwa::controllers::{PlanEntry, Segment};
    let n = (hwa::Contract::SEGMENT_QUEUE_SIZE as usize).min(20);
    let make_rb = || {
        let mut rb = RingBuffer::new();
        for k in 0..n {
            let a = 0.05 * k as f32;
            let (ux, uy) = (a.cos(), a.sin());
            let seg = Segment {
                displacement_su: Real::from_f32(0.55),
                speed_enter_su_s: Real::from_f32(7.0),
                speed_exit_su_s: Real::from_f32(if k == n - 1 { 7.0 } else { 200.0 }),
                speed_enter_constrained_su_s: Real::from_f32(200.0),
                speed_exit_constrained_su_s: Real::from_f32(200.0),
                accel_enter_su_s: hwa::math::ZERO,
                accel_exit_su_s: hwa::math::ZERO,
                t_1: hwa::math::ZERO,
                t_2: hwa::math::ZERO,
                t_3: hwa::math::ZERO,
                proj_prev: Real::from_f32(if k == 0 { 1.0 } else { 0.05f32.cos() }),
                proj_next: Real::from_f32(0.05f32.cos()),
                unit_vector_dir: hwa::make_vector_real!(x = 1.0)
                    .with_coord(CoordSel::X, Some(Real::from_f32(ux)))
                    .with_coord(CoordSel::Y, Some(Real::from_f32(uy))),
                src_pos: Default::default(),
                dest_pos: Default::default(),
                dest_world_pos: Default::default(),
                tool_power: Default::default(),
                constraints: constraints(),
                speed_target_su_s: Real::from_f32(200.0),
            };
            rb.data[k] = PlanEntry::PlannedMove(seg, Default::default(), Default::default(), false, k as u32);
        }
        rb.used = n as u8;
        rb.head = 0;
        rb
    };
    // Displacement tolerance of a 80 steps/mm XY machine (one step per axis), as in production
    let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
    let reps: usize = std::env::var("PERF_CHAINING_REPS").ok().and_then(|v| v.parse().ok()).unwrap_or(2000);
    let mut total = std::time::Duration::ZERO;
    for _ in 0..reps {
        let mutex = embassy_sync::mutex::Mutex::<hwa::AsyncNoopMutexType, _>::new(make_rb());
        let mut guard = mutex.lock().await;
        let t = Instant::now();
        let _ = black_box(motion::perform_motion_chaining(&mut guard, tol));
        total += t.elapsed();
    }
    report(
        &format!("perform_motion_chaining ({} segments)", n),
        total.as_nanos() as f64 / reps as f64,
    );
}

