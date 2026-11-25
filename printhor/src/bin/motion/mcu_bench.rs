//! On-target micro-benchmarks of the motion hot paths (feature `mcu-bench`), run by `M801`.
//!
//! The host benchmarks (`perf_bench`) run on a CPU with FPU: they rank the hot paths, but their
//! absolute times say little about a Cortex-M0+ without FPU nor hardware divide (SKR Mini E3),
//! where every float operation is a library call. These run on the board itself:
//! - an integer loop, to check the effective CPU speed (clock, flash wait states),
//! - the float primitives (`f32`/`f64` and the `Real` square and cube roots),
//! - the motion profile computation for pen-test-like segments,
//! - a micro-segment window (sampling + step interpolation), as the stepper task runs it.
//!
//! The executor is blocked while each benchmark runs (it yields in between): run it with the
//! machine idle. The step ticker interrupt (if running) is included in the times.
use crate::{hwa, motion};
use core::hint::black_box;
use hwa::math::Real;
use motion::{Constraints, HybridMotionProfile, MicroSegmentInterpolator, MotionProfile, SegmentSampler};

/// One result: name, value and unit
pub type BenchResult = (&'static str, u32, &'static str);

pub const NUM_BENCHES: usize = 20;

fn now_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}

/// Runs `f` `n` times: nanoseconds per call
fn ns_per_op(n: u32, mut f: impl FnMut()) -> u32 {
    let t = now_us();
    for _ in 0..n {
        f();
    }
    let dt = now_us().saturating_sub(t);
    (dt * 1000 / n.max(1) as u64).min(u32::MAX as u64) as u32
}

/// The constraints of the pen-test moves (F7000) with the tuned jerk
fn constraints() -> Constraints {
    Constraints {
        v_max: Real::from_f32(116.7),
        a_max: Real::from_f32(2400.0),
        j_max: Real::from_f32(50000.0),
    }
}

fn profile(s: f32, v_i: f32, v_o: f32) -> Option<HybridMotionProfile> {
    profile_a(s, v_i, v_o, 0.0, 0.0)
}

/// With boundary accelerations (a chained junction: polynomial blend). Without motion chaining
/// they are ignored
fn profile_a(s: f32, v_i: f32, v_o: f32, _a_i: f32, _a_o: f32) -> Option<HybridMotionProfile> {
    HybridMotionProfile::compute(
        black_box(Real::from_f32(s)),
        black_box(Real::from_f32(v_i)),
        black_box(Real::from_f32(v_o)),
        #[cfg(feature = "with-motion-chaining")]
        black_box(Real::from_f32(_a_i)),
        #[cfg(feature = "with-motion-chaining")]
        black_box(Real::from_f32(_a_o)),
        &constraints(),
        false,
    )
    .ok()
}

/// Runs every benchmark, yielding to the executor between them
pub async fn run() -> [BenchResult; NUM_BENCHES] {
    let mut r: [BenchResult; NUM_BENCHES] = [("", 0, ""); NUM_BENCHES];
    let mut i = 0;
    let mut push_unit = |name: &'static str, value: u32, unit: &'static str| {
        if i < NUM_BENCHES {
            r[i] = (name, value, unit);
            i += 1;
        }
    };
    let pause = || embassy_time::Timer::after_millis(2);
    macro_rules! push {
        ($name:expr, $ns:expr $(,)?) => {
            push_unit($name, $ns, "ns/op")
        };
    }

    // ~5 instructions per iteration: ~80 ns at 64 MHz without flash stalls
    let mut x = 1u32;
    push!(
        "int lcg (~5 instr)",
        ns_per_op(20_000, || x = black_box(x).wrapping_mul(1664525).wrapping_add(1013904223)),
    );
    pause().await;

    let (mut a, b) = (1.2345f32, 1.0001f32);
    push!("f32 add", ns_per_op(5_000, || a = black_box(a) + black_box(b)));
    push!("f32 mul", ns_per_op(5_000, || a = black_box(a) * black_box(b)));
    push!("f32 div", ns_per_op(5_000, || a = black_box(a) / black_box(b)));
    let mut lt = 0u32;
    push!("f32 cmp", ns_per_op(5_000, || lt += (black_box(a) < black_box(b)) as u32));
    black_box(lt);
    pause().await;
    let (mut c, d) = (1.2345f64, 1.0001f64);
    push!("f64 mul", ns_per_op(2_000, || c = black_box(c) * black_box(d)));
    push!("f64 div", ns_per_op(2_000, || c = black_box(c) / black_box(d)));
    pause().await;
    let mut e = Real::from_f32(2.5);
    push!("Real sqrt", ns_per_op(2_000, || e = black_box(black_box(e).sqrt().unwrap_or(e))));
    push!("Real cbrt", ns_per_op(1_000, || e = black_box(black_box(e).cbrt())));
    black_box((x, a, c, e));
    pause().await;

    // Pen-test segments (0.55 mm). Rest to rest, chained cruise, speed change, and a long move
    push!("profile 0.55mm 0->0", ns_per_op(20, || {
        black_box(profile(0.55, 0.0, 0.0));
    }));
    pause().await;
    push!("profile 0.55mm 60->60", ns_per_op(20, || {
        black_box(profile(0.55, 60.0, 60.0));
    }));
    pause().await;
    push!("profile 0.55mm 20->80", ns_per_op(20, || {
        black_box(profile(0.55, 20.0, 80.0));
    }));
    pause().await;
    push!("profile 50mm 0->0", ns_per_op(20, || {
        black_box(profile(50.0, 0.0, 0.0));
    }));
    pause().await;
    push!("profile 0.55mm blend 60->70", ns_per_op(20, || {
        black_box(profile_a(0.55, 60.0, 70.0, 800.0, 1000.0));
    }));
    pause().await;
    push!("profile 0.55mm blend 70->60", ns_per_op(20, || {
        black_box(profile_a(0.55, 70.0, 60.0, -1000.0, -800.0));
    }));
    pause().await;

    // Micro-segment windows of a 50 mm move, as the stepper task does them (2nd-order DDA)
    let (window_ns, windows) = match profile(50.0, 0.0, 0.0) {
        Some(p) => {
            let period_us =
                crate::tasks::task_stepper::STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as u64;
            let tick_us = crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US;
            let dir = hwa::make_vector_real!(x = 0.6, y = 0.8);
            let spm = hwa::make_vector_real!(x = 80.0, y = 80.0);
            let mut windows = 0u32;
            let t = now_us();
            let mut sampler = SegmentSampler::new_us(&p, period_us);
            let mut lin = MicroSegmentInterpolator::new(dir, p.end_pos(), spm);
            let mut v0 = p.eval_velocity(hwa::math::ZERO);
            while let Some(pos) = sampler.next() {
                let w = Real::from_lit(sampler.dt_us().max(1) as i64, 0);
                let v1 = sampler.current_speed();
                let _ = black_box(lin.advance_to_with_velocity(pos, w, v0, v1, tick_us));
                v0 = v1;
                windows += 1;
            }
            let dt = now_us().saturating_sub(t);
            ((dt * 1000 / windows.max(1) as u64).min(u32::MAX as u64) as u32, windows)
        }
        None => (0, 0),
    };
    push_unit("window sample+interp", window_ns, "ns/window");
    push_unit("windows measured", windows, "");
    pause().await;

    #[cfg(feature = "with-motion-chaining")]
    {
        let (per_move, per_chaining) = chaining();
        push_unit("chaining (pen, batched)", per_move, "ns/move");
        push_unit("chaining (pen, one)", per_chaining, "ns/chaining");
    }
    r
}

/// The planner on the pen-test: 0.55 mm moves along a gentle curve (3° turns) at F7000,
/// chained every 4 appended moves (as `MotionPlanner`). Returns the cost per appended move
/// (amortized) and of one chaining of the full queue (its window). The queue lives on the stack
/// of this (synchronous) function only.
#[cfg(feature = "with-motion-chaining")]
fn chaining() -> (u32, u32) {
    use hwa::controllers::motion_control::motion_ring_buffer::RingBuffer;
    use hwa::controllers::{PlanEntry, Segment};
    use hwa::HwiContract;
    let mutex = embassy_sync::mutex::Mutex::<
        <hwa::Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >::new(RingBuffer::new());
    let Ok(mut rb) = mutex.try_lock() else {
        return (0, 0);
    };
    let n = (<hwa::Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize).min(20);
    let rest = Real::from_f32(2.5);
    // One full step of a 80 steps/mm XY machine
    let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
    let mut chainings = 0u32;
    let t = now_us();
    for k in 0..n {
        let seg = Segment {
            speed_enter_su_s: rest,
            speed_exit_su_s: rest,
            accel_enter_su_s: hwa::math::ZERO,
            accel_exit_su_s: hwa::math::ZERO,
            speed_rest_su_s: rest,
            speed_target_su_s: constraints().v_max,
            displacement_su: Real::from_f32(0.55),
            speed_enter_constrained_su_s: constraints().v_max,
            speed_exit_constrained_su_s: constraints().v_max,
            proj_prev: Real::from_f32(if k == 0 { 1.0 } else { 0.9986 }),
            proj_next: Real::from_f32(0.9986),
            unit_vector_dir: hwa::make_vector_real!(x = 0.6, y = 0.8),
            src_pos: Default::default(),
            dest_pos: Default::default(),
            dest_world_pos: Default::default(),
            tool_power: hwa::math::ZERO,
            speed_programmed_su_s: hwa::math::ZERO,
            constraints: constraints(),
        };
        rb.data[k] = PlanEntry::PlannedMove(seg, Default::default(), Default::default(), false, k as u32);
        rb.used += 1;
        if (k + 1) % 4 == 0 || k + 1 == n {
            let _ = motion::perform_motion_chaining_after_append(&mut rb, tol);
            chainings += 1;
        }
    }
    let per_move = now_us().saturating_sub(t) * 1000 / n.max(1) as u64;
    let t = now_us();
    let _ = black_box(motion::perform_motion_chaining_after_append(&mut rb, tol));
    let one = now_us().saturating_sub(t) * 1000;
    black_box(chainings);
    (per_move.min(u32::MAX as u64) as u32, one.min(u32::MAX as u64) as u32)
}
