//! Soft-float cost of the motion hot paths (feature `op-count`, host). The same cases as the
//! on-target benchmarks (`mcu_bench`, `M801`), measured as counted `Real` operations weighted by
//! their cost on the SKR Mini E3 (see `math::op_count`). Run with:
//!
//! ```text
//! cargo test --bin printhor --no-default-features --features native,cartessian-cnc-wip,op-count op_cost -- --nocapture --test-threads=1
//! ```
use crate::{hwa, motion};
use hwa::math::{op_count, Real};
use motion::{Constraints, HybridMotionProfile};

fn constraints() -> Constraints {
    Constraints {
        v_max: Real::from_f32(116.7),
        a_max: Real::from_f32(2400.0),
        j_max: Real::from_f32(50000.0),
    }
}

fn report(name: &str, n: u64) -> f64 {
    let c = op_count::get();
    let per: [u64; 9] = core::array::from_fn(|i| c[i] / n.max(1));
    let us = op_count::cost_us(&c) / n.max(1) as f64;
    let detail: std::vec::Vec<std::string::String> = op_count::NAMES
        .iter()
        .zip(per.iter())
        .filter(|(_, v)| **v > 0)
        .map(|(k, v)| std::format!("{}={}", k, v))
        .collect();
    std::println!("OPCOST {:<32} {:>9.0} us  [{}]", name, us, detail.join(" "));
    us
}

fn profile(s: f32, v_i: f32, v_o: f32, _a_i: f32, _a_o: f32) {
    let _ = HybridMotionProfile::compute(
        Real::from_f32(s),
        Real::from_f32(v_i),
        Real::from_f32(v_o),
        #[cfg(feature = "with-motion-chaining")]
        Real::from_f32(_a_i),
        #[cfg(feature = "with-motion-chaining")]
        Real::from_f32(_a_o),
        &constraints(),
        false,
    );
}

#[test]
fn op_cost_profile() {
    for (name, s, vi, vo, ai, ao) in [
        ("profile 0.55mm 0->0", 0.55f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32),
        ("profile 0.55mm 60->60", 0.55, 60.0, 60.0, 0.0, 0.0),
        ("profile 0.55mm 20->80", 0.55, 20.0, 80.0, 0.0, 0.0),
        ("profile 0.55mm 40->60", 0.55, 40.0, 60.0, 0.0, 0.0),
        ("profile 5mm 10->30", 5.0, 10.0, 30.0, 0.0, 0.0),
        ("profile 50mm 0->0", 50.0, 0.0, 0.0, 0.0, 0.0),
        ("profile 0.55mm blend 60->70", 0.55, 60.0, 70.0, 800.0, 1000.0),
        ("profile 0.55mm blend 70->60", 0.55, 70.0, 60.0, -1000.0, -800.0),
    ] {
        op_count::reset();
        profile(s, vi, vo, ai, ao);
        report(name, 1);
    }
}

/// The planner on the pen-test (as `mcu_bench::chaining`): 0.55 mm moves along a gentle curve
/// (3° turns) at F7000, chained every 4 appended moves
#[cfg(feature = "with-motion-chaining")]
#[test]
fn op_cost_chaining() {
    for batch in [1usize, 2, 4, 8, 12] {
        chaining_batched(batch, batch == 4);
    }
}

#[cfg(feature = "with-motion-chaining")]
fn chaining_batched(batch: usize, verbose: bool) {
    use hwa::controllers::motion_control::motion_ring_buffer::RingBuffer;
    use hwa::controllers::{PlanEntry, Segment};
    use hwa::HwiContract;
    let mutex = embassy_sync::mutex::Mutex::<
        <hwa::Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >::new(RingBuffer::new());
    let Ok(mut rb) = mutex.try_lock() else {
        panic!("lock");
    };
    let n = (<hwa::Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize).min(20);
    let rest = Real::from_f32(2.5);
    let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
    op_count::reset();
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
        if (k + 1) % batch == 0 || k + 1 == n {
            let _ = motion::perform_motion_chaining_after_append(&mut rb, tol);
        }
    }
    report(&std::format!("chaining (pen, batch {}) /move", batch), n as u64);
    if !verbose {
        return;
    }
    op_count::reset();
    motion::motion_chaining_counters::reset();
    let _ = motion::perform_motion_chaining_after_append(&mut rb, tol);
    std::println!("OPCOST counters [peak, floor/state, admissible, exact, newton, floors, stuck]: {:?}", motion::motion_chaining_counters::get());
    report("chaining (pen, one)", 1);
    // The resulting plan (to check that optimizations do not change it)
    let mut v = std::vec::Vec::new();
    for k in 1..=n as u8 {
        if let Ok(s) = rb.planned_segment_from_tail(k) {
            v.push(std::format!("{:.4}/{:.2}", s.speed_exit_su_s.inner(), s.accel_exit_su_s.inner()));
        }
    }
    std::println!("OPCOST plan(newest first): {}", v.join(" "));
}

/// The stepper task per segment on the pen-test circle (0.55 mm at F7000, 100 Hz windows):
/// profile + sampling + per-window second-order DDA, as `task_stepper` does
#[test]
fn op_cost_stepper_segment() {
    let v = Real::from_f32(116.7);
    let s = Real::from_f32(0.55);
    let dir = hwa::make_vector_real!(x = 0.6, y = 0.8);
    let spm = hwa::make_vector_real!(x = 80.0, y = 80.0);
    op_count::reset();
    let Ok(trajectory) = HybridMotionProfile::compute(
        s,
        v,
        v,
        #[cfg(feature = "with-motion-chaining")]
        hwa::math::ZERO,
        #[cfg(feature = "with-motion-chaining")]
        hwa::math::ZERO,
        &constraints(),
        false,
    ) else {
        panic!("profile");
    };
    report("stepper: profile", 1);
    op_count::reset();
    let mut sampler = motion::SegmentSampler::new(&trajectory, Real::from_f32(0.01));
    let mut interp = motion::MicroSegmentInterpolator::new_with_start_offset(
        dir.abs(),
        s,
        spm,
        hwa::make_vector_real!(x = 0.2, y = -0.3),
    );
    report("stepper: setup", 1);
    op_count::reset();
    use motion::MotionProfile;
    let mut v0 = trajectory.eval_velocity(hwa::math::ZERO);
    let mut windows = 0u64;
    let mut dda = [0u64; 9];
    loop {
        let Some(p) = sampler.next() else { break };
        windows += 1;
        let w = Real::from_lit(sampler.dt_us().max(1) as i64, 0);
        let v1 = sampler.current_speed();
        let c1 = op_count::get();
        let more = interp.advance_to_with_velocity(p, w, v0, v1, 20).unwrap_or(false);
        let c2 = op_count::get();
        for i in 0..9 {
            dda[i] += c2[i] - c1[i];
        }
        v0 = v1;
        if !more {
            break;
        }
    }
    std::println!("OPCOST windows={} dda only: {:.0} us", windows, op_count::cost_us(&dda));
    report("stepper: sampling+dda (all windows)", 1);
}
