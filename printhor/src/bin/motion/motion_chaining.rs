//! Motion Chaining Algorithm
//!
//! This module implements the motion chaining algorithm used to smooth the velocity profile
//! across consecutive motion segments.
//!
//! The goal is to preserve continuity of velocity and acceleration while respecting:
//! - segment target speed,
//! - axis/controller kinematic limits,
//! - geometry at junctions,
//! - and the finite distance available to decelerate/accelerate.
//!
//! The algorithm works in two layers:
//!
//! 1. A per-segment kinematic pass (`compute_maximum_enter_and_exit_speed`) that derives the
//!    constrained entry/exit speeds and feasible accelerations for the segment itself.
//! 2. A chained junction pass (`perform_motion_chaining`) that iteratively propagates corner
//!    limits through the buffer until the profile stabilizes.
//!
//! Junction limiting is not linear in the projection alone. Instead, we combine:
//! - geometric continuity from the dot product / projection,
//! - corner sharpness,
//! - and the distance needed to brake to a safe junction speed.
//!
//! This avoids the pathological behavior where every segment collapses to the same low speed.
//!
//! ## Junction accelerations
//!
//! Once the junction speeds are final, every interior junction gets the acceleration implied
//! by the velocity profile, `a = v·dv/ds`, with `dv/ds` estimated by the monotonicity-preserving
//! PCHIP formula. It is zero at every local velocity extremum (cruise plateau, velocity peak,
//! full-stop corner) and has the sign of the velocity trend. A final admissibility pass
//! guarantees, with the same predicate used by `HybridMotionProfile`, that every polynomial blend
//! exists and never reverses its velocity, so the stepper never has to alter the planned boundary
//! conditions: `a_o[n] == a_i[n+1]` holds for every junction of the plan.

use crate::hwa;
#[allow(unused)]
use hwa::{
    controllers::{motion_control::motion_ring_buffer::RingBuffer, PlanEntry},
    math,
    Contract, HwiContract,
};
use printhor_hwa_common::math::{Real, HALF, SIXTH};



#[cfg(test)]
std::thread_local! {
    /// Cost counters (tests): calls of max_speed_pass, braking_floor, segment_admissible,
    /// compute_maximum_enter_and_exit_speed, Newton iterations of jerk_ramp_time, braking floors
    /// raised from a fixed run start, and runs left infeasible at the end of the plan
    pub(crate) static CHAIN_COUNTERS: core::cell::Cell<[u32; 7]> = const { core::cell::Cell::new([0; 7]) };
}
#[cfg(test)]
fn count(i: usize) {
    CHAIN_COUNTERS.with(|c| {
        let mut v = c.get();
        v[i] += 1;
        c.set(v);
    });
}
#[cfg(not(test))]
#[inline(always)]
fn count(_i: usize) {}

/// Access to the cost counters (tests and benchmarks)
#[cfg(test)]
pub mod counters {
    pub fn reset() {
        super::CHAIN_COUNTERS.with(|c| c.set([0; 7]));
    }
    pub fn get() -> [u32; 7] {
        super::CHAIN_COUNTERS.with(|c| c.get())
    }
}

/// Speed limit of the junction of two moves, as a fraction of the lowest of their target speeds
/// `v_ref` (the chaining caps the junction at `v_ref` times it): the projection `cos θ` of their
/// directions (none above 90°), further limited so that the velocity vector, whose direction
/// changes at once there, jumps by no more than `dv`: `|Δv| = 2·v·sin(θ/2) ≤ dv`. Without that
/// limit a 60° corner of a 117 mm/s polyline was crossed at 58 mm/s: a 58 mm/s jump the motors
/// take without any ramp (a jolt). `dv ≤ 0` disables it.
///
/// Joint space machines (`with-motion-anthropomorphic-kinematics`, or any machine built with
/// `with-joint-space-corners`): the junctions are not
/// geometric corners but the samples of a joint trajectory, whose direction in the joint space
/// turns sharply wherever a joint starts or stops (a leg lifting off or landing), even when no
/// joint velocity jumps much. The projection is not a physical limit there: with `dv > 0` only the
/// velocity jump is limited (as the classic jerk of Marlin), and the projection applies when it
/// is disabled. With the projection, the samples of a walking gait stopped at every lift-off.
pub fn corner_speed_factor(cos_theta: Real, v_ref: Real, dv: Real) -> Real {
    let cos_theta = cos_theta.min(math::ONE);
    let proj = cos_theta.max(math::ZERO);
    if !(dv > math::ZERO) || !(v_ref > math::ZERO) || !(cos_theta < math::ONE) {
        return proj;
    }
    let bound = if JOINT_SPACE_CORNERS { math::ONE } else { proj };
    // sin(θ/2) = √((1 − cos θ)/2)
    match ((math::ONE - cos_theta) * HALF).sqrt() {
        Some(sin_half) if sin_half > math::ZERO => bound.min(dv / (math::TWO * sin_half * v_ref)),
        _ => bound,
    }
}

/// See [corner_speed_factor]: the junctions of a joint space machine are limited by the velocity
/// jump only. Always for the anthropomorphic kinematics, optionally for any machine
/// (`with-joint-space-corners`)
const JOINT_SPACE_CORNERS: bool = cfg!(any(
    feature = "with-motion-anthropomorphic-kinematics",
    feature = "with-joint-space-corners"
));

/// Throughput slowdown (in the spirit of Marlin's `SLOWDOWN`): the target speed of a new move so
/// that the planner, which needs `producer_period_s` per move, keeps the queue half full.
///
/// When the moves are shorter than what the planner can produce in the time they take (a curve
/// made of many short segments on a slow MCU), the queue empties: every plan has to brake to rest
/// within the few moves queued, and speeds up again when the next one arrives. The motion is then
/// a sawtooth at the segment rate (a rattle). Instead, each move is given at least the duration
/// `T·(1 + (N − 2n)/(2N))` (`n` moves queued out of `N`): `1.5·T` with the queue empty, `T` when
/// half full and `0.5·T` when full. The queue then settles around half full and the speed around
/// what the planner can sustain, constant along a uniform curve, with a long look-ahead.
///
/// The gain (½) trades how fast the queue fills (< 1 s on the pen-test circle) for the speed
/// ripple a change of one queued move causes (±2.5% with a 40-move queue). Moves that last
/// longer than `1.5·T` are never affected. One multiplication, plus a division when it slows
/// down the move.
pub fn throughput_speed_cap(
    displacement: Real,
    v_target: Real,
    queued: u8,
    queue_size: u8,
    producer_period_s: Real,
) -> Real {
    if queued >= queue_size || !(producer_period_s > math::ZERO) || !(v_target > math::ZERO) {
        return v_target;
    }
    let size = queue_size as i64;
    let num = Real::from_lit(3 * size - 2 * queued as i64, 0);
    let den = Real::from_lit(2 * size, 0);
    let t_min = producer_period_s * num / den;
    if displacement >= v_target * t_min {
        v_target
    } else {
        displacement / t_min
    }
}

/// Boundary accelerations below this fraction of `a_max` are routed to the S-Curve
/// (same threshold as `HybridMotionProfile`).
#[inline]
fn accel_negligible(a: Real, a_max: Real) -> bool {
    a.abs() <= a_max * Real::from_f32(1.0e-4)
}


/// `true` when the profile computed for these boundary conditions will reproduce them exactly:
/// zero-acceleration segments (S-Curve or monotone smoothstep blend) always do; segments with
/// non-zero boundary accelerations need an admissible polynomial blend.
///
/// The stepper does not execute the planned displacement `s` exactly: before computing the
/// profile it re-derives it from the actual (step-quantized) position (`Segment::fix_deviation`),
/// which changes it by up to `tol`. The blend must therefore be admissible over the whole interval
/// `[s - tol, s + tol]`; otherwise a plan that sits right at the feasibility boundary is accepted
/// here and then rejected by the stepper, which has to alter `a_o` (acceleration discontinuity).
/// The discriminant is linear in `s`, so checking both ends (plus the nominal value, for the
/// velocity-reversal test) covers the interval. A segment not longer than `2·tol` is dominated by
/// the step quantization and only admits zero boundary accelerations.
/// Cheap sufficient condition of [segment_admissible] (no square root): `true` guarantees that the
/// blend exists and never reverses its velocity for every displacement in `[s - tol, s + tol]`.
///
/// The blend duration is `T = 2s/(q + √Δ)` with `q = (v_i+v_o)/2` and `√Δ ≥ 0`, so `T ≤ 2(s+tol)/q`
/// for all of them. The Bernstein control points of the blend velocity are linear in `T` and at
/// `T = 0` they are `v_i` or `v_o` (non-negative): when they are also non-negative at the bound,
/// they are for every actual `T`, and so is the velocity (convex hull property). The discriminant
/// is linear in `s`: checking both ends of the interval covers it.
fn blend_surely_admissible(s: Real, v_i: Real, v_o: Real, a_i: Real, a_o: Real, tol: Real) -> bool {
    let q = (v_i + v_o) * HALF;
    let s_lo = s - tol;
    if !(q > math::ZERO) || !(s_lo > math::ZERO) || v_i < math::ZERO || v_o < math::ZERO {
        return false;
    }
    let s_hi = s + tol;
    let k = math::FOUR * math::TENTH * (a_i - a_o);
    let q2 = q * q;
    if q2 + k * s_lo < math::ZERO || q2 + k * s_hi < math::ZERO {
        return false;
    }
    let t = math::TWO * s_hi / q;
    let (dv, ta_i, ta_o) = (v_o - v_i, t * a_i, t * a_o);
    // c1, c2 = v_i + (1/5, 2/5)·ta_i: never negative when ta_i is not (v_i ≥ 0), so only
    // computed otherwise (same decision, fewer soft-float operations)
    if ta_i < math::ZERO {
        let c1 = v_i + ta_i * math::FIFTH;
        let c2 = v_i + ta_i * math::TWO * math::FIFTH;
        if c1 < math::ZERO || c2 < math::ZERO {
            return false;
        }
    }
    let b3 = math::TEN * dv - math::SIX * ta_i - math::FOUR * ta_o;
    let c3 = v_i + ta_i * math::THREE * math::FIFTH + b3 * math::TENTH;
    if c3 < math::ZERO {
        return false;
    }
    let b4 = math::EIGHT * ta_i + math::SEVEN * ta_o - math::FIFTEEN * dv;
    let c4 = v_i + ta_i * math::FOUR * math::FIFTH + b3 * math::FOUR * math::TENTH + b4 * math::FIFTH;
    !(c4 < math::ZERO)
}

fn segment_admissible(
    s: Real,
    v_i: Real,
    v_o: Real,
    a_i: Real,
    a_o: Real,
    a_max: Real,
    tol: Real,
) -> bool {
    count(2);
    if !(s > math::ZERO)
        || (v_i + v_o).is_negligible()
        || (accel_negligible(a_i, a_max) && accel_negligible(a_o, a_max))
    {
        return true;
    }
    if blend_surely_admissible(s, v_i, v_o, a_i, a_o, tol) {
        return true;
    }
    // The exact (sampled) predicate
    count(3);
    if !(tol > math::ZERO) {
        return crate::motion::blend_is_admissible(s, v_i, v_o, a_i, a_o);
    }
    // Both ends cover the interval: the discriminant is linear in s, and the blend velocity at
    // any instant τ (the sampled ones included) is linear in its duration T, which increases
    // with s, so its minimum over the interval is at one of the ends
    s > tol * math::TWO
        && crate::motion::blend_is_admissible(s + tol, v_i, v_o, a_i, a_o)
        && crate::motion::blend_is_admissible(s - tol, v_i, v_o, a_i, a_o)
}

/// Exit acceleration 5% inside the feasibility boundary of the polynomial blend with a fixed
/// entry state: any `a_o` at or below it has a blend (see `blend_accel_exit_cap`). The boundary
/// decreases with the displacement, so it is evaluated at the largest one the stepper may execute
/// (`s + tol`, see `segment_admissible`). The margin is taken over the admissible range
/// `[a_i, cap]`, so it never crosses the entry acceleration.
fn admissible_exit_accel_cap(s: Real, v_i: Real, v_o: Real, a_i: Real, tol: Real) -> Real {
    let cap = crate::motion::blend_accel_exit_cap(s + tol.max(math::ZERO), v_i, v_o, a_i);
    a_i + (cap - a_i) * Real::from_f32(0.95)
}

/// Mirror of [admissible_exit_accel_cap] for a fixed exit state: entry acceleration 5% inside
/// the feasibility boundary of the polynomial blend (any `a_i` at or above it has a blend).
fn admissible_entry_accel_floor(s: Real, v_i: Real, v_o: Real, a_o: Real, tol: Real) -> Real {
    let floor = a_o - (crate::motion::blend_accel_exit_cap(s + tol.max(math::ZERO), v_i, v_o, a_o) - a_o);
    a_o + (floor - a_o) * Real::from_f32(0.95)
}

/// Smallest `t ∈ (0, t_j]` at which the jerk ramp `j/6·t³ + a_i/2·t² + v_i·t` covers `s`, given
/// that it covers more than `s` at `t_j`.
///
/// Fast path (`a_i, v_i ≥ 0`, the usual case): the polynomial is convex and increasing on `t ≥ 0`,
/// so Newton's method started from an upper bound of the root decreases monotonically to it —
/// a few multiply/adds per iteration instead of the general Cardano solver (cbrt, sqrt, divisions,
/// possibly acos/cos). The start point is the smallest of the cheap single-term bounds
/// `t_j`, `s/v_i`, `√(2s/a_i)` (each term alone already covers `s` there).
/// Otherwise (decelerating start) the general cubic solver is used.
fn jerk_ramp_time(j: Real, a_i: Real, v_i: Real, s: Real, t_j: Real) -> Real {
    if a_i < math::ZERO || v_i < math::ZERO {
        return math::solve_cubic(j * SIXTH, a_i * HALF, v_i, -s)
            .filter(|t| *t <= t_j)
            .unwrap_or(t_j);
    }
    if !(v_i > math::ZERO) && !(a_i > math::ZERO) {
        // Pure jerk term: closed form
        return (math::SIX * s / j).cbrt().min(t_j);
    }
    let mut t = t_j;
    if v_i > math::ZERO {
        t = t.min(s / v_i);
    }
    if a_i > math::ZERO {
        if let Some(r) = (math::TWO * s / a_i).sqrt() {
            t = t.min(r);
        }
    }
    let (j6, j2, a2) = (j * SIXTH, j * HALF, a_i * HALF);
    for _ in 0..40 {
        count(4);
        let f = ((j6 * t + a2) * t + v_i) * t - s;
        let df = (j2 * t + a_i) * t + v_i;
        if !(df > math::ZERO) {
            break;
        }
        let dt = f / df;
        t = t - dt;
        // Monotone from above: a 1e-6 relative step is converged (the machine precision limit
        // took several more iterations, each one a handful of soft-float calls on a Cortex-M0+)
        if !(dt > t * Real::from_f32(1.0e-6)) {
            break;
        }
    }
    t.max(math::ZERO).min(t_j)
}

/// Computes the maximum exit speed and acceleration for a segment given an initial speed and acceleration.
/// This function models a simplified S-curve acceleration profile (constant jerk, then constant acceleration)
/// to find the highest possible speed that can be reached over the given displacement.
fn max_speed_pass(
    s_o: Real,
    v_i: Real,
    a_i: Real,
    constraints: &crate::motion::Constraints,
) -> (Real, Real, Real, Real, Real) {
    count(0);
    // Plan with the same effective jerk the profile will be executed with, otherwise the
    // planner would request velocity changes the profile cannot perform in this distance.
    let j_max = crate::motion::effective_jerk(constraints.j_max, crate::motion::DEFAULT_JERK_SHAPE);
    let a_max = constraints.a_max;
    let v_max_req = constraints.v_max;

    let t_j = (a_max - a_i).abs() / j_max;
    let v_j = a_i * t_j + HALF * j_max * t_j * t_j + v_i;
    let s_j = v_i * t_j + HALF * a_i * t_j * t_j + SIXTH * j_max * t_j * t_j * t_j;

    let (t1, t2, t3, v_max, a_max_actual) = if v_j > v_max_req {
        let t_v = (v_max_req - v_i - HALF * (a_max - a_i) * (a_max - a_i) / j_max) / a_max;
        if t_v < Real::from_f32(0.0) {
            let disc = ((v_max_req - v_i) * j_max + HALF * a_i * a_i).max(math::ZERO);
            let t_a = disc.sqrt().unwrap_or(math::ZERO) / j_max;
            (t_a, t_a, t_a, v_max_req, a_i + j_max * t_a)
        } else {
            (t_j, t_j + t_v, t_j + t_v, v_max_req, a_max)
        }
    } else if s_j > s_o {
        // A root exists in (0, t_j) because s(0) - s_o < 0 < s(t_j) - s_o. `solve_cubic` returns the
        // smallest positive root and never panics; t_j is a safe (conservative) fallback.
        let total_time = jerk_ramp_time(j_max, a_i, v_i, s_o, t_j);
        let v_exit = v_i + a_i * total_time + HALF * j_max * total_time * total_time;
        (total_time, total_time, total_time, v_exit, a_i + j_max * total_time)
    } else {
        let s_const_a = s_o - s_j;
        // Cancellation-free root of a/2·t² + v_j·t − s = 0: t = 2s / (v_j + √(v_j² + 2as))
        let root = (v_j * v_j + math::TWO * a_max * s_const_a)
            .sqrt()
            .unwrap_or(math::ZERO);
        let denom = v_j + root;
        let t_const_a = if denom > math::ZERO {
            math::TWO * s_const_a / denom
        } else {
            (root - v_j) / a_max
        };
        let v_exit = v_j + a_max * t_const_a;
        if v_exit > v_max_req {
            let t_v = (s_o - s_j) / v_max_req;
            (t_j, t_j + t_v, t_j + t_v, v_max_req, a_max)
        } else {
            (t_j, t_j + t_const_a, t_j + t_const_a, v_exit, a_max)
        }
    };
    (t1, t2, t3, v_max, a_max_actual)
}



/// For a given segment, calculates the maximum possible entry and exit speeds.
///
/// This function is a core part of the motion chaining algorithm. Its purpose is to determine
/// the kinematic limits of a segment, which are then used by `perform_motion_chaining` to
/// optimize junction speeds.
///
/// It performs two calculations:
/// 1. Forward pass: calculates the maximum possible exit speed and exit acceleration.
/// 2. Backward pass: calculates the maximum possible entry speed and entry acceleration.
///
/// The result is then clamped to preserve physically valid continuity when chained with
/// neighboring segments.
pub fn compute_maximum_enter_and_exit_speed(
    is_initial: bool,
    _num_order: u32,
    curr_segment: &mut hwa::controllers::Segment,
) {
    count(3);
    if is_initial {
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[motion_chaining] Planning initial profile for segment {:?}",
            _num_order
        );
    } else {
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[motion_chaining] Re-planning profile for segment @[head -{:?}]",
            _num_order
        );
    }

    let (_, _, _, v_exit_constr, a_exit_constr) = max_speed_pass(
        curr_segment.displacement_su,
        curr_segment.speed_enter_su_s,
        curr_segment.accel_enter_su_s,
        &curr_segment.constraints,
    );
    curr_segment.speed_exit_constrained_su_s = v_exit_constr;

    let (_, _, _, v_entry_constr, a_entry_constr_rev) = max_speed_pass(
        curr_segment.displacement_su,
        curr_segment.speed_exit_su_s,
        -curr_segment.accel_exit_su_s,
        &curr_segment.constraints,
    );
    curr_segment.speed_enter_constrained_su_s = v_entry_constr;

    if is_initial {
        curr_segment.accel_enter_su_s = Real::from_f32(0.0);
        curr_segment.accel_exit_su_s = Real::from_f32(0.0);
    } else {
        let a_i_raw = -a_entry_constr_rev;
        let a_o_raw = a_exit_constr;

        let v_sum = curr_segment.speed_enter_su_s + curr_segment.speed_exit_su_s;
        let s = curr_segment.displacement_su;
        let a_diff = a_i_raw - a_o_raw;

        // Polynomial-blend discriminant: Δ = (v_i+v_o)²/4 + 0.4·(a_i − a_o)·s ≥ 0
        let (a_i, a_o) = if !s.is_negligible() && a_diff < math::ZERO {
            let qb2 = v_sum * v_sum * math::QUARTER;
            let delta = qb2 + math::FOUR * math::TENTH * a_diff * s;
            if delta < math::ZERO {
                let k = -qb2 / (math::FOUR * math::TENTH * a_diff * s);
                (a_i_raw * k, a_o_raw * k)
            } else {
                (a_i_raw, a_o_raw)
            }
        } else {
            (a_i_raw, a_o_raw)
        };

        // Physical magnitude clamp: ensures the quintic velocity profile v(t) stays
        // non-negative throughout [0, T]. The bound |a| <= v*(v_i+v_o)/s comes from
        // requiring the linear approximation of velocity v(t) ~= v_i + a_i*t to not
        // cross zero before t=T ~= s/(v_i+v_o). This can only increase the discriminant
        // (because it reduces |a_i - a_o|), preserving the delta>=0 invariant.
        let (a_i, a_o) = {
            let v_i = curr_segment.speed_enter_su_s;
            let v_o = curr_segment.speed_exit_su_s;
            let v_sum = v_i + v_o;
            let s_seg = curr_segment.displacement_su;
            if !s_seg.is_negligible() && !v_sum.is_negligible() {
                let a_i_bound = (v_i * v_sum / s_seg).abs();
                let a_o_bound = (v_o * v_sum / s_seg).abs();
                (
                    a_i.clamp(-a_i_bound, a_i_bound),
                    a_o.clamp(-a_o_bound, a_o_bound),
                )
            } else {
                (a_i, a_o)
            }
        };
        curr_segment.accel_exit_su_s = a_o;
        curr_segment.accel_enter_su_s = a_i;
    }

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!(
        "[motion_chaining] Profile for segment @[head-{:?}]: v_enter_constr={:?}, v_exit_constr={:?}, a_enter={:?}, a_exit={:?}",
        _num_order,
        curr_segment.speed_enter_constrained_su_s,
        curr_segment.speed_exit_constrained_su_s,
        curr_segment.accel_enter_su_s,
        curr_segment.accel_exit_su_s
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Closed-form jerk-limited ramps (the planner's velocity model)
// ─────────────────────────────────────────────────────────────────────────────
//
// `R(v₀, v₁)`: displacement of an S-Curve ramp between the speeds `v₀` and `v₁` with zero
// acceleration at both ends, for `a_max` and the effective jerk `j` (the ones the profile is
// executed with). With `Δ = |v₁ - v₀|` and `t_j = a/j`:
//   Δ ≥ a²/j: R = (v₀+v₁)/2 · (t_j + Δ/a)
//   Δ < a²/j: R = (v₀+v₁)/2 · 2·√(Δ/j)
// `ramp_peak` and `ramp_floor` are its inverses in the upper and lower speed. They cost one square
// root (or a few Newton steps), where the former planner ran S-Curve passes (`max_speed_pass`)
// several times per segment and junction: ~600 soft-float operations per segment, about 2 ms on
// a Cortex-M0+ (the planner has ~1 ms per 0.5 mm segment at 117 mm/s).

/// An upper bound of `∛x` (`x > 0`) within ~0.3%: the bit-level seed and one Newton step (from
/// any side, a Newton step of the convex `t³ − x` lands above the root). A start for
/// [cubic_root_bounded], which needs no more
fn cbrt_rough(x: Real) -> Real {
    if !(x > math::ZERO) {
        return math::ZERO;
    }
    let xf = x.to_f64() as f32;
    let t = Real::from_f32(f32::from_bits(xf.to_bits() / 3 + 709_921_077));
    // (with a margin for the rounding of the step)
    (t - (t * t * t - x) / (math::THREE * t * t)) * Real::from_f32(1.0001)
}

/// Positive root of `t³ + p·t = q` (`p ≥ 0`, `q > 0`) given an upper bound `t_up` of it, to ~1e-5
/// relative. The root also lies above `q/(p + t_up²)`: Newton starts at the middle of both bounds
/// (from below the root, its first step lands above it, convexity, then it decreases to it), a
/// couple of iterations of a few operations.
fn cubic_root_bounded(p: Real, q: Real, t_up: Real) -> Real {
    let mut t = (t_up + q / (p + t_up * t_up)) * HALF;
    for _ in 0..12 {
        count(4);
        let t2 = t * t;
        let den = math::THREE * t2 + p;
        if !(den > math::ZERO) {
            break;
        }
        let dt = ((t2 + p) * t - q) / den;
        t = t - dt;
        // Quadratic convergence: the relative error left after a step is below the square of
        // the relative step (f''/2f' ≤ 1/t here), so a step under √1e-5 leaves it under 1e-5
        if !(dt.abs() > t * Real::from_f32(3.0e-3)) {
            break;
        }
    }
    t.max(math::ZERO).min(t_up)
}

/// `(a_max, effective jerk)` of a segment
#[inline]
fn ramp_limits(seg: &hwa::controllers::Segment) -> (Real, Real) {
    (
        seg.constraints.a_max,
        crate::motion::effective_jerk(seg.constraints.j_max, crate::motion::DEFAULT_JERK_SHAPE),
    )
}

/// `R(v₀, v₁)`, see above
fn ramp_distance(v0: Real, v1: Real, a: Real, j: Real) -> Real {
    let dv = (v1 - v0).abs();
    if !(dv > math::ZERO) {
        return math::ZERO;
    }
    let t = if dv * j >= a * a {
        a / j + dv / a
    } else {
        math::TWO * (dv / j).sqrt().unwrap_or(math::ZERO)
    };
    (v0 + v1) * HALF * t
}

/// Warm start of consecutive [ramp_peak_hinted] calls from the same speed and limits over growing
/// displacements (the backward and forward passes): the previous root
#[derive(Clone, Copy)]
struct PeakHint {
    key: (Real, Real, Real),
    q: Real,
    r: Real,
}

/// Highest `v₁ ≥ v₀` with `R(v₀, v₁) ≤ d`: the speed reachable (or, backwards, the speed from
/// which `v₀` is reachable braking) over the displacement `d`
fn ramp_peak(v0: Real, d: Real, a: Real, j: Real) -> Real {
    ramp_peak_hinted(v0, d, a, j, &mut None)
}

/// [ramp_peak], warm started from the previous call when it had the same `v₀`, `a`, `j` and a
/// shorter displacement (see `hint`), which it updates
fn ramp_peak_hinted(v0: Real, d: Real, a: Real, j: Real, hint: &mut Option<PeakHint>) -> Real {
    count(0);
    if !(d > math::ZERO) || !(a > math::ZERO) || !(j > math::ZERO) {
        return v0;
    }
    let t_j = a / j;
    let dv_b = a * t_j;
    // Displacement at the regime boundary (Δ = a²/j, T = 2·t_j)
    let d_b = (v0 + v0 + dv_b) * t_j;
    let dv = if d >= d_b {
        // (2v₀ + Δ)(t_j + Δ/a) = 2d  ⇔  Δ²/a + b·Δ + c = 0, cancellation-free positive root
        let b = math::TWO * v0 / a + t_j;
        let c = math::TWO * (v0 * t_j - d);
        let disc = (b * b - math::FOUR * c / a).max(math::ZERO);
        let den = b + disc.sqrt().unwrap_or(math::ZERO);
        if den > math::ZERO { -math::TWO * c / den } else { math::ZERO }
    } else {
        // (2v₀ + j·r²)·r = d with r = √(Δ/j) < t_j, i.e. r³ + (2v₀/j)·r = d/j. Upper bounds of
        // the root: t_j and each term alone (the cubic one through a cheap cube root, with a margin
        // that keeps it above the root, only when it is the smallest)
        let inv_j = j.recip();
        let (p, q) = (math::TWO * v0 * inv_j, d * inv_j);
        let mut r = t_j;
        if v0 > math::ZERO {
            r = r.min(d / (math::TWO * v0));
        }
        let key = (v0, a, j);
        match *hint {
            // The previous root r₀ (of q₀ ≤ q) is below this one: the tangent there (Newton from
            // below a convex function) lands above it, and close when the displacement grew little
            Some(h) if h.key == key && !(q < h.q) => {
                r = r.min(h.r + (q - h.q) / (math::THREE * h.r * h.r + p));
            }
            _ => {
                if !(r * r * r <= q) {
                    r = r.min(cbrt_rough(q));
                }
            }
        }
        let r = cubic_root_bounded(p, q, r);
        *hint = Some(PeakHint { key, q, r });
        j * r * r
    };
    v0 + dv.max(math::ZERO)
}

/// Lowest `v₁ ≤ v₀` with `R(v₁, v₀) ≤ d`: the speed `v₀` can brake down to over `d`. Never below
/// the exact value (conservative: an exit at or above it is always reachable)
fn ramp_floor(v0: Real, d: Real, a: Real, j: Real) -> Real {
    count(1);
    if !(v0 > math::ZERO) {
        return math::ZERO;
    }
    if !(d > math::ZERO) || !(a > math::ZERO) || !(j > math::ZERO) {
        return v0;
    }
    if ramp_distance(math::ZERO, v0, a, j) <= d {
        return math::ZERO;
    }
    let t_j = a / j;
    let dv_b = a * t_j;
    let d_b = if dv_b < v0 { (v0 + v0 - dv_b) * t_j } else { Real::from_f32(f32::MAX) };
    let dv = if d >= d_b {
        // (2v₀ − Δ)(t_j + Δ/a) = 2d  ⇔  Δ²/a − b·Δ + c = 0 (b = 2v₀/a − t_j, c = 2d − 2v₀·t_j),
        // smallest root, cancellation-free
        let b = math::TWO * v0 / a - t_j;
        let c = math::TWO * (d - v0 * t_j);
        let disc = (b * b - math::FOUR * c / a).max(math::ZERO);
        let den = b + disc.sqrt().unwrap_or(math::ZERO);
        if den > math::ZERO { math::TWO * c / den } else { math::ZERO }
    } else {
        // (2v₀ − j·r²)·r = d: concave and increasing where it matters. Newton from below (r with
        // f ≤ 0) increases monotonically to the root: an unconverged r only raises the floor
        let mut r = d / (math::TWO * v0);
        for _ in 0..12 {
            count(4);
            let f = (math::TWO * v0 - j * r * r) * r - d;
            let df = math::TWO * v0 - math::THREE * j * r * r;
            if !(df > math::ZERO) {
                break;
            }
            let dr = -f / df;
            if !(dr > r * Real::from_f32(1.0e-5)) {
                break;
            }
            r = r + dr;
        }
        j * r * r
    };
    (v0 - dv.max(math::ZERO)).max(math::ZERO)
}

/// State `(v, a)` at the displacement `x ∈ [0, R(v₀, v₁)]` of the rising ramp from `v₀` to `v₁ > v₀`
/// (zero acceleration at both ends). A falling ramp is its time reversal: same speed at `R − x`,
/// opposite acceleration.
#[cfg(test)]
fn ramp_state(v0: Real, v1: Real, x: Real, a: Real, j: Real) -> (Real, Real) {
    ramp_state_hinted(v0, v1, x, a, j, &mut None)
}

/// The constants of a rising ramp (see [ramp_state]), kept by [StateHint]
#[derive(Clone, Copy)]
struct RampShape {
    key: (Real, Real, Real, Real),
    /// Jerk phase duration, peak acceleration
    t_p: Real,
    a_pk: Real,
    /// Ramp distance
    r: Real,
    j6: Real,
    j2: Real,
    /// End of the first jerk phase (displacement, speed) and of the constant acceleration
    x1: Real,
    v1p: Real,
    x2: Real,
}

/// Consecutive [ramp_state_hinted] calls along the same ramp (the sampling of a run): its
/// constants, and the previous solution `(phase, target, root)` to warm start the next one
#[derive(Clone, Copy)]
struct StateHint {
    shape: RampShape,
    prev: Option<(u8, Real, Real)>,
}

/// [ramp_state], reusing the constants of the ramp and warm starting from the previous call along
/// it (see `hint`, which it updates)
fn ramp_state_hinted(v0: Real, v1: Real, x: Real, a: Real, j: Real, hint: &mut Option<StateHint>) -> (Real, Real) {
    count(1);
    let dv = v1 - v0;
    if !(dv > math::ZERO) || !(x > math::ZERO) || !(a > math::ZERO) || !(j > math::ZERO) {
        return (v0, math::ZERO);
    }
    let key = (v0, v1, a, j);
    let h = match hint {
        Some(h) if h.shape.key == key => h,
        _ => {
            // Jerk phase duration, peak acceleration, constant acceleration duration and total
            // duration (that of `ramp_distance`, without its square root again)
            let (t_p, a_pk, t_c, t_r) = if dv * j >= a * a {
                let (t_j, t_dv) = (a / j, dv / a);
                (t_j, a, t_dv - t_j, t_j + t_dv)
            } else {
                let t_p = (dv / j).sqrt().unwrap_or(math::ZERO);
                (t_p, j * t_p, math::ZERO, math::TWO * t_p)
            };
            let (j6, j2) = (j * SIXTH, j * HALF);
            let x1 = (v0 + j6 * t_p * t_p) * t_p;
            let v1p = v0 + j2 * t_p * t_p;
            let shape = RampShape {
                key,
                t_p,
                a_pk,
                r: (v0 + v1) * HALF * t_r,
                j6,
                j2,
                x1,
                v1p,
                x2: x1 + (v1p + a_pk * t_c * HALF) * t_c,
            };
            hint.insert(StateHint { shape, prev: None })
        }
    };
    let RampShape { t_p, a_pk, r, j6, j2, x1, v1p, x2, .. } = h.shape;
    if !(x < r) {
        return (v1, math::ZERO);
    }
    if !(x > x1) {
        // v₀·t + j·t³/6 = x  ⇔  t³ + (6v₀/j)·t = 6x/j: upper bounds t_p, x/v₀, and the previous
        // root (a shorter x: the tangent there, Newton from below a convex function, lands above)
        let mut t = t_p;
        if v0 > math::ZERO {
            t = t.min(x / v0);
        }
        if let Some((1, x_prev, t_prev)) = h.prev {
            t = t.min(match x < x_prev {
                true => t_prev,
                false => t_prev + (x - x_prev) / (v0 + j2 * t_prev * t_prev),
            });
        }
        let inv_j6 = j6.recip();
        let t = cubic_root_bounded(v0 * inv_j6, x * inv_j6, t);
        h.prev = Some((1, x, t));
        return (v0 + j2 * t * t, j * t);
    }
    if !(x > x2) {
        // Constant acceleration: a/2·τ² + v·τ − Δx = 0, cancellation-free root
        let dx = x - x1;
        let root = (v1p * v1p + math::TWO * a_pk * dx).sqrt().unwrap_or(math::ZERO);
        let den = v1p + root;
        let tau = if den > math::ZERO { math::TWO * dx / den } else { math::ZERO };
        return (v1p + a_pk * tau, a_pk);
    }
    // Last jerk phase, measured backwards from the end: v₁·τ − j·τ³/6 = R − x (concave
    // increasing): Newton from below increases to the root. Lower bounds: xr/v₁, and a Newton
    // step from the previous root (the tangent lies above a concave function: it lands below)
    let xr = r - x;
    let mut tau = xr / v1;
    if let Some((3, _, t_prev)) = h.prev {
        let df = v1 - j2 * t_prev * t_prev;
        if df > math::ZERO {
            tau = tau.max(t_prev - ((v1 - j6 * t_prev * t_prev) * t_prev - xr) / df);
        }
    }
    for _ in 0..12 {
        count(4);
        let f = (v1 - j6 * tau * tau) * tau - xr;
        let df = v1 - j2 * tau * tau;
        if !(df > math::ZERO) {
            break;
        }
        let dt = -f / df;
        if !(dt > tau * Real::from_f32(1.0e-5)) {
            break;
        }
        tau = tau + dt;
        // Quadratic convergence: the relative error left is below ~2 times the square of the
        // relative step (|f''|/2f' ≤ 2/τ here), under 1e-5 for a step under 2e-3
        if !(dt > tau * Real::from_f32(2.0e-3)) {
            break;
        }
    }
    let tau = tau.min(t_p);
    h.prev = Some((3, xr, tau));
    (v1 - j2 * tau * tau, j * tau)
}

/// Inverse of [ramp_state] in the speed: the displacement `x ∈ [0, R(v₀, v₁)]` of the rising
/// ramp from `v₀` to `v₁` at which the speed is `v` (closed form: one square root)
fn ramp_position(v0: Real, v1: Real, v: Real, a: Real, j: Real) -> Real {
    let dv = v1 - v0;
    if !(dv > math::ZERO) || !(v > v0) || !(a > math::ZERO) || !(j > math::ZERO) {
        return math::ZERO;
    }
    let r = ramp_distance(v0, v1, a, j);
    if !(v < v1) {
        return r;
    }
    let (t_p, a_pk, t_c) = if dv * j >= a * a {
        (a / j, a, dv / a - a / j)
    } else {
        let t_p = (dv / j).sqrt().unwrap_or(math::ZERO);
        (t_p, j * t_p, math::ZERO)
    };
    let v1p = v0 + j * t_p * t_p * HALF;
    if !(v > v1p) {
        let t = (math::TWO * (v - v0) / j).sqrt().unwrap_or(math::ZERO);
        return (v0 + j * t * t * SIXTH) * t;
    }
    let x1 = (v0 + j * t_p * t_p * SIXTH) * t_p;
    if !(v > v1p + a_pk * t_c) {
        let tau = (v - v1p) / a_pk;
        return x1 + (v1p + a_pk * tau * HALF) * tau;
    }
    // Last jerk phase, measured backwards from the end
    let tau = (math::TWO * (v1 - v) / j).sqrt().unwrap_or(math::ZERO);
    (r - (v1 - j * tau * tau * SIXTH) * tau).max(math::ZERO)
}

/// Speed limit (cap) of the junction between `older` and `seg`: the lowest of their target speeds
/// scaled by the corner projection (≈1 collinear, 0 at a 90° corner), and never below the rest
/// speed of the junction (both segments start, stop and therefore turn at it instantly)
#[inline]
fn junction_cap(older: &hwa::controllers::Segment, seg: &hwa::controllers::Segment) -> Real {
    (older.speed_target_su_s.min(seg.speed_target_su_s) * seg.proj_prev.max(math::ZERO))
        .max(older.speed_rest_su_s.min(seg.speed_rest_su_s))
}

/// Number of newest planned segments to (re)chain (the window), at least 2 and at most `used`.
///
/// Appending a segment only relaxes the plan (the end of the queue moves away), so only the
/// junctions within braking distance of the end can change: the window covers the stopping
/// distance from the highest target speed, plus the newest segment, and starts at a junction with
/// zero acceleration at its cap. Older junctions keep their planned values, and the oldest segment
/// of the window is treated as the pinned one (its entry state is the exit state of the frozen
/// segment before it).
fn chain_window(rb: &RingBuffer, used: u8) -> Result<u8, ()> {
    let (mut dist, mut v_ref) = (math::ZERO, math::ZERO);
    let (mut a, mut j) = (Real::from_f32(f32::MAX), Real::from_f32(f32::MAX));
    let newest_len = rb.planned_segment_from_tail(1)?.displacement_su;
    // Segments appended since the last chaining wait at their own rest speed, which may differ
    // from the exit of the segment before (a slow move after a fast one): every such junction must
    // be inside the window (not its pinned entry), or that speed jump stays in the plan.
    //
    // So must the junction before the oldest of them: the previous end of the plan, where the
    // previous chaining stopped at rest. When the speeds on both sides of it are equal (the
    // common rest speed of a uniform machine), the check below can not see it: with batched
    // chaining (several segments appended per chaining), the window could start right there
    // and freeze that stop in the plan for good (a halt every batch along a smooth trajectory)
    let mut min_w = rb.unchained.max(1);
    for i in 1..used {
        let (seg, older) = (rb.planned_segment_from_tail(i)?, rb.planned_segment_from_tail(i + 1)?);
        if older.speed_exit_su_s != seg.speed_enter_su_s || older.accel_exit_su_s != seg.accel_enter_su_s {
            min_w = min_w.max(i);
        }
    }
    // Stopping distance from v_ref, recomputed only when (v_ref, a, j) change (a square root and
    // two divisions: the same for every segment of a uniform queue)
    let (mut brake, mut brake_key) = (math::ZERO, None);
    for i in 1..=used {
        let seg = rb.planned_segment_from_tail(i)?;
        let (a_i, j_i) = ramp_limits(seg);
        a = a.min(a_i);
        j = j.min(j_i);
        v_ref = v_ref.max(seg.speed_target_su_s);
        dist += seg.displacement_su;
        // The window starts at a junction with zero acceleration (cruise, velocity extremum, rest):
        // the forward pass starts there as a fresh ramp, which would lose the momentum of a ramp
        // in progress (its entry acceleration).
        //
        // And at its cap: a junction below it (a plateau, a velocity peak) is set by a constraint
        // downstream, such as a corner within the window braking to the end of the plan. That
        // corner relaxes as the end moves away, and the chaining of the whole queue then raises
        // the junctions before it: frozen, they would stay slower (up to 12.5 mm/s, see
        // `test_windowed_chaining_relaxes_plateau_before_corner`)
        if i > min_w && seg.accel_enter_su_s.is_zero() {
            if brake_key != Some((v_ref, a, j)) {
                brake = ramp_distance(math::ZERO, v_ref, a, j);
                brake_key = Some((v_ref, a, j));
            }
            if dist >= brake + newest_len {
                if i == used {
                    return Ok(i);
                }
                // (with the rounding margin of the sampled junction speeds)
                let cap = junction_cap(rb.planned_segment_from_tail(i + 1)?, seg);
                if !(seg.speed_enter_su_s < cap * Real::from_f32(1.0 - 1.0e-4)) {
                    return Ok(i);
                }
            }
        }
    }
    Ok(used)
}

/// Performs the motion chaining (look-ahead) on the ring buffer, after a segment was appended.
///
/// The junction speeds come from a closed-form model of jerk-limited ramps (see `ramp_peak`),
/// over the window of segments the new one can influence (see `chain_window`):
///
/// 1. **Caps**: each junction is limited by the target speed of both sides, scaled by the corner
///    projection (≈1 collinear, 0 at a 90° corner), and never below the rest speed of the
///    junction (the segments start, stop and therefore turn at it instantly).
/// 2. **Backward pass** (newest → oldest) from the exit of the newest segment (the stop at the end
///    of the plan): each junction is limited to the speed from which the plan can brake to the
///    last cap-limited junction (the *anchor*) over the displacement in between, as a single ramp.
///    Braking (and accelerating) is therefore spread over consecutive short segments.
/// 3. **Forward pass** (oldest → newest) from the pinned entry, the same accelerating.
/// 4. **Braking floor**: the pinned entry can not be lowered, so no junction is planned below the
///    speed it can brake down to (it only binds after the window moved, or with corners).
/// 5. **Junction accelerations** (PCHIP estimate of `v·dv/ds`, only between ramp-limited
///    segments) and **blend admissibility** (the predicate the profile uses), as before.
///
/// Invariants (as before): `older.exit == newer.enter` for speeds and accelerations, the pinned
/// entry state of the oldest segment of the window is never modified, the newest segment exits
/// at its planned (rest) speed with zero acceleration, and every polynomial blend is admissible
/// for any displacement in `[s - tol, s + tol]`.
///
/// `displacement_tolerance` is the largest change of a segment displacement the stepper may
/// introduce when it corrects the position deviation before executing it (see
/// `segment_admissible`); `0` plans for the exact displacements.
pub fn perform_motion_chaining(
    rb: &mut embassy_sync::mutex::MutexGuard<
        <Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >,
    displacement_tolerance: Real,
) -> Result<(), ()> {
    chain(rb, displacement_tolerance, false)
}

/// [perform_motion_chaining] right after a segment was appended to a plan that was already
/// chained (the planner): only the window the new segment can influence is (re)chained (see
/// `chain_window`), so the cost is usually bounded by the braking distance instead of the queue
/// length (the whole queue when the window can not be planned from its pinned entry, see `chain`).
pub fn perform_motion_chaining_after_append(
    rb: &mut embassy_sync::mutex::MutexGuard<
        <Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >,
    displacement_tolerance: Real,
) -> Result<(), ()> {
    chain(rb, displacement_tolerance, true)
}

fn chain(
    rb: &mut embassy_sync::mutex::MutexGuard<
        <Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >,
    displacement_tolerance: Real,
    windowed: bool,
) -> Result<(), ()> {
    let tol = displacement_tolerance.max(Real::zero());
    let used = rb.used;
    if used < 2 {
        return Ok(());
    }

    // Count consecutive PlannedMove segments from the tail (newest first). SetPosition, Homing,
    // Dwell, Executing or Empty entries act as a chain boundary.
    let used = {
        let mut n = 0u8;
        for offset in 1..=used {
            if rb.planned_segment_from_tail(offset).is_ok() {
                n = offset;
            } else {
                break;
            }
        }
        n
    };
    if used < 2 {
        return Ok(());
    }
    let mut w = if windowed { chain_window(rb, used)? } else { used } as usize;

    // Previous-plan fallback: a plan that can only be made feasible by raising braking floors (or
    // not at all) from its pinned entry is not committed. The pinned state may be off the ramp
    // model (a blend state left by the admissibility repair, e.g. a corner at its limit with a
    // non-zero acceleration), which the previous plan executes but the ramp model can not
    // reproduce. The previous plan is kept up to its first junction with zero acceleration, a
    // state of the ramp model, and only the newer segments are planned again from there. The
    // previous plan is valid (continuous and admissible), and appending only relaxes it.
    //
    // A window is planned again over the whole queue first (the exact chaining, as
    // `perform_motion_chaining`), not counted as an attempt: its pinned entry may be a junction
    // the previous plan brakes from with blends (a cruise at its cap a few segments before a
    // corner), from which the ramp model can not brake in time, while an older state can. The
    // fallback would keep the previous plan up to the segments just appended, which then keep
    // their rest-to-rest plan: a full stop right after the corner, kept by the next fallbacks for
    // good (see `test_windowed_chaining_never_freezes_a_stop_after_a_corner`)
    let mut whole_queue = w == used as usize;
    let mut attempt = 0;
    loop {
        match chain_plan(rb, tol, used, w, windowed, attempt + 1 < MAX_PLAN_ATTEMPTS)? {
            Some(_) if !whole_queue => {
                whole_queue = true;
                w = used as usize;
            }
            Some(m) if w - m >= 2 => {
                w -= m;
                attempt += 1;
            }
            _ => break,
        }
    }
    Ok(())
}

/// Previous-plan fallback of [chain_plan]: the previous plan is kept up to junction `m` of the
/// window of `w` segments, whose entry state becomes the exit state of the previous plan there
/// (`v_exit`, zero acceleration), the pinned entry of the next attempt over the `w - m` newest
fn keep_previous_plan(
    rb: &mut embassy_sync::mutex::MutexGuard<
        <Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >,
    w: usize,
    m: usize,
    v_exit: Option<Real>,
) -> Result<Option<usize>, ()> {
    if let Some(v_exit) = v_exit {
        let seg = rb.mut_planned_segment_from_tail((w - m) as u8)?;
        seg.speed_enter_su_s = v_exit;
        seg.accel_enter_su_s = math::ZERO;
    }
    Ok(Some(m))
}

/// Plans to try in [chain]: the first one, and up to this minus one previous-plan fallbacks
const MAX_PLAN_ATTEMPTS: usize = 4;

/// One planning attempt of [chain] over the window of the `w` newest segments. Returns
/// `Some(m)` without committing anything when the plan needs braking floors and `fallback` is
/// allowed: the previous plan is kept up to its junction `m` (zero acceleration), see [chain].
fn chain_plan(
    rb: &mut embassy_sync::mutex::MutexGuard<
        <Contract as HwiContract>::MotionRingBufferMutexType,
        RingBuffer,
    >,
    tol: Real,
    #[allow(unused)] used: u8,
    w: usize,
    #[allow(unused)] windowed: bool,
    fallback: bool,
) -> Result<Option<usize>, ()> {
    const N: usize = <Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize;

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!("[motion_chaining] START. Segments: {} window: {}", used, w);

    // Window, oldest (k = 0, pinned) to newest (k = w-1). Junction k (1..w) is the entry of
    // segment k; junction 0 is the pinned entry and junction w the exit of the newest segment.
    let mut s = [math::ZERO; N];
    let mut lim = [(math::ZERO, math::ZERO); N];
    // Speed below which a junction is at rest (same criterion as `HybridMotionProfile`)
    let mut rest_tol = [math::ZERO; N + 1];
    let mut cap = [math::ZERO; N];
    // Junction speeds of the previous plan (still reachable: appending only relaxes it), and
    // its exit speed of the segment before each junction when the previous plan can be kept up to
    // that junction (see the fallback): zero acceleration on both sides (a state of the ramp
    // model). The two sides may differ in speed: a newly appended segment still waits at its own
    // rest speed, which may differ from the exit of the one before; the exit is kept
    let mut v_prev = [math::ZERO; N];
    let mut prev_keep = [None; N];
    let mut prev_exit = (math::ZERO, math::ZERO);
    let mut v = [math::ZERO; N + 1];
    let (v_pin, a_pin, v_end) = {
        let oldest = rb.planned_segment_from_tail(w as u8)?;
        let newest = rb.planned_segment_from_tail(1)?;
        (oldest.speed_enter_su_s, oldest.accel_enter_su_s, newest.speed_exit_su_s)
    };
    let mut older: Option<&hwa::controllers::Segment> = None;
    for k in 0..w {
        let seg = rb.planned_segment_from_tail((w - k) as u8)?;
        s[k] = seg.displacement_su;
        lim[k] = ramp_limits(seg);
        let r = seg.constraints.v_max * Real::from_f32(1.0e-5);
        rest_tol[k] = rest_tol[k].max(r);
        rest_tol[k + 1] = r;
        if let Some(older) = older {
            // Never below the rest speed of the junction: both segments start and stop at their
            // rest speed instantly, so they can also turn any corner at it. A lower cap (a sharp
            // corner, 0 at 90°) can not be reached when the plan ended at rest right there: the
            // braking floor raised it anyway
            cap[k] = junction_cap(older, seg);
            v_prev[k] = seg.speed_enter_su_s;
            if seg.accel_enter_su_s.is_zero() && prev_exit.1.is_zero() {
                prev_keep[k] = Some(prev_exit.0);
            }
        }
        prev_exit = (seg.speed_exit_su_s, seg.accel_exit_su_s);
        older = Some(seg);
    }
    v[0] = v_pin;
    v[w] = v_end;

    // Pinned entry with a non-zero acceleration: a ramp in progress. It is a point of a fresh
    // ramp (zero acceleration) that started `t = |a|/j` earlier, at `v_virt = v − a·t/2`, a
    // displacement `t·(v_virt + a·t/6)` back. Modeling it as a fresh ramp from the pinned speed
    // instead counts the jerk phase again (a brake looked infeasible and its corners were raised
    // by the braking floor) and, when sampled, plans junction states the next chaining (pinned at
    // one of them) finds inconsistent. Only for a run in the direction of the acceleration that
    // goes at least as far as `v + a·t/2` (`v_end`), the speed reached by ramping the acceleration
    // down right away: a shorter run overshoots its end and comes back, which no single ramp
    // represents (the conservative `v_pin_eff` model and the blend absorb it). `j` is the effective
    // jerk of the run. Returns `(v_virt, displacement back)`.
    // `pin_virtual` returns `(v_virt, displacement back, v + a·t/2)`.
    let pin_virtual = |j: Real| -> Option<(Real, Real, Real)> {
        if a_pin.is_zero() || !(j > math::ZERO) {
            return None;
        }
        let t = a_pin.abs() / j;
        let half_dv = a_pin * t * HALF;
        let v_virt = v_pin - half_dv;
        Some((v_virt, t * (v_virt + a_pin * t * SIXTH), v_pin + half_dv))
    };
    let pin_ramp = |j: Real, v_end: Real| -> Option<(Real, Real)> {
        let (v_virt, d_back, v_far) = pin_virtual(j)?;
        // (with the rounding margin of the sweep: a ramp planned to land exactly on `v_end`)
        let margin = v_pin * Real::from_f32(1.0e-4);
        let deep = if a_pin > math::ZERO { v_end >= v_far - margin } else { v_end <= v_far + margin };
        deep.then_some((v_virt, d_back))
    };

    // ── Backward pass ──────────────────────────────────────────────────────────
    {
        let (mut anchor, mut dist) = (v_end, math::ZERO);
        let (mut a, mut j) = (Real::from_f32(f32::MAX), Real::from_f32(f32::MAX));
        let mut hint = None;
        for k in (1..w).rev() {
            dist += s[k];
            a = a.min(lim[k].0);
            j = j.min(lim[k].1);
            let reach = ramp_peak_hinted(anchor, dist, a, j, &mut hint);
            if cap[k] <= reach {
                v[k] = cap[k];
                anchor = cap[k];
                dist = math::ZERO;
                a = Real::from_f32(f32::MAX);
                j = Real::from_f32(f32::MAX);
            } else {
                v[k] = reach;
            }
        }
    }

    // ── Forward pass ───────────────────────────────────────────────────────────
    // A pinned entry accelerating is a ramp in progress: its reach is that of the ramp it belongs
    // to (see `pin_ramp`), and never below the speeds the previous plan sampled along it (the
    // reach is a zero-acceleration peak: right after the pin it is even below the pinned speed,
    // which made a spurious minimum). A braking one is bounded by a fresh ramp, the sweep refines it
    {
        let (mut anchor, mut dist) = (v_pin, math::ZERO);
        let (mut a, mut j) = (Real::from_f32(f32::MAX), Real::from_f32(f32::MAX));
        let mut from_pin = a_pin > math::ZERO;
        let mut hint = None;
        for k in 1..w {
            dist += s[k - 1];
            a = a.min(lim[k - 1].0);
            j = j.min(lim[k - 1].1);
            let reach = match from_pin.then(|| pin_virtual(j)).flatten() {
                Some((v_virt, d_back, _)) => {
                    ramp_peak_hinted(v_virt, dist + d_back, a, j, &mut hint).max(v_prev[k])
                }
                None => ramp_peak_hinted(anchor, dist, a, j, &mut hint),
            };
            if v[k] <= reach {
                from_pin = false;
                anchor = v[k];
                dist = math::ZERO;
                a = Real::from_f32(f32::MAX);
                j = Real::from_f32(f32::MAX);
            } else {
                v[k] = reach;
            }
        }
    }

    // ── Knots ──────────────────────────────────────────────────────────────────
    // Junctions with zero acceleration: the pinned entry (with its own acceleration), the end,
    // cap-limited junctions (corners) and every velocity extremum or plateau of the passes above.
    // Between two knots the speed is monotone and the motion is a single S-Curve ramp (plus a
    // cruise at the top when the displacement is longer than the ramp).
    let mut knot = [false; N + 1];
    knot[0] = true;
    knot[w] = true;
    for k in 1..w {
        knot[k] = !(v[k] < cap[k]) || !((v[k] - v[k - 1]) * (v[k + 1] - v[k]) > math::ZERO);
    }
    // Pinned entry with a positive acceleration: the speed still rises while it is ramped down
    // to zero (time a/j). Accounted as a higher entry speed over a shorter displacement.
    let (v_pin_eff, d_pin_lost) = if a_pin > math::ZERO && lim[0].1 > math::ZERO {
        let t = a_pin / lim[0].1;
        let v_eff = v_pin + a_pin * t * HALF;
        (v_eff, v_eff * t)
    } else {
        (v_pin, math::ZERO)
    };
    // Make every run feasible: a rising run lowers its top, a falling run lowers its start unless
    // the start is fixed (pinned, or raised by this pass), then the end is raised (braking floor).
    // Lowering a start may break the run before it: a few sweeps settle it.
    let mut fixed = [false; N + 1];
    fixed[0] = true;
    fixed[w] = true;
    // Braking floors raised, and a run left infeasible at the end of the plan `(from, to)`
    let mut floors = 0u32;
    let mut stuck: Option<(Real, Real)> = None;
    // Knots made at a corner limit by a previous round
    let mut split_knot = [false; N + 1];
    let mut acc = [math::ZERO; N + 1];
    // Corner limits: the passes bound the knots only. The junctions inside a run are sampled from
    // its S-Curve ramp, which passes them faster than the zero-acceleration peaks the passes reason
    // with: a gentle corner inside a ramp could be crossed above its limit. Such a junction becomes
    // a knot at its limit and the run is planned again (the sweep lowers it when the split runs
    // need it). A few rounds settle it; the check is a comparison per junction.
    for _round in 0..3 {
        for _sweep in 0..3 {
            // A velocity-extremum knot that a previous sweep moved past its neighbor knots is no
            // longer an extremum: kept, it would force a zero acceleration inside a monotone run
            // (two ramps instead of one), making it look infeasible
            if _sweep > 0 || _round > 0 {
                let mut prev = 0usize;
                for k in 1..w {
                    if !knot[k] {
                        continue;
                    }
                    if !fixed[k] && !split_knot[k] && v[k] < cap[k] {
                        let mut next = k + 1;
                        while !knot[next] {
                            next += 1;
                        }
                        // (an accelerating pinned entry still rises to `v_pin_eff`)
                        let v_before = if prev == 0 { v_pin_eff } else { v[prev] };
                        if (v[k] - v_before) * (v[next] - v[k]) > math::ZERO {
                            knot[k] = false;
                            continue;
                        }
                    }
                    prev = k;
                }
            }
            let mut changed = false;
            let (mut ka, mut d) = (0usize, math::ZERO);
            let (mut ra, mut rj) = (Real::from_f32(f32::MAX), Real::from_f32(f32::MAX));
            for kb in 1..=w {
                d += s[kb - 1];
                ra = ra.min(lim[kb - 1].0);
                rj = rj.min(lim[kb - 1].1);
                if !knot[kb] {
                    continue;
                }
                let vb = v[kb];
                let (va, d_run) = match (ka == 0, pin_ramp(rj, vb)) {
                    (true, Some((v_virt, d_back))) => (v_virt, d + d_back),
                    (true, _) => (v_pin_eff, d - d_pin_lost),
                    _ => (v[ka], d),
                };
                // Rounding margin: a run planned exactly at the limit is recomputed from a pinned state
                // that moved along it, a few ulps short. Without it, the braking floor (not monotone
                // in the triangular regime: R(v₁, v₀) peaks at v₁ = v₀/3) jumps from 0 to ~v₀/2
                let d_ok = d_run * (math::ONE + Real::from_f32(1.0e-4));
                if vb > va {
                    if ramp_distance(va, vb, ra, rj) > d_ok && !fixed[kb] {
                        v[kb] = ramp_peak(va, d_run.max(math::ZERO), ra, rj);
                        changed = true;
                    }
                } else if va > vb && ramp_distance(vb, va, ra, rj) > d_ok {
                    if ka == 0
                        && kb == 1
                        && segment_admissible(s[0], v_pin, vb, a_pin, math::ZERO, lim[0].0, tol)
                    {
                        // A single segment from the pinned state: executed as a blend, which exists.
                        // The pinned state may be off the ramp model (the admissibility repair of the
                        // previous plan moves the entry acceleration of the segment that brakes into
                        // a stop); raising the floor would cross that stop (a corner) at speed
                    } else if !fixed[ka] {
                        v[ka] = ramp_peak(vb, d_run, ra, rj);
                        changed = true;
                    } else if kb < w {
                        floors += 1;
                        v[kb] = ramp_floor(va, d_run.max(math::ZERO), ra, rj);
                        fixed[kb] = true;
                        changed = true;
                    } else if stuck.is_none() {
                        stuck = Some((va, vb));
                    }
                }
                ka = kb;
                d = math::ZERO;
                ra = Real::from_f32(f32::MAX);
                rj = Real::from_f32(f32::MAX);
            }
            if !changed {
                break;
            }
        }

        // ── Sampling: speeds and accelerations of the junctions inside each run ───────────
        acc = [math::ZERO; N + 1];
        acc[0] = a_pin;
        {
            let mut ka = 0usize;
            while ka < w {
                let mut kb = ka + 1;
                while !knot[kb] {
                    kb += 1;
                }
                if kb > ka + 1 {
                    let (mut va, vb) = (v[ka], v[kb]);
                    let (mut ra, mut rj, mut d) =
                        (Real::from_f32(f32::MAX), Real::from_f32(f32::MAX), math::ZERO);
                    for k in ka..kb {
                        ra = ra.min(lim[k].0);
                        rj = rj.min(lim[k].1);
                        d += s[k];
                    }
                    // A ramp in progress at the pinned entry: sample the ramp it belongs to, which
                    // starts `d_back` before the pinned junction (see `pin_ramp`)
                    let mut x0 = math::ZERO;
                    if ka == 0 {
                        if let Some((v_virt, d_back)) = pin_ramp(rj, vb) {
                            va = v_virt;
                            d += d_back;
                            x0 = d_back;
                        }
                    }
                    let (lo, hi) = if vb > va { (va, vb) } else { (vb, va) };
                    let r = ramp_distance(lo, hi, ra, rj);
                    // A run the sweeps could not make feasible is compressed (never expected)
                    let scale = if r > d && d > math::ZERO { r / d } else { math::ONE };
                    let slack = (d - r).max(math::ZERO);
                    // Corner limits inside the run: the ramp is placed as early (rising) or as
                    // late (falling) as possible, the time-optimal choice, and passes the
                    // junctions at its highest speeds. Shift it by the least `e` of the slack
                    // (a rising run cruises at the bottom first, a falling one brakes earlier and
                    // cruises at the bottom last) so that it passes every junction within its
                    // limit. Only a limit below the bottom speed, or one the slack can not
                    // absorb (no shift then), splits the run (see below). Not for a ramp in
                    // progress at the pin.
                    let mut e = math::ZERO;
                    if x0.is_zero() && slack > math::ZERO && va != vb {
                        let mut x = math::ZERO;
                        for k in (ka + 1)..kb {
                            x += s[k - 1];
                            if cap[k] < hi && !(cap[k] < lo) {
                                let xc = ramp_position(lo, hi, cap[k], ra, rj);
                                // Ramp coordinate of the junction must stay below `xc`
                                let need = if vb > va { x - xc } else { r + slack - x - xc };
                                e = e.max(need);
                            }
                        }
                        // All or nothing: a partial shift still splits the run, and the split
                        // (a zero-acceleration knot at the limit) then sits on a ramp displaced
                        // from where the braking floors and the next chainings expect it
                        if e > slack {
                            e = math::ZERO;
                        }
                    }
                    let mut x = x0;
                    let mut hint = None;
                    for k in (ka + 1)..kb {
                        x += s[k - 1];
                        let xs = x * scale;
                        let (vk, ak) = if vb > va {
                            // (cruise at the bottom for `e`), ramp, then cruise at the top
                            if !(xs > e) {
                                (lo, math::ZERO)
                            } else {
                                ramp_state_hinted(lo, hi, xs - e, ra, rj, &mut hint)
                            }
                        } else if va > vb && x0 > math::ZERO {
                            // A brake in progress continues: brake first, then cruise at the bottom
                            if xs < r {
                                let (vr, ar) = ramp_state_hinted(lo, hi, r - xs, ra, rj, &mut hint);
                                (vr, -ar)
                            } else {
                                (lo, math::ZERO)
                            }
                        } else if va > vb {
                            // Cruise at the top first, then brake (time reversal of the rising
                            // ramp), (then cruise at the bottom for `e`)
                            let b0 = slack - e;
                            if !(xs > b0) {
                                (hi, math::ZERO)
                            } else if xs < b0 + r {
                                let (vr, ar) = ramp_state_hinted(lo, hi, r - (xs - b0), ra, rj, &mut hint);
                                (vr, -ar)
                            } else {
                                (lo, math::ZERO)
                            }
                        } else {
                            (va, math::ZERO)
                        };
                        v[k] = vk;
                        acc[k] = ak.clamp(-ra, ra);
                    }
                }
                ka = kb;
            }
        }


        let mut split = false;
        for k in 1..w {
            if !knot[k] && v[k] > cap[k] {
                knot[k] = true;
                split_knot[k] = true;
                v[k] = cap[k];
                split = true;
            }
        }
        if !split {
            break;
        }
    }

    let keep_prev_until = (1..w).find(|&k| prev_keep[k].is_some());
    if fallback && (floors > 0 || stuck.is_some()) {
        if let Some(m) = keep_prev_until {
            return keep_previous_plan(rb, w, m, prev_keep[m]);
        }
    }
    for _ in 0..floors {
        count(5);
    }
    if let Some((_va, _vb)) = stuck {
        count(6);
        hwa::warn!(
            "[motion_chaining] Can not brake from {:?} to the end of the plan ({:?})",
            _va,
            _vb
        );
    }

    // ── Blend Admissibility Pass ───────────────────────────────────────────────
    // Every segment with non-zero boundary accelerations is executed as a polynomial blend:
    // guarantee here (with the predicate the profile uses) that it exists and never reverses its
    // velocity for any displacement the stepper may execute, so that the profile never needs to
    // alter the boundary conditions (acceleration discontinuity). Repair: shrink the free boundary
    // accelerations of every non-admissible segment towards zero (zero boundary accelerations are
    // always admissible), writing the same value on both sides of each junction.
    let mut plan_admissible = false;
    // Segments to (re)check: those whose boundary accelerations changed since they were checked
    // (the others would get the same outcome: admissible or left as is). Same plan as checking
    // them all every round, a fraction of the cost (each check is ~50 soft-float operations)
    let mut need = [true; N];
    for _round in 0..4 {
        let mut all_ok = true;
        for k in 0..w {
            if !need[k] {
                continue;
            }
            need[k] = false;
            let (v_i, v_o, a_i, a_o) = (v[k], v[k + 1], acc[k], acc[k + 1]);
            if (a_i.is_zero() && a_o.is_zero())
                || segment_admissible(s[k], v_i, v_o, a_i, a_o, lim[k].0, tol)
            {
                continue;
            }
            // A junction at rest is a velocity minimum: its acceleration stays zero (a negative
            // one would reverse the next segment; the profile ignores it, breaking continuity)
            let entry_free = k > 0 && v[k] > rest_tol[k];
            let exit_free = k + 1 < w && v[k + 1] > rest_tol[k + 1];
            // Segments dominated by the step quantization go straight to zero accelerations
            let quantized = !(s[k] > tol * math::TWO);
            let a_o_new = if exit_free {
                let cap = admissible_exit_accel_cap(s[k], v_i, v_o, a_i, tol);
                Some(if quantized {
                    math::ZERO.min(cap)
                } else if a_o > cap {
                    cap
                } else {
                    a_o * HALF
                })
            } else {
                None
            };
            // A pinned segment whose exit acceleration is already zero is left as is: moving that
            // exit below the feasibility cap would push a non-zero (decelerating) acceleration
            // into the next segment. The stepper caps the pinned entry acceleration instead
            if !entry_free && (a_o.is_zero() || a_o_new.is_none_or(|a| a == a_o)) {
                continue;
            }
            all_ok = false;
            // Changed or not, this segment is checked again next round (as all of them were)
            need[k] = true;
            if let Some(a) = a_o_new {
                acc[k + 1] = a;
                if k + 1 < w {
                    need[k + 1] = true;
                }
            }
            if entry_free {
                // (k > 0)
                need[k - 1] = true;
                acc[k] = if quantized {
                    math::ZERO
                } else if !exit_free && a_i < a_o {
                    // Fixed exit: raise the entry 5% inside the blend feasibility boundary
                    // (a_i = a_o - 5·(v_i+v_o)²/(8·s), the mirror of `blend_accel_exit_cap`)
                    let floor = admissible_entry_accel_floor(s[k], v_i, v_o, a_o, tol);
                    if a_i < floor { floor } else { a_i * HALF }
                } else {
                    a_i * HALF
                };
            }
        }
        if all_ok {
            plan_admissible = true;
            break;
        }
    }
    if !plan_admissible {
        #[cfg(feature = "debug-motion-planning")]
        hwa::warn!("[motion_chaining] Falling back to zero junction accelerations");
        for k in 1..w {
            acc[k] = math::ZERO;
        }
    }

    // The pinned segment keeps its entry state, and its exit is left as is when it already has
    // zero acceleration (see above): when its blend is still not admissible (its state is off the
    // ramp model, which found the run feasible), the profile would alter its exit acceleration.
    // The previous plan executes it: keep it (see the fallback in [chain])
    if fallback && w > 1 {
        let a_o = acc[1];
        if !(acc[0].is_zero() && a_o.is_zero())
            && !segment_admissible(s[0], v[0], v[1], acc[0], a_o, lim[0].0, tol)
        {
            if let Some(m) = keep_prev_until {
                return keep_previous_plan(rb, w, m, prev_keep[m]);
            }
        }
    }

    // ── Commit the speeds ──────────────────────────────────────────────────────
    for k in 0..w {
        let seg = rb.mut_planned_segment_from_tail((w - k) as u8)?;
        if k > 0 {
            seg.speed_enter_su_s = v[k];
        }
        seg.speed_exit_su_s = v[k + 1];
        // Informative only (logs)
        seg.speed_enter_constrained_su_s = v[k];
        seg.speed_exit_constrained_su_s = v[k + 1];
    }

    // ── Commit the accelerations ───────────────────────────────────────────────
    for k in 0..w {
        let seg = rb.mut_planned_segment_from_tail((w - k) as u8)?;
        if k > 0 {
            seg.accel_enter_su_s = acc[k];
        }
        seg.accel_exit_su_s = if k + 1 == w { math::ZERO } else { acc[k + 1] };
    }

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!("[motion_chaining] END");
    Ok(None)
}

#[cfg(all(test, feature = "debug-motion-planning"))]
pub fn display_content(
    rb: &embassy_sync::mutex::MutexGuard<
        <hwa::types::MotionRingBufferMutexStrategy as hwa::AsyncMutexStrategy>::AsyncMutexType,
        RingBuffer,
    >,
    left_offset: u8,
    right_offset: u8,
) -> Result<(), ()> {
    let mut stb = Vec::new();
    for i in 0..left_offset - right_offset + 1 {
        let s = rb.planned_segment_from_tail(left_offset - i)?;
        stb.push(format!(
            "[{:?},{:?}]",
            s.speed_enter_su_s, s.speed_exit_su_s
        ))
    }
    hwa::info!(": {:?}", stb.join(" "));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hwa::math::Real;
    use hwa::controllers::Segment;
    use hwa::controllers::PlanEntry;
    use hwa::controllers::motion_control::motion_ring_buffer::RingBuffer;
    use embassy_sync::mutex::Mutex;

    /// The throughput slowdown matches the production period at half queue, slows down more below
    /// it, less above it, never when full, and never affects long moves
    #[test]
    fn test_throughput_speed_cap() {
        use super::throughput_speed_cap as cap;
        let r = Real::from_f32;
        let (s, v, t) = (r(0.55), r(116.7), r(0.011));
        let near = |a: Real, b: f64| (a.to_f64() - b).abs() < 1e-3 * b.max(1.0);
        // Half queue: one move per production period
        assert!(near(cap(s, v, 20, 40, t), 0.55 / 0.011));
        // Empty queue: 1.5 periods per move; 3/4: 0.75
        assert!(near(cap(s, v, 0, 40, t), 0.55 / 0.0165));
        assert!(near(cap(s, v, 30, 40, t), 0.55 / 0.00825));
        // Full queue, unknown period, long move: untouched
        assert_eq!(cap(s, v, 40, 40, t), v);
        assert_eq!(cap(s, v, 10, 40, math::ZERO), v);
        assert_eq!(cap(r(50.0), v, 10, 40, t), v);
        // Never raises the target
        assert_eq!(cap(s, r(10.0), 10, 40, t), r(10.0));
    }

    /// The corner limit keeps the velocity jump `2·v·sin(θ/2)` within `dv`, and never raises the
    /// projection limit
    #[test]
    fn test_corner_speed_factor() {
        let (v_ref, dv) = (Real::from_f32(117.0), Real::from_f32(5.0));
        for deg in [0.0f32, 3.0, 10.0, 45.0, 60.0, 89.0, 90.0, 135.0, 180.0] {
            let c = Real::from_f32(deg.to_radians().cos());
            let f = corner_speed_factor(c, v_ref, dv);
            if !JOINT_SPACE_CORNERS {
                assert!(f <= c.max(math::ZERO), "{}°: {:?} above the projection", deg, f);
            }
            assert!(!(f < math::ZERO), "{}°: negative", deg);
            let jump = math::TWO * f * v_ref * Real::from_f32((deg.to_radians() / 2.0).sin());
            assert!(jump <= dv * Real::from_f32(1.0001), "{}°: jump {:?}", deg, jump);
        }
        // Collinear: unlimited. Gentle (3°): 5 mm/s allows ~95 mm/s. 60°: 5 mm/s
        assert_eq!(corner_speed_factor(math::ONE, v_ref, dv), math::ONE);
        let v3 = corner_speed_factor(Real::from_f32(3.0f32.to_radians().cos()), v_ref, dv) * v_ref;
        assert!((v3 - Real::from_f32(95.5)).abs() < math::ONE, "{:?}", v3);
        let v60 = corner_speed_factor(math::HALF, v_ref, dv) * v_ref;
        assert!((v60 - dv).abs() < Real::from_f32(0.01), "{:?}", v60);
        // Disabled: the projection alone
        assert_eq!(corner_speed_factor(math::HALF, v_ref, math::ZERO), math::HALF);
        // A reversal (135°): at rest for a Cartesian machine (projection), crossed at the speed
        // whose velocity jump is `dv` for a joint space one
        let reversal = Real::from_f32(135.0f32.to_radians().cos());
        let v135 = corner_speed_factor(reversal, v_ref, dv) * v_ref;
        if JOINT_SPACE_CORNERS {
            let jump = math::TWO * v135 * Real::from_f32((135.0f32.to_radians() / 2.0).sin());
            assert!((jump - dv).abs() < Real::from_f32(0.01), "135°: jump {:?}", jump);
        } else {
            assert_eq!(v135, math::ZERO);
        }
    }

    fn create_test_segment(
        displacement: f32,
        v_max: f32,
        a_max: f32,
        j_max: f32,
    ) -> Segment {
        Segment {
            displacement_su: Real::from_f32(displacement),
            speed_enter_su_s: Real::from_f32(0.0),
            speed_exit_su_s: Real::from_f32(0.0),
            speed_enter_constrained_su_s: Real::from_f32(0.0),
            speed_exit_constrained_su_s: Real::from_f32(0.0),
            accel_enter_su_s: Real::from_f32(0.0),
            accel_exit_su_s: Real::from_f32(0.0),
            speed_rest_su_s: Real::from_f32(0.0),
            proj_prev: Real::from_f32(1.0),
            proj_next: Real::from_f32(1.0),
            unit_vector_dir: Default::default(),
            src_pos: Default::default(),
            dest_pos: Default::default(),
            dest_world_pos: Default::default(),
            tool_power: Default::default(),
            speed_programmed_su_s: Default::default(),
            constraints: crate::motion::Constraints {
                v_max: Real::from_f32(v_max),
                a_max: Real::from_f32(a_max),
                j_max: Real::from_f32(j_max),
            },
            speed_target_su_s: Default::default()
        }
    }

    /// The fast Newton path of `jerk_ramp_time` returns the root of the jerk-ramp equation
    /// (checked through its residual) within `(0, t_j]` and agrees with the general cubic solver.
    #[test]
    fn test_jerk_ramp_time_matches_cubic_solver() {
        let mut seed: u64 = 0x1234_5678_9ABC_DEF1;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / (1u64 << 24) as f32
        };
        for _ in 0..20000 {
            let j = Real::from_f32(1000.0 + 100000.0 * rnd());
            let a_i = Real::from_f32(3000.0 * rnd() * rnd());
            let v_i = Real::from_f32(if rnd() < 0.1 { 0.0 } else { 200.0 * rnd() });
            let t_j = Real::from_f32(0.001 + 0.2 * rnd());
            let s_j = v_i * t_j + HALF * a_i * t_j * t_j + SIXTH * j * t_j * t_j * t_j;
            let s = s_j * Real::from_f32(0.001 + 0.998 * rnd());
            let t = jerk_ramp_time(j, a_i, v_i, s, t_j);
            assert!(t > math::ZERO && t <= t_j);
            let residual = ((SIXTH * j * t + HALF * a_i) * t + v_i) * t - s;
            assert!(
                (residual.abs() / s).to_f64() < 1.0e-5,
                "residual {:?} for t={:?} (j={:?} a={:?} v={:?} s={:?})",
                residual, t, j, a_i, v_i, s
            );
            let reference = math::solve_cubic(j * SIXTH, a_i * HALF, v_i, -s)
                .filter(|t| *t <= t_j)
                .unwrap_or(t_j);
            assert!(((t - reference).abs() / reference).to_f64() < 2.0e-5, "t={:?} vs {:?}", t, reference);
        }
    }

    #[test]
    fn test_plan_velocity_profile_short_segment() {
        // A segment too short to reach max acceleration
        let mut segment = create_test_segment(10.0, 100.0, 50.0, 200.0);
        compute_maximum_enter_and_exit_speed(true, 0, &mut segment);
        // Check that the exit speed is less than v_max
        assert!(segment.speed_exit_constrained_su_s < Real::from_f32(100.0));
        // With v_exit = 0, backward pass should result in a constrained entry speed < v_max
        assert!(segment.speed_enter_constrained_su_s < Real::from_f32(100.0));
    }

    #[test]
    fn test_plan_velocity_profile_velocity_limited() {
        // A segment long enough to be velocity-limited
        let mut segment = create_test_segment(1000.0, 100.0, 50.0, 200.0);
        compute_maximum_enter_and_exit_speed(true, 0, &mut segment);
        // Check that the exit speed is equal to v_max
        assert_eq!(segment.speed_exit_constrained_su_s, Real::from_f32(100.0));
    }

    #[test]
    fn test_plan_velocity_profile_forward_backward() {
        let mut segment = create_test_segment(100.0, 100.0, 50.0, 200.0);
        segment.speed_enter_su_s = Real::from_f32(10.0);
        segment.speed_exit_su_s = Real::from_f32(20.0);

        compute_maximum_enter_and_exit_speed(false, 0, &mut segment);

        // With v_enter = 10, forward pass should give some max exit speed.
        assert!(segment.speed_exit_constrained_su_s > segment.speed_enter_su_s);

        // With v_exit = 20, backward pass should give some max entry speed.
        assert!(segment.speed_enter_constrained_su_s > segment.speed_exit_su_s);
    }

    #[test]
    fn test_plan_velocity_profile_half_scurve() {
        // Test a scenario where the segment is not long enough to perform a full
        // trapezoidal acceleration profile, but is velocity-limited.
        let v_max = 20.0;
        let a_max = 50.0;
        let j_max = 200.0;
        let v_enter = 10.0;
        // 6 units: enough to reach v_max with the effective (smooth-jerk, ⅔·j_max) jerk.
        // (5 units only reach ≈19.58 because the planner now uses the same jerk as the profile.)
        let displacement = 6.0;

        let mut segment = create_test_segment(displacement, v_max, a_max, j_max);
        segment.speed_enter_su_s = Real::from_f32(v_enter);
        compute_maximum_enter_and_exit_speed(false, 0, &mut segment);

        // The calculated maximum exit speed should be capped at the requested v_max.
        assert_eq!(segment.speed_exit_constrained_su_s, Real::from_f32(v_max));
    }

    /// Verify that `compute_maximum_enter_and_exit_speed` clamps the resulting accelerations
    /// so that the quintic time-estimation discriminant is non-negative:
    ///   delta = (v_i + v_o)^2 / 4 + (a_i - a_o) * s_o / 3 >= 0
    ///
    /// Previously, large ±a_max accelerations produced by the independent forward/backward
    /// passes would cause delta < 0 for short decelerating segments, triggering:
    ///   WARN: Unable to perform movement of length 6.9999995: delta(-6119.25) < 0
    #[test]
    fn test_accel_clamping_prevents_negative_discriminant() {
        let a_max = 1302.75_f32;
        let j_max = a_max * 50.0;
        let mut segment = create_test_segment(7.0, 50.0, a_max, j_max);
        segment.speed_enter_su_s = Real::from_f32(14.282858);
        segment.speed_exit_su_s = Real::from_f32(0.0);
        // Simulate the worst-case prior state: ±a_max from a previous chaining pass
        segment.accel_enter_su_s = Real::from_f32(-a_max);
        segment.accel_exit_su_s = Real::from_f32(a_max);

        compute_maximum_enter_and_exit_speed(false, 0, &mut segment);

        // delta = (v_i + v_o)^2 / 4 + (a_i - a_o) * s_o / 3  must be >= 0
        let v_sum = segment.speed_enter_su_s + segment.speed_exit_su_s;
        let a_i = segment.accel_enter_su_s;
        let a_o = segment.accel_exit_su_s;
        let s = segment.displacement_su;
        let delta = v_sum * v_sum * Real::from_f32(0.25)
            + (a_i - a_o) * s * Real::from_f32(0.4);
        assert!(
            delta >= Real::from_f32(-1e-4),
            "delta {:?} must be >= 0; accelerations were not properly clamped",
            delta
        );
    }

    /// Same scenario directly against the exact values from the error log to confirm
    /// that `HybridMotionProfile::compute` now succeeds end-to-end.
    #[test]
    #[cfg(feature = "with-motion-chaining")]
    fn test_accel_clamping_allows_profile_compute() {
        use crate::motion::HybridMotionProfile;

        let a_max = 1302.75_f32;
        let j_max = a_max * 50.0;
        let mut segment = create_test_segment(7.0, 50.0, a_max, j_max);
        segment.speed_enter_su_s = Real::from_f32(14.282858);
        segment.speed_exit_su_s = Real::from_f32(0.0);
        segment.accel_enter_su_s = Real::from_f32(-a_max);
        segment.accel_exit_su_s = Real::from_f32(a_max);

        compute_maximum_enter_and_exit_speed(false, 0, &mut segment);

        let result = HybridMotionProfile::compute(
            segment.displacement_su,
            segment.speed_enter_su_s,
            segment.speed_exit_su_s,
            segment.accel_enter_su_s,
            segment.accel_exit_su_s,
            &segment.constraints,
            false,
        );
        assert!(
            result.is_ok(),
            "HybridMotionProfile::compute should succeed after clamping, got {:?}",
            result.err()
        );
    }

    #[futures_test::test]
    async fn test_perform_motion_chaining_3_segments_corner() {
        let mut rb = RingBuffer::new();

        // Segments ordered from oldest to newest in the path: seg1 -> seg2 -> seg3
        // RingBuffer access is 1-based from tail:
        // tail(1) = newest = seg3
        // tail(2) = seg2
        // tail(3) = oldest = seg1

        // Segment 1: Moving in X. Start at 0.
        let mut seg1 = create_test_segment(100.0, 50.0, 1000.0, 5000.0);
        seg1.unit_vector_dir.x = Some(Real::from_f32(1.0));
        seg1.proj_next = Real::from_f32(0.0); // 90 degree corner to seg2
        seg1.speed_enter_su_s = Real::from_f32(0.0);
        seg1.speed_exit_su_s = Real::from_f32(50.0); // Initial assumption

        // Segment 2: Moving in Y.
        let mut seg2 = create_test_segment(100.0, 50.0, 1000.0, 5000.0);
        seg2.unit_vector_dir.y = Some(Real::from_f32(1.0));
        seg2.proj_prev = Real::from_f32(0.0); // 90 degree corner from seg1
        seg2.proj_next = Real::from_f32(1.0); // Straight to seg3
        seg2.speed_enter_su_s = Real::from_f32(50.0);
        seg2.speed_exit_su_s = Real::from_f32(50.0);

        // Segment 3: Moving in Y. End at 0.
        let mut seg3 = create_test_segment(100.0, 50.0, 1000.0, 5000.0);
        seg3.unit_vector_dir.y = Some(Real::from_f32(1.0));
        seg3.proj_prev = Real::from_f32(1.0); // Straight from seg2
        seg3.speed_enter_su_s = Real::from_f32(50.0);
        seg3.speed_exit_su_s = Real::from_f32(0.0);

        // Pre-calculate constraints for all segments
        compute_maximum_enter_and_exit_speed(true, 2, &mut seg1);
        compute_maximum_enter_and_exit_speed(false, 1, &mut seg2);
        compute_maximum_enter_and_exit_speed(false, 0, &mut seg3);

        // Setup RingBuffer: head=0, used=3.
        // data[0] = oldest = seg1
        // data[1] = seg2
        // data[2] = newest = seg3
        rb.data[0] = PlanEntry::PlannedMove(seg1, Default::default(), Default::default(), false, 0);
        rb.data[1] = PlanEntry::PlannedMove(seg2, Default::default(), Default::default(), false, 1);
        rb.data[2] = PlanEntry::PlannedMove(seg3, Default::default(), Default::default(), false, 2);
        rb.used = 3;
        rb.head = 0;

        let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;

        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        // Junction is between seg1 (older, tail(3)) and seg2 (newer, tail(2)).
        let final_seg1 = guard.planned_segment_from_tail(3).unwrap();
        let final_seg2 = guard.planned_segment_from_tail(2).unwrap();

        // The junction between seg1 and seg2 is a 90-degree corner.
        // The speed at this junction should be forced to be very low (ideally 0).
        // This means seg1.exit_speed and seg2.enter_speed should be ~0.
        assert!(final_seg1.speed_exit_su_s < Real::from_f32(1.0));
        assert!(final_seg2.speed_enter_su_s < Real::from_f32(1.0));
    }

    // ── Helper to evaluate the polynomial-blend discriminant for a segment ───
    //   Δ = (v_i + v_o)²/4 + 0.4·(a_i − a_o)·s
    fn quintic_delta(seg: &Segment) -> Real {
        let v_sum = seg.speed_enter_su_s + seg.speed_exit_su_s;
        let a_diff = seg.accel_enter_su_s - seg.accel_exit_su_s;
        v_sum * v_sum * Real::from_f32(0.25) + a_diff * seg.displacement_su * Real::from_f32(0.4)
    }

    /// Build a ring-buffer with `n` collinear segments (all moving in the +X
    /// direction) of the given length, speed, a_max and j_max, push them into a
    /// Mutex and return the locked guard.
    async fn make_collinear_rb(
        n: u8,
        displacement: f32,
        v_max: f32,
        a_max: f32,
        j_max: f32,
    ) -> embassy_sync::mutex::Mutex<hwa::AsyncNoopMutexType, RingBuffer> {
        let mut rb = RingBuffer::new();
        for k in 0..n as usize {
            let mut seg = create_test_segment(displacement, v_max, a_max, j_max);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.proj_prev = Real::from_f32(1.0);
            seg.proj_next = Real::from_f32(1.0);
            seg.speed_target_su_s = Real::from_f32(v_max);
            // First segment starts from rest; last segment must stop.
            seg.speed_enter_su_s = Real::from_f32(0.0);
            seg.speed_exit_su_s = if k == n as usize - 1 {
                Real::from_f32(0.0)
            } else {
                Real::from_f32(v_max)
            };
            rb.data[k] = PlanEntry::PlannedMove(
                seg,
                Default::default(),
                Default::default(),
                false,
                k as u32,
            );
        }
        rb.used = n;
        rb.head = 0;
        embassy_sync::mutex::Mutex::<hwa::AsyncNoopMutexType, _>::new(rb)
    }

    /// After chaining, every adjacent pair must satisfy
    ///   speed_exit[older] == speed_enter[newer]
    #[futures_test::test]
    async fn test_velocity_continuity_after_chaining_collinear() {
        const N: u8 = 5;
        let mutex = make_collinear_rb(N, 50.0, 50.0, 500.0, 5000.0).await;
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        for i in 1..N {
            let v_exit = guard.planned_segment_from_tail(i + 1).unwrap().speed_exit_su_s;
            let v_enter = guard.planned_segment_from_tail(i).unwrap().speed_enter_su_s;
            let diff = (v_exit - v_enter).abs();
            assert!(
                diff < Real::from_f32(1e-4),
                "Velocity discontinuity at junction {}: v_exit={:?} v_enter={:?} diff={:?}",
                i, v_exit, v_enter, diff
            );
        }
    }

    /// Tool actions between moves (`M106`, `M3`...) are attached to the entries
    /// (`RingBuffer::post_tools`) instead of taking slots: the chaining plans exactly the same
    /// junctions with or without them (the moves around them do not stop), and leaves them as they
    /// were
    #[futures_test::test]
    async fn test_tool_actions_do_not_break_the_chaining() {
        use hwa::controllers::{CutterState, ToolActions};
        const N: u8 = 4;
        let plain = make_collinear_rb(N, 10.0, 50.0, 500.0, 5000.0).await;
        let with_tools = make_collinear_rb(N, 10.0, 50.0, 500.0, 5000.0).await;
        let laser_on = ToolActions { cutter: Some(CutterState::Clockwise(100)), ..ToolActions::NONE };
        let laser_off_fan_on = ToolActions {
            cutter: Some(CutterState::Off),
            fan_layer: Some(100),
            ..ToolActions::NONE
        };
        {
            let mut g = with_tools.lock().await;
            g.post_tools[0] = laser_on;
            g.post_tools[2] = laser_off_fan_on;
        }
        let mut a = plain.lock().await;
        perform_motion_chaining(&mut a, Real::zero()).unwrap();
        let mut b = with_tools.lock().await;
        perform_motion_chaining(&mut b, Real::zero()).unwrap();
        for i in 1..=N {
            let (sa, sb) = (a.planned_segment_from_tail(i).unwrap(), b.planned_segment_from_tail(i).unwrap());
            assert_eq!(sa.speed_enter_su_s, sb.speed_enter_su_s, "entry speed of segment {}", N - i);
            assert_eq!(sa.speed_exit_su_s, sb.speed_exit_su_s, "exit speed of segment {}", N - i);
            assert_eq!(sa.accel_enter_su_s, sb.accel_enter_su_s, "entry acceleration of segment {}", N - i);
            assert_eq!(sa.accel_exit_su_s, sb.accel_exit_su_s, "exit acceleration of segment {}", N - i);
        }
        // The junctions are chained (no stop where the tool actions are)
        for k in 0..(N - 1) as usize {
            let exit = b.planned_segment_from_tail(N - k as u8).unwrap().speed_exit_su_s;
            assert!(exit > Real::from_f32(1.0), "junction {} stopped: {:?}", k, exit);
        }
        assert_eq!(b.post_tools[0], laser_on);
        assert_eq!(b.post_tools[1], ToolActions::NONE);
        assert_eq!(b.post_tools[2], laser_off_fan_on);
    }

    /// After chaining, every adjacent pair must satisfy
    ///   accel_exit[older] == accel_enter[newer]
    #[futures_test::test]
    async fn test_acceleration_continuity_after_chaining_collinear() {
        const N: u8 = 5;
        let mutex = make_collinear_rb(N, 50.0, 50.0, 500.0, 5000.0).await;
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        for i in 1..N {
            let a_exit = guard.planned_segment_from_tail(i + 1).unwrap().accel_exit_su_s;
            let a_enter = guard.planned_segment_from_tail(i).unwrap().accel_enter_su_s;
            let diff = (a_exit - a_enter).abs();
            assert!(
                diff < Real::from_f32(1e-4),
                "Acceleration discontinuity at junction {}: a_exit={:?} a_enter={:?} diff={:?}",
                i, a_exit, a_enter, diff
            );
        }
    }

    /// After chaining, the Quintic discriminant must be >= 0 for every segment.
    /// This exercises the Feasibility Pass added to `perform_motion_chaining`.
    #[futures_test::test]
    async fn test_no_negative_discriminant_after_chaining_collinear() {
        const N: u8 = 5;
        let mutex = make_collinear_rb(N, 50.0, 50.0, 500.0, 5000.0).await;
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        for i in 1..=N {
            let seg = guard.planned_segment_from_tail(i).unwrap();
            let delta = quintic_delta(seg);
            assert!(
                delta >= Real::from_f32(-1e-4),
                "Negative discriminant for segment {}: delta={:?}  a_i={:?}  a_o={:?}  v_i={:?}  v_o={:?}  s={:?}",
                i,
                delta,
                seg.accel_enter_su_s,
                seg.accel_exit_su_s,
                seg.speed_enter_su_s,
                seg.speed_exit_su_s,
                seg.displacement_su,
            );
        }
    }

    /// Short segments + high a_max is the worst case for the feasibility pass:
    /// the kinematic forward pass gives a large a_exit that would make delta < 0.
    /// After chaining, all discriminants must be >= 0 and a_o[n] == a_i[n+1].
    #[futures_test::test]
    async fn test_feasibility_pass_short_segments_high_accel() {
        // Tight parameters similar to the real firmware (native, 16 µstep/mm)
        const N: u8 = 6;
        let mutex = make_collinear_rb(N, 10.0, 100.0, 1500.0, 75000.0).await;
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        for i in 1..=N {
            let seg = guard.planned_segment_from_tail(i).unwrap();
            let delta = quintic_delta(seg);
            assert!(
                delta >= Real::from_f32(-1e-4),
                "Negative discriminant for segment {} (short/high-accel): delta={:?}",
                i, delta
            );
        }

        // Acceleration continuity
        for i in 1..N {
            let a_exit = guard.planned_segment_from_tail(i + 1).unwrap().accel_exit_su_s;
            let a_enter = guard.planned_segment_from_tail(i).unwrap().accel_enter_su_s;
            assert!(
                (a_exit - a_enter).abs() < Real::from_f32(1e-4),
                "Acceleration discontinuity at junction {} (short/high-accel): a_exit={:?} a_enter={:?}",
                i, a_exit, a_enter
            );
        }
    }

    /// A junction at rest (full-stop corner) is a velocity minimum: its acceleration must stay
    /// zero. The admissibility repair used to move the exit acceleration of the segment that
    /// brakes into the stop to its (negative) feasibility cap, leaving the next segment with
    /// `v_i = 0, a_i < 0` ("Ignoring boundary accelerations ... of a rest-to-rest segment").
    #[futures_test::test]
    async fn test_rest_junctions_keep_zero_acceleration() {
        let mut seed: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / (1u64 << 24) as f32
        };
        let (v_max, a_max, j_max) = (150.0f32, 3000.0f32, 200000.0f32);
        let mut violations = 0;
        for case in 0..400 {
            let mut rb = RingBuffer::new();
            let n = 4 + (rnd() * 12.0) as usize;
            for k in 0..n {
                let len = if rnd() < 0.5 { 0.05 + 0.5 * rnd() } else { 0.5 + 10.0 * rnd() };
                let mut seg = create_test_segment(len, v_max, a_max, j_max);
                seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
                // Some junctions are full-stop corners
                let corner = k > 0 && rnd() < 0.3;
                seg.proj_prev = if k == 0 || corner { Real::from_f32(0.0) } else { Real::from_f32(1.0) };
                seg.speed_target_su_s = Real::from_f32(v_max * (0.3 + 0.7 * rnd()));
                rb.data[k] = PlanEntry::PlannedMove(seg, Default::default(), Default::default(), false, k as u32);
            }
            rb.used = n as u8;
            rb.head = 0;
            let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
            let mut guard = mutex.lock().await;
            let tol = if case % 2 == 0 { Real::zero() } else { Real::from_f32(0.0125) };
            perform_motion_chaining(&mut guard, tol).unwrap();
            for i in 1..=n as u8 {
                let seg = guard.planned_segment_from_tail(i).unwrap();
                let rest_tol = seg.constraints.v_max * Real::from_f32(1.0e-5);
                let at_rest_enter = seg.speed_enter_su_s <= rest_tol;
                let at_rest_exit = seg.speed_exit_su_s <= rest_tol;
                if (at_rest_enter && !seg.accel_enter_su_s.is_zero())
                    || (at_rest_exit && !seg.accel_exit_su_s.is_zero())
                {
                    violations += 1;
                    std::println!(
                        "case {} seg tail({}): s={:?} v_i={:?} v_o={:?} a_i={:?} a_o={:?}",
                        case, i, seg.displacement_su, seg.speed_enter_su_s, seg.speed_exit_su_s,
                        seg.accel_enter_su_s, seg.accel_exit_su_s
                    );
                }
            }
        }
        assert_eq!(violations, 0, "{} rest junctions with non-zero acceleration", violations);
    }

    /// Chains `segs` (oldest first, the oldest one pinned at its entry state) behind an executing
    /// entry, as the planner does while the stepper consumes the queue, and writes the plan back.
    /// Returns the counters of braking floors raised and runs left infeasible.
    async fn chain_behind_executing(segs: &mut [Segment], windowed: bool, tol: Real) -> (u32, u32) {
        use hwa::controllers::motion_control::MovType;
        let mut rb = RingBuffer::new();
        rb.data[0] = PlanEntry::Executing(MovType::Move(Default::default(), Default::default()), false, 0);
        for (k, seg) in segs.iter().enumerate() {
            rb.data[k + 1] = PlanEntry::PlannedMove(*seg, Default::default(), Default::default(), false, k as u32 + 1);
        }
        rb.used = segs.len() as u8 + 1;
        rb.head = 0;
        let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;
        counters::reset();
        if windowed {
            perform_motion_chaining_after_append(&mut guard, tol).unwrap();
        } else {
            perform_motion_chaining(&mut guard, tol).unwrap();
        }
        let c = counters::get();
        let n = segs.len();
        for (k, seg) in segs.iter_mut().enumerate() {
            *seg = *guard.planned_segment_from_tail((n - k) as u8).unwrap();
        }
        (c[5], c[6])
    }

    /// Batched chaining as the planner does it: several segments are appended per chaining, each
    /// one entering and exiting at its rest speed (the planner leaves the exit of the segment
    /// before as it was). Along collinear segments at the same speed, no junction may stay at rest
    /// except the end of the plan.
    ///
    /// Regression: when the rest speed was the same on both sides of the previous end of the plan
    /// (any uniform machine), the window could start right at it (its braking distance is short),
    /// pinning that stop for good: the motion halted every batch (a walking gait took 50% longer).
    #[futures_test::test]
    async fn test_batched_chaining_releases_the_previous_end_of_the_plan() {
        const N: usize = <Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize;
        const BATCH: usize = 4;
        // Short braking distance (high acceleration and jerk) compared to a batch of segments
        let (v_t, a_max, j_max, len) = (100.0f32, 6000.0f32, 2.0e6f32, 5.0f32);
        let rest = Real::from_f32(10.0);
        let tol = Real::from_f32(0.01);
        let mut queue: std::vec::Vec<Segment> = std::vec::Vec::new();
        let (mut executed, mut stops) = (0u32, 0u32);
        for step in 0..60 {
            for _ in 0..BATCH {
                let mut seg = create_test_segment(len, v_t, a_max, j_max);
                seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
                seg.speed_target_su_s = Real::from_f32(v_t);
                seg.speed_enter_su_s = rest;
                seg.speed_exit_su_s = rest;
                seg.speed_rest_su_s = rest;
                queue.push(seg);
            }
            // As `chain_behind_executing`, with the batch still unchained
            {
                use hwa::controllers::motion_control::MovType;
                let mut rb = RingBuffer::new();
                rb.data[0] = PlanEntry::Executing(MovType::Move(Default::default(), Default::default()), false, 0);
                for (k, seg) in queue.iter().enumerate() {
                    rb.data[k + 1] = PlanEntry::PlannedMove(*seg, Default::default(), Default::default(), false, k as u32 + 1);
                }
                rb.used = queue.len() as u8 + 1;
                rb.head = 0;
                rb.unchained = BATCH as u8;
                let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
                let mut guard = mutex.lock().await;
                perform_motion_chaining_after_append(&mut guard, tol).unwrap();
                let n = queue.len();
                for (k, seg) in queue.iter_mut().enumerate() {
                    *seg = *guard.planned_segment_from_tail((n - k) as u8).unwrap();
                }
            }
            // The stepper executes the oldest ones (all but the last batch at the end)
            while queue.len() > N - 1 - BATCH || (step == 59 && queue.len() > BATCH) {
                let seg = queue.remove(0);
                executed += 1;
                if !(seg.speed_exit_su_s > rest * Real::from_f32(1.001)) {
                    stops += 1;
                }
            }
        }
        assert!(executed > 100, "{} segments executed", executed);
        assert_eq!(stops, 0, "{} stops in {} collinear segments", stops, executed);
    }

    /// Streaming (the native simulator scenario): segments are appended and chained while the
    /// oldest ones are consumed, so the pinned entry of the chain is often in the middle of a
    /// brake (`a < 0`). A consistent plan must stay feasible when re-chained from any of its
    /// states: no braking floor is raised, no run is left infeasible ("Can not brake from .. to
    /// the end of the plan"), and no full-stop corner is crossed at speed.
    ///
    /// Regressions: the brake in progress was modeled as a fresh ramp from the pinned speed (the
    /// jerk phase counted twice), the shallow-brake case of that model, and the rounding margin of
    /// a run planned exactly at its limit (the braking floor jumps from 0 to ~v/2 there).
    #[futures_test::test]
    async fn test_streaming_chain_stays_feasible_from_pinned_brakes() {
        const N: usize = <Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize;
        let mut seed: u64 = 0x2545_F491_4F6C_DD1D;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / (1u64 << 24) as f32
        };
        // The limits of the native simulator (effective jerk 4800)
        let (v_t, a_max, j_max) = (117.0f32, 3600.0f32, 7200.0f32);
        let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
        let rest = Real::from_f32(10.0);
        let (mut floors, mut infeasible, mut corners, mut pinned_brakes) = (0u32, 0u32, 0u32, 0u32);
        for run in 0..6 {
            let mut queue: std::vec::Vec<Segment> = std::vec::Vec::new();
            for step in 0..400 {
                // Lengths above the step quantization (2·tol) and only collinear or full-stop
                // junctions: the plans are always consistent
                let len = if rnd() < 0.6 { 0.1 + 0.6 * rnd() } else { 0.7 + 4.0 * rnd() };
                let mut seg = create_test_segment(len, v_t, a_max, j_max);
                seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
                seg.speed_target_su_s = Real::from_f32(v_t);
                seg.proj_prev = if !queue.is_empty() && rnd() < 0.12 { Real::from_f32(0.0) } else { Real::from_f32(1.0) };
                seg.speed_enter_su_s = if queue.is_empty() { rest } else { queue.last().unwrap().speed_exit_su_s };
                seg.speed_exit_su_s = rest;
                seg.speed_rest_su_s = rest;
                if let Some(prev) = queue.last_mut() {
                    // As the planner: the previous tail may now exit up to its target speed
                    prev.speed_exit_su_s = prev.speed_target_su_s;
                }
                queue.push(seg);
                if queue.len() >= 2 {
                    if queue[0].accel_enter_su_s < math::ZERO {
                        pinned_brakes += 1;
                    }
                    let (f, i) = chain_behind_executing(&mut queue, step % 3 != 0, tol).await;
                    if f + i > 0 {
                        std::println!("run {} step {}: {} braking floors, {} infeasible runs", run, step, f, i);
                    }
                    floors += f;
                    infeasible += i;
                }
                for k in 1..queue.len() {
                    if queue[k].proj_prev.is_zero() && queue[k].speed_enter_su_s > rest * Real::from_f32(1.0001) {
                        corners += 1;
                    }
                }
                // The stepper takes the oldest segments (a full queue, or at random)
                while queue.len() >= N - 1 || (queue.len() > 2 && rnd() < 0.35) {
                    queue.remove(0);
                }
            }
        }
        assert!(pinned_brakes > 100, "the scenario must pin brakes in progress ({})", pinned_brakes);
        assert_eq!((floors, infeasible, corners), (0, 0, 0), "braking floors, infeasible runs, corners at speed");
    }

    /// A pinned brake in progress whose next knot is above `v − a²/(2j)` (the speed it reaches by
    /// ramping the deceleration down right away) is not the continuation of a single ramp: it must
    /// not be modeled as one (the knot was raised above the pinned speed and its corner limit, and
    /// the plan could not brake to its end). Case taken from the native simulator.
    #[futures_test::test]
    async fn test_shallow_pinned_brake_does_not_raise_knots() {
        let (a_max, j_max) = (1000.0f32, 7200.0f32);
        let lens = [0.2897767f32, 1.2066702, 0.7076567, 0.78842163, 0.5944775, 0.8970152];
        let caps = [0.0f32, 41.4784, 44.88704, 44.95691, 44.964138, 44.955593];
        let (v_pin, a_pin) = (Real::from_f32(41.31011), Real::from_f32(-185.28748));
        let mut segs: std::vec::Vec<Segment> = std::vec::Vec::new();
        for k in 0..lens.len() {
            let mut seg = create_test_segment(lens[k], 45.0, a_max, j_max);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(45.0);
            // cap = min(targets)·proj
            seg.proj_prev = Real::from_f32(if k == 0 { 1.0 } else { caps[k] / 45.0 });
            seg.speed_enter_su_s = if k == 0 { v_pin } else { Real::from_f32(45.0) };
            seg.accel_enter_su_s = if k == 0 { a_pin } else { math::ZERO };
            seg.speed_exit_su_s = Real::from_f32(if k + 1 == lens.len() { 10.0 } else { 45.0 });
            segs.push(seg);
        }
        let (floors, infeasible) = chain_behind_executing(&mut segs, false, Real::from_f32(0.0178)).await;
        assert_eq!(infeasible, 0, "the plan must brake to its end");
        assert_eq!(floors, 0, "no knot raised by the braking floor");
        for k in 1..segs.len() {
            assert!(
                segs[k].speed_enter_su_s <= Real::from_f32(caps[k] * 1.0001),
                "junction {} above its corner limit: {:?} > {}", k, segs[k].speed_enter_su_s, caps[k]
            );
        }
        assert_eq!(segs[0].speed_enter_su_s, v_pin);
        assert_eq!(segs[0].accel_enter_su_s, a_pin);
    }

    /// A brake planned exactly at its limit, re-chained from a pinned state along it, must not
    /// lose the corner to rounding (the braking floor jumped from 0 to 41.7 mm/s for a shortfall
    /// of 1.5e-7). Case taken from the native simulator: a 90° corner after six ~0.54 mm segments.
    #[futures_test::test]
    async fn test_pinned_brake_at_the_limit_keeps_the_corner() {
        let (v_t, a_max, j_max) = (117.0f32, 3600.0f32, 7200.0f32);
        let lens = [0.5360297f32, 0.53795516, 0.5399555, 0.54202497, 0.54417104, 0.54638815, 1.0, 0.5, 0.5];
        let (v_pin, a_pin) = (Real::from_f32(53.774014), Real::from_f32(-362.79984));
        let mut segs: std::vec::Vec<Segment> = std::vec::Vec::new();
        for k in 0..lens.len() {
            let mut seg = create_test_segment(lens[k], v_t, a_max, j_max);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(v_t);
            seg.proj_prev = Real::from_f32(if k == 6 { 0.0 } else { 1.0 });
            seg.speed_enter_su_s = if k == 0 { v_pin } else { Real::from_f32(v_t) };
            seg.accel_enter_su_s = if k == 0 { a_pin } else { math::ZERO };
            seg.speed_exit_su_s = Real::from_f32(if k + 1 == lens.len() { 10.0 } else { v_t });
            segs.push(seg);
        }
        let (floors, infeasible) = chain_behind_executing(&mut segs, false, Real::from_f32(0.0178)).await;
        assert_eq!((floors, infeasible), (0, 0), "braking floors, infeasible runs");
        assert!(segs[6].speed_enter_su_s.is_zero(), "90° corner crossed at {:?}", segs[6].speed_enter_su_s);
        // The brake continues the pinned one: decreasing speeds and decelerations at every junction
        for k in 1..6 {
            assert!(segs[k].speed_enter_su_s < segs[k - 1].speed_enter_su_s, "junction {}", k);
            assert!(segs[k].accel_enter_su_s < math::ZERO, "junction {}", k);
        }
    }

    /// No junction is planned above its corner limit `cap = min(targets)·proj`. The passes only
    /// bound the knots: the junctions inside a run were sampled from its S-Curve ramp (which passes
    /// them faster than the zero-acceleration peaks the passes reason with), so a gentle corner
    /// inside an acceleration or a brake was crossed above its limit (e.g. 25.9 mm/s at a 19.3 mm/s
    /// corner in the native simulator). Streaming, with intermediate corners and targets.
    #[futures_test::test]
    async fn test_junctions_respect_corner_limits() {
        const N: usize = <Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize;
        let mut seed: u64 = 0xD1B5_4A32_D192_ED03;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / (1u64 << 24) as f32
        };
        let (a_max, j_max) = (3600.0f32, 7200.0f32);
        let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
        let rest = Real::from_f32(10.0);
        let (mut over, mut worst, mut junctions) = (0u32, 0.0f32, 0u32);
        let (mut floors, mut infeasible) = (0u32, 0u32);
        for run in 0..6 {
            let mut queue: std::vec::Vec<Segment> = std::vec::Vec::new();
            for step in 0..400 {
                let len = if rnd() < 0.6 { 0.1 + 0.6 * rnd() } else { 0.7 + 4.0 * rnd() };
                let v_t = if rnd() < 0.8 { 117.0 } else { 20.0 + 97.0 * rnd() };
                let mut seg = create_test_segment(len, 117.0, a_max, j_max);
                seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
                seg.speed_target_su_s = Real::from_f32(v_t);
                let r = rnd();
                seg.proj_prev = Real::from_f32(if queue.is_empty() || r < 0.6 { 1.0 } else if r < 0.7 { 0.0 } else { rnd() });
                seg.speed_enter_su_s = if queue.is_empty() { rest } else { queue.last().unwrap().speed_exit_su_s };
                seg.speed_exit_su_s = rest;
                seg.speed_rest_su_s = rest;
                if let Some(prev) = queue.last_mut() {
                    prev.speed_exit_su_s = prev.speed_target_su_s;
                }
                queue.push(seg);
                if queue.len() >= 2 {
                    let (f, i) = chain_behind_executing(&mut queue, step % 3 != 0, tol).await;
                    floors += f;
                    infeasible += i;
                    for k in 1..queue.len() {
                        let cap = (queue[k - 1].speed_target_su_s.min(queue[k].speed_target_su_s)
                            * queue[k].proj_prev.max(math::ZERO))
                            .max(rest);
                        let v = queue[k].speed_enter_su_s;
                        junctions += 1;
                        if v > cap * Real::from_f32(1.0001) + Real::from_f32(1.0e-4) {
                            over += 1;
                            worst = worst.max((v - cap).to_f64() as f32);
                            if over <= 5 {
                                std::println!("run {} step {} junction {}: v={:?} > cap={:?}", run, step, k, v, cap);
                            }
                        }
                    }
                }
                while queue.len() >= N - 1 || (queue.len() > 2 && rnd() < 0.35) {
                    queue.remove(0);
                }
            }
        }
        assert_eq!(over, 0, "{} of {} junctions above their corner limit (worst by {} mm/s)", over, junctions, worst);
        assert_eq!((floors, infeasible), (0, 0), "braking floors, infeasible runs");
    }

    /// A corner whose limit is below the rest speed (a slow move, 20 mm/s, turning 77°: cap 4.5
    /// mm/s, with a 10 mm/s rest) right after a pinned entry at rest: the plan can not brake below
    /// the rest speed there, and it does not need to (the segments turn at their rest speed
    /// instantly). Case taken from the native simulator, where the braking floor raised it.
    #[futures_test::test]
    async fn test_corner_below_rest_speed_after_rest_pin() {
        let lens = [0.12423056f32, 0.2727426, 0.7269057];
        let targets = [21.041721f32, 20.000002, 20.0];
        let projs = [1.0f32, 0.98514414, 0.22295797];
        let rest = Real::from_f32(10.0);
        let mut segs: std::vec::Vec<Segment> = std::vec::Vec::new();
        for k in 0..lens.len() {
            let mut seg = create_test_segment(lens[k], 117.0, 1000.0, 7200.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(targets[k]);
            seg.proj_prev = Real::from_f32(projs[k]);
            seg.speed_enter_su_s = rest;
            seg.speed_exit_su_s = rest;
            seg.speed_rest_su_s = rest;
            segs.push(seg);
        }
        let (floors, infeasible) = chain_behind_executing(&mut segs, true, Real::from_f32(0.0178)).await;
        assert_eq!((floors, infeasible), (0, 0), "braking floors, infeasible runs");
        assert!(segs[2].speed_enter_su_s <= rest, "corner at {:?}", segs[2].speed_enter_su_s);
    }

    /// Previous-plan fallback: the pinned state of a brake planned as blends (the admissibility
    /// repair left a corner at its limit with a non-zero acceleration, 12.97 mm/s at −169 mm/s²)
    /// is off the ramp model: re-planning from it found the brake infeasible over two segments
    /// and raised the braking floor (that corner crossed at 30 mm/s). The previous plan, which
    /// executes it, is kept up to its first zero-acceleration junction. Case taken from the
    /// streaming test.
    #[futures_test::test]
    async fn test_previous_plan_kept_when_pin_is_off_the_ramp_model() {
        let rest = 10.0f32;
        let lens = [0.14272168f32, 3.995992, 0.4072541, 0.8693208];
        let caps = [117.0f32, 68.65028, 12.974175, rest];
        // Previous plan (entry state of each segment), exiting at rest
        let prev = [(48.13549f32, -73.75681f32), (47.89507, -88.02341), (12.974174, -168.97357), (rest, 0.0)];
        let mut segs: std::vec::Vec<Segment> = std::vec::Vec::new();
        for k in 0..lens.len() {
            let mut seg = create_test_segment(lens[k], 117.0, 3600.0, 7200.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(117.0);
            seg.proj_prev = Real::from_f32(caps[k] / 117.0);
            seg.speed_rest_su_s = Real::from_f32(rest);
            seg.speed_enter_su_s = Real::from_f32(prev[k].0);
            seg.accel_enter_su_s = Real::from_f32(prev[k].1);
            let next = prev.get(k + 1).copied().unwrap_or((rest, 0.0));
            seg.speed_exit_su_s = Real::from_f32(next.0);
            seg.accel_exit_su_s = Real::from_f32(next.1);
            segs.push(seg);
        }
        let (floors, infeasible) = chain_behind_executing(&mut segs, false, Real::from_f32(0.0178)).await;
        assert_eq!((floors, infeasible), (0, 0), "braking floors, infeasible runs");
        for k in 1..segs.len() {
            assert!(
                segs[k].speed_enter_su_s <= Real::from_f32(caps[k].max(rest) * 1.0001),
                "junction {} above its corner limit: {:?} > {}", k, segs[k].speed_enter_su_s, caps[k]
            );
            assert_eq!(segs[k - 1].speed_exit_su_s, segs[k].speed_enter_su_s, "velocity continuity");
            assert_eq!(segs[k - 1].accel_exit_su_s, segs[k].accel_enter_su_s, "acceleration continuity");
        }
    }

    /// A corner split inside a brake lowers the peak knot before it below the speed an
    /// accelerating pinned entry still rises to (`v + a²/(2j)`): that knot is then no longer an
    /// extremum and is dropped (a single brake from the pin), instead of forcing a zero
    /// acceleration that made the run infeasible and raised the braking floors. Case taken from
    /// the native simulator.
    #[futures_test::test]
    async fn test_corner_split_after_accelerating_pin() {
        let (a_max, j_max) = (3600.0f32, 7200.0f32);
        let lens = [0.5238806f32, 0.3757176, 0.5238984, 0.32330424, 0.26548192, 0.27506837];
        let caps = [0.0f32, 20.223816, 19.286053, 17.708437, 19.177141, 18.5983];
        let (v_pin, a_pin) = (Real::from_f32(16.569674), Real::from_f32(140.20895));
        let mut segs: std::vec::Vec<Segment> = std::vec::Vec::new();
        for k in 0..lens.len() {
            let mut seg = create_test_segment(lens[k], 117.0, a_max, j_max);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(117.0);
            seg.proj_prev = Real::from_f32(if k == 0 { 1.0 } else { caps[k] / 117.0 });
            seg.speed_enter_su_s = if k == 0 { v_pin } else { Real::from_f32(117.0) };
            seg.accel_enter_su_s = if k == 0 { a_pin } else { math::ZERO };
            seg.speed_exit_su_s = Real::from_f32(if k + 1 == lens.len() { 10.0 } else { 117.0 });
            segs.push(seg);
        }
        let (floors, infeasible) = chain_behind_executing(&mut segs, false, Real::from_f32(0.0178)).await;
        assert_eq!((floors, infeasible), (0, 0), "braking floors, infeasible runs");
        for k in 1..segs.len() {
            assert!(
                segs[k].speed_enter_su_s <= Real::from_f32(caps[k] * 1.0001),
                "junction {} above its corner limit: {:?} > {}", k, segs[k].speed_enter_su_s, caps[k]
            );
        }
    }

    /// Ramp shift: a brake with slack passes a gentle corner near its end above the limit when
    /// placed as late as possible. It is shifted (brakes earlier, cruises at the bottom last) by
    /// the least amount that passes the corner within its limit, still as a single ramp: the
    /// corner keeps a braking acceleration, instead of becoming a zero-acceleration plateau (a
    /// split knot). 60 → 10 mm/s over 6 mm (the ramp needs 3.5), 30 mm/s corner 1 mm before the end.
    #[futures_test::test]
    async fn test_ramp_shift_passes_corner_within_a_single_brake() {
        let rest = Real::from_f32(10.0);
        let cap = 30.0f32;
        let lens = [5.0f32, 1.0];
        let mut segs: std::vec::Vec<Segment> = std::vec::Vec::new();
        for k in 0..lens.len() {
            let mut seg = create_test_segment(lens[k], 117.0, 3600.0, 30000.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(117.0);
            seg.proj_prev = Real::from_f32(if k == 0 { 1.0 } else { cap / 117.0 });
            seg.speed_rest_su_s = rest;
            seg.speed_enter_su_s = if k == 0 { Real::from_f32(60.0) } else { rest };
            seg.speed_exit_su_s = rest;
            segs.push(seg);
        }
        let (floors, infeasible) = chain_behind_executing(&mut segs, false, Real::from_f32(0.0178)).await;
        assert_eq!((floors, infeasible), (0, 0), "braking floors, infeasible runs");
        let (v1, a1) = (segs[1].speed_enter_su_s, segs[1].accel_enter_su_s);
        assert!(v1 <= Real::from_f32(cap * 1.0001), "corner at {:?} > {}", v1, cap);
        assert!(v1 > Real::from_f32(cap * 0.95), "corner far below its limit: {:?}", v1);
        assert!(a1 < math::ZERO, "the corner is crossed braking (single ramp), a={:?}", a1);
    }

    /// As the planner: new segments wait unchained at their own rest speed (10 mm/s, or 2 mm/s
    /// for slow moves: it differs from the exit of the segment before) and are chained in batches
    /// (`CHAIN_BATCH`) while the stepper consumes the queue. After every chaining the whole queue
    /// is continuous (speed and acceleration), every blend is admissible (the profile never alters
    /// a boundary condition: "Infeasible exit acceleration .. capped"), no corner exceeds
    /// `max(cap, rest)` and no braking floor is raised. Regressions: the previous-plan fallback cut
    /// the plan at such a junction (a speed jump), and a pinned segment left non-admissible.
    #[futures_test::test]
    async fn test_batched_chaining_with_mixed_rest_speeds() {
        const N: usize = <Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize;
        const CHAIN_BATCH: usize = 4;
        let mut seed: u64 = 0x94D0_49BB_1331_11EB;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / (1u64 << 24) as f32
        };
        let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
        let (mut floors, mut infeasible, mut jumps, mut inadmissible, mut over) = (0u32, 0u32, 0u32, 0u32, 0u32);
        let mut mixed = 0u32;
        for _run in 0..8 {
            let mut queue: std::vec::Vec<Segment> = std::vec::Vec::new();
            let mut unchained = 0usize;
            for _step in 0..400 {
                let slow = rnd() < 0.2;
                let len = if rnd() < 0.6 { 0.1 + 0.6 * rnd() } else { 0.7 + 4.0 * rnd() };
                let (target, rest, a_max) = if slow { (5.0 + 15.0 * rnd(), 2.0, 1000.0) } else { (117.0, 10.0, 3600.0) };
                let mut seg = create_test_segment(len, 117.0, a_max, 7200.0);
                seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
                seg.speed_target_su_s = Real::from_f32(target);
                seg.proj_prev = Real::from_f32(if queue.is_empty() || rnd() < 0.6 { 1.0 } else { rnd() });
                seg.speed_rest_su_s = Real::from_f32(rest);
                seg.speed_enter_su_s = seg.speed_rest_su_s;
                seg.speed_exit_su_s = seg.speed_rest_su_s;
                if let Some(prev) = queue.last() {
                    if prev.speed_exit_su_s != seg.speed_enter_su_s {
                        mixed += 1;
                    }
                }
                queue.push(seg);
                unchained += 1;
                if queue.len() >= 2 && (unchained >= CHAIN_BATCH || queue.len() <= CHAIN_BATCH + 2) {
                    let (f, i) = chain_behind_executing(&mut queue, true, tol).await;
                    floors += f;
                    infeasible += i;
                    unchained = 0;
                    for k in 0..queue.len() {
                        let q = &queue[k];
                        if k > 0 {
                            let p = &queue[k - 1];
                            if p.speed_exit_su_s != q.speed_enter_su_s || p.accel_exit_su_s != q.accel_enter_su_s {
                                jumps += 1;
                            }
                            let cap = (p.speed_target_su_s.min(q.speed_target_su_s) * q.proj_prev)
                                .max(p.speed_rest_su_s.min(q.speed_rest_su_s));
                            if q.speed_enter_su_s > cap * Real::from_f32(1.0001) + Real::from_f32(1.0e-4) {
                                over += 1;
                            }
                        }
                        if !(q.accel_enter_su_s.is_zero() && q.accel_exit_su_s.is_zero())
                            && !crate::motion::blend_is_admissible(
                                q.displacement_su, q.speed_enter_su_s, q.speed_exit_su_s,
                                q.accel_enter_su_s, q.accel_exit_su_s)
                        {
                            inadmissible += 1;
                        }
                    }
                }
                while queue.len() >= N - 1 || (queue.len() > 2 && rnd() < 0.35) {
                    queue.remove(0);
                    unchained = unchained.min(queue.len());
                }
            }
        }
        assert!(mixed > 200, "the scenario must append segments at a different rest speed ({})", mixed);
        assert_eq!(
            (jumps, inadmissible, over, floors, infeasible), (0, 0, 0, 0, 0),
            "speed/accel jumps, inadmissible blends, corners above max(cap, rest), braking floors, infeasible runs"
        );
    }

    /// The pinned segment brakes (27 mm/s at −405 mm/s²) and the previous plan ends at its rest
    /// speed (10 mm/s); slow moves (rest 2 mm/s) were appended after a corner. The ramp model
    /// continues the brake down to 2 mm/s, but the blend of the pinned segment does not exist (the
    /// profile capped its exit acceleration: "Infeasible exit acceleration .. capped"). The
    /// previous plan is kept up to that junction, made continuous (the appended segment entered at
    /// its own rest speed), and only the slow moves are planned again. Case taken from the native
    /// simulator.
    #[futures_test::test]
    async fn test_pinned_segment_blend_kept_admissible_with_previous_plan() {
        let mut segs: std::vec::Vec<Segment> = std::vec::Vec::new();
        // (length, target, rest, proj_prev, a_max)
        let spec = [(1.3228718f32, 117.0f32, 10.0f32, 1.0f32, 3600.0f32), (0.8180121, 5.0, 2.0, 0.0, 1000.0), (0.40000057, 5.0, 2.0, 1.0, 1000.0)];
        for &(len, target, rest, proj, a_max) in spec.iter() {
            let mut seg = create_test_segment(len, 117.0, a_max, 7200.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(target);
            seg.speed_rest_su_s = Real::from_f32(rest);
            seg.proj_prev = Real::from_f32(proj);
            seg.speed_enter_su_s = Real::from_f32(rest);
            seg.speed_exit_su_s = Real::from_f32(rest);
            segs.push(seg);
        }
        segs[0].speed_enter_su_s = Real::from_f32(27.064701);
        segs[0].accel_enter_su_s = Real::from_f32(-404.74823);
        let (floors, infeasible) = chain_behind_executing(&mut segs, false, Real::from_f32(0.0178)).await;
        assert_eq!((floors, infeasible), (0, 0), "braking floors, infeasible runs");
        for k in 0..segs.len() {
            let q = &segs[k];
            if k > 0 {
                assert_eq!(segs[k - 1].speed_exit_su_s, q.speed_enter_su_s, "velocity continuity at {}", k);
                assert_eq!(segs[k - 1].accel_exit_su_s, q.accel_enter_su_s, "acceleration continuity at {}", k);
            }
            assert!(
                (q.accel_enter_su_s.is_zero() && q.accel_exit_su_s.is_zero())
                    || crate::motion::blend_is_admissible(q.displacement_su, q.speed_enter_su_s,
                        q.speed_exit_su_s, q.accel_enter_su_s, q.accel_exit_su_s),
                "segment {} blend not admissible: v {:?} -> {:?}, a {:?} -> {:?}",
                k, q.speed_enter_su_s, q.speed_exit_su_s, q.accel_enter_su_s, q.accel_exit_su_s
            );
        }
        assert_eq!(segs[0].speed_enter_su_s, Real::from_f32(27.064701), "pinned entry");
    }

    /// `ramp_position` inverts `ramp_state` in the speed (all the ramp regimes and phases)
    #[test]
    fn test_ramp_position_inverts_ramp_state() {
        let mut seed: u64 = 0x5851_F42D_4C95_7F2D;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / (1u64 << 24) as f32
        };
        for _ in 0..20000 {
            let (a, j) = (Real::from_f32(500.0 + 3000.0 * rnd()), Real::from_f32(2000.0 + 50000.0 * rnd()));
            let v0 = Real::from_f32(100.0 * rnd() * rnd());
            let v1 = v0 + Real::from_f32(0.01 + 150.0 * rnd());
            // (in the speed: near the ends of the ramp the speed is flat and the position is
            // ill-conditioned)
            let v = v0 + (v1 - v0) * Real::from_f32(rnd());
            let x = ramp_position(v0, v1, v, a, j);
            let (vb, _) = ramp_state(v0, v1, x, a, j);
            assert!(
                ((vb - v).abs() / v1).to_f64() < 1.0e-4,
                "v={:?} -> x={:?} -> v={:?} (v0={:?} v1={:?})", v, x, vb, v0, v1
            );
        }
    }

    /// Velocity continuity must hold even for the 3-segment corner case.
    #[futures_test::test]
    async fn test_velocity_continuity_3_segments_corner() {
        let mut rb = RingBuffer::new();

        let mut seg1 = create_test_segment(100.0, 50.0, 1000.0, 5000.0);
        seg1.unit_vector_dir.x = Some(Real::from_f32(1.0));
        seg1.proj_next = Real::from_f32(0.0);
        seg1.speed_enter_su_s = Real::from_f32(0.0);
        seg1.speed_exit_su_s = Real::from_f32(50.0);
        seg1.speed_target_su_s = Real::from_f32(50.0);

        let mut seg2 = create_test_segment(100.0, 50.0, 1000.0, 5000.0);
        seg2.unit_vector_dir.y = Some(Real::from_f32(1.0));
        seg2.proj_prev = Real::from_f32(0.0);
        seg2.proj_next = Real::from_f32(1.0);
        seg2.speed_enter_su_s = Real::from_f32(0.0);
        seg2.speed_exit_su_s = Real::from_f32(50.0);
        seg2.speed_target_su_s = Real::from_f32(50.0);

        let mut seg3 = create_test_segment(100.0, 50.0, 1000.0, 5000.0);
        seg3.unit_vector_dir.y = Some(Real::from_f32(1.0));
        seg3.proj_prev = Real::from_f32(1.0);
        seg3.speed_enter_su_s = Real::from_f32(0.0);
        seg3.speed_exit_su_s = Real::from_f32(0.0);
        seg3.speed_target_su_s = Real::from_f32(50.0);

        rb.data[0] = PlanEntry::PlannedMove(seg1, Default::default(), Default::default(), false, 0);
        rb.data[1] = PlanEntry::PlannedMove(seg2, Default::default(), Default::default(), false, 1);
        rb.data[2] = PlanEntry::PlannedMove(seg3, Default::default(), Default::default(), false, 2);
        rb.used = 3;
        rb.head = 0;

        let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        // Velocity must be continuous at all junctions.
        for i in 1..3_u8 {
            let v_exit = guard.planned_segment_from_tail(i + 1).unwrap().speed_exit_su_s;
            let v_enter = guard.planned_segment_from_tail(i).unwrap().speed_enter_su_s;
            assert!(
                (v_exit - v_enter).abs() < Real::from_f32(1e-4),
                "Velocity discontinuity at junction {}: v_exit={:?} v_enter={:?}",
                i, v_exit, v_enter
            );
        }

        // Acceleration must be continuous at all junctions.
        for i in 1..3_u8 {
            let a_exit = guard.planned_segment_from_tail(i + 1).unwrap().accel_exit_su_s;
            let a_enter = guard.planned_segment_from_tail(i).unwrap().accel_enter_su_s;
            assert!(
                (a_exit - a_enter).abs() < Real::from_f32(1e-4),
                "Acceleration discontinuity at junction {}: a_exit={:?} a_enter={:?}",
                i, a_exit, a_enter
            );
        }

        // No negative discriminants.
        for i in 1..=3_u8 {
            let seg = guard.planned_segment_from_tail(i).unwrap();
            let delta = quintic_delta(seg);
            assert!(
                delta >= Real::from_f32(-1e-4),
                "Negative discriminant for segment {} (corner): delta={:?}",
                i, delta
            );
        }
    }

    /// The oldest planned segment follows the segment the stepper is executing: its entry state
    /// (speed and acceleration) is pinned and must survive chaining untouched.
    #[futures_test::test]
    async fn test_pinned_entry_after_executing_segment() {
        use hwa::controllers::motion_control::MovType;
        let mut rb = RingBuffer::new();
        rb.data[0] = PlanEntry::Executing(
            MovType::Move(Default::default(), Default::default()),
            false,
            0,
        );
        let v_pin = Real::from_f32(20.0);
        let a_pin = Real::from_f32(150.0);
        for k in 1..=4usize {
            let mut seg = create_test_segment(20.0, 50.0, 500.0, 5000.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(50.0);
            seg.speed_enter_su_s = if k == 1 { v_pin } else { Real::from_f32(0.0) };
            seg.accel_enter_su_s = if k == 1 { a_pin } else { Real::from_f32(0.0) };
            seg.speed_exit_su_s = if k == 4 { Real::from_f32(0.0) } else { Real::from_f32(50.0) };
            rb.data[k] = PlanEntry::PlannedMove(seg, Default::default(), Default::default(), false, k as u32);
        }
        rb.used = 5;
        rb.head = 0;
        let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();
        let oldest = guard.planned_segment_from_tail(4).unwrap();
        assert_eq!(oldest.speed_enter_su_s, v_pin, "pinned entry speed was modified");
        assert_eq!(oldest.accel_enter_su_s, a_pin, "pinned entry acceleration was modified");
    }

    /// A segment appended while the stepper brakes towards the end of a full queue (the plan is
    /// queue-starved): the pinned entry of the window is in the middle of the final braking ramp
    /// and there is no zero-acceleration junction to start the window at (`chain_window` falls back
    /// to the whole chain). Appending only adds displacement, so the re-chained plan must stay
    /// within the limits: the mean deceleration of every segment, (v_in² − v_out²)/2s, can never
    /// exceed a_max (whatever the blends do inside, they must at least average it).
    #[futures_test::test]
    async fn test_append_while_braking_keeps_decel_within_limits() {
        use hwa::controllers::motion_control::MovType;
        let (n, len, v_t, a_max, j_max) = (12usize, 5.0f32, 300.0f32, 1500.0f32, 19620.0f32);
        let rest = Real::from_f32(5.0);
        let mk = |len: f32| {
            let mut seg = create_test_segment(len, v_t, a_max, j_max);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.speed_target_su_s = Real::from_f32(v_t);
            seg.speed_enter_su_s = rest;
            seg.speed_exit_su_s = rest;
            seg
        };
        let mean_decel = |seg: &Segment| -> f64 {
            let (vi, vo) = (seg.speed_enter_su_s.to_f64(), seg.speed_exit_su_s.to_f64());
            (vi * vi - vo * vo) / (2.0 * seg.displacement_su.to_f64())
        };
        let mut rb = RingBuffer::new();
        rb.head = 0;
        rb.used = 0;
        let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;
        // Queue the job as the planner does: the previous tail may now exit up to its target
        // speed, append, then chain the window
        for k in 0..n {
            if k > 0 {
                let prev = guard.mut_planned_segment_from_tail(1).unwrap();
                prev.speed_exit_su_s = prev.speed_target_su_s;
            }
            guard.data[k] = PlanEntry::PlannedMove(mk(len), Default::default(), Default::default(), false, k as u32);
            guard.used += 1;
            perform_motion_chaining_after_append(&mut guard, Real::zero()).unwrap();
        }
        // Segment k is at offset n - k from the tail
        let entry = |g: &RingBuffer, k: usize| *g.planned_segment_from_tail((n - k) as u8).unwrap();
        for k in 0..n {
            let d = mean_decel(&entry(&guard, k));
            assert!(d.abs() <= a_max as f64 * 1.001, "precondition: segment {} averages {} mm/s²", k, d);
        }
        // The stepper executes up to the deepest point of the final brake: the next segment is
        // pinned at the most negative junction acceleration
        let pin = (1..n)
            .min_by(|&a, &b| {
                let (aa, ab) = (entry(&guard, a).accel_enter_su_s.to_f64(), entry(&guard, b).accel_enter_su_s.to_f64());
                aa.partial_cmp(&ab).unwrap()
            })
            .unwrap();
        let (v_pin, a_pin) = (entry(&guard, pin).speed_enter_su_s, entry(&guard, pin).accel_enter_su_s);
        assert!(
            a_pin.to_f64() < -0.9 * a_max as f64,
            "precondition: the pinned entry must be deep in the brake (a = {:?})",
            a_pin
        );
        if std::env::var("CHAIN_DEBUG").is_ok() {
            for k in 0..n {
                let e = entry(&guard, k);
                std::println!("PRE k={} s={:.2} v_in={:.2} a_in={:.1} v_out={:.2} a_out={:.1} mean={:.1}", k, e.displacement_su.to_f64(), e.speed_enter_su_s.to_f64(), e.accel_enter_su_s.to_f64(), e.speed_exit_su_s.to_f64(), e.accel_exit_su_s.to_f64(), mean_decel(&e));
            }
            std::println!("pin={}", pin);
        }
        guard.data[pin - 1] = PlanEntry::Executing(MovType::Move(Default::default(), Default::default()), false, (pin - 1) as u32);
        guard.head = (pin - 1) as u8;
        guard.used = (n - (pin - 1)) as u8;
        // A slot was freed: a short segment arrives
        {
            let prev = guard.mut_planned_segment_from_tail(1).unwrap();
            prev.speed_exit_su_s = prev.speed_target_su_s;
        }
        guard.data[n] = PlanEntry::PlannedMove(mk(0.5), Default::default(), Default::default(), false, n as u32);
        guard.used += 1;
        perform_motion_chaining_after_append(&mut guard, Real::zero()).unwrap();

        if std::env::var("CHAIN_DEBUG").is_ok() {
            for offset in (1..guard.used).rev() {
                let e = *guard.planned_segment_from_tail(offset).unwrap();
                std::println!("POST off={} s={:.2} v_in={:.2} a_in={:.1} v_out={:.2} a_out={:.1} mean={:.1}", offset, e.displacement_su.to_f64(), e.speed_enter_su_s.to_f64(), e.accel_enter_su_s.to_f64(), e.speed_exit_su_s.to_f64(), e.accel_exit_su_s.to_f64(), mean_decel(&e));
            }
        }
        let oldest = *guard.planned_segment_from_tail(guard.used - 1).unwrap();
        assert_eq!(oldest.speed_enter_su_s, v_pin, "pinned entry speed was modified");
        assert_eq!(oldest.accel_enter_su_s, a_pin, "pinned entry acceleration was modified");
        for offset in (1..guard.used).rev() {
            let seg = *guard.planned_segment_from_tail(offset).unwrap();
            let d = mean_decel(&seg);
            assert!(
                d.abs() <= a_max as f64 * 1.001,
                "pinned at (v = {:?}, a = {:?}): segment at offset {} [{:?} -> {:?} over {:?}] averages {} mm/s² (a_max = {})",
                v_pin, a_pin, offset, seg.speed_enter_su_s, seg.speed_exit_su_s, seg.displacement_su, d, a_max
            );
        }
    }

    /// End-to-end: after chaining, the profiles the stepper will compute keep the planned
    /// boundary conditions (no internal capping) and are continuous in velocity and
    /// acceleration across every junction.
    /// The stepper re-derives each displacement from the step-quantized position before computing
    /// the profile (`Segment::fix_deviation`), changing it by up to one step. Reproduces the case
    /// reported from the Benchy integration run: planned `s = 0.4229` was admissible by a 0.85%
    /// margin, the stepper got `s = 0.4285` and had to cap `a_o` (acceleration discontinuity).
    #[test]
    fn test_admissibility_covers_displacement_correction() {
        let (v_i, v_o, a_i, a_o) = (
            Real::from_f32(11.036788),
            Real::from_f32(4.559544),
            Real::from_f32(-356.4954),
            Real::zero(),
        );
        let s_planned = Real::from_f32(0.4228953);
        let s_executed = Real::from_f32(0.42849958);
        let a_max = Real::from_f32(3600.0);
        // One full step per axis at 80 steps/mm on X and Y
        let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
        assert!(crate::motion::blend_is_admissible(s_planned, v_i, v_o, a_i, a_o));
        assert!(!crate::motion::blend_is_admissible(s_executed, v_i, v_o, a_i, a_o));
        assert!(segment_admissible(s_planned, v_i, v_o, a_i, a_o, a_max, Real::zero()));
        assert!(
            !segment_admissible(s_planned, v_i, v_o, a_i, a_o, a_max, tol),
            "a blend that the deviation correction can make infeasible must be repaired"
        );
        // The repaired exit acceleration is feasible over the whole correction interval
        let a_o_cap = admissible_exit_accel_cap(s_planned, v_i, v_o, a_i, tol);
        assert!(a_o_cap < a_o);
        for s in [s_planned - tol, s_executed, s_planned + tol] {
            assert!(crate::motion::blend_is_admissible(s, v_i, v_o, a_i, a_o_cap));
        }
    }

    /// Property: after chaining with a displacement tolerance, the stepper computes every profile
    /// without altering the planned boundary conditions for any displacement in `[s-tol, s+tol]`.
    /// Random chains of short segments with corners, some starting from a pinned entry state.
    #[futures_test::test]
    #[cfg(feature = "with-motion-chaining")]
    async fn test_chaining_robust_to_displacement_correction() {
        use crate::motion::{HybridMotionProfile, ProfileKind};
        fn rnd(seed: &mut u64) -> f32 {
            *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((*seed >> 40) as f32) / ((1u64 << 24) as f32)
        }
        let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
        let mut seed = 0x5eed_u64;
        let mut violations = [0usize; 2];
        let mut checked = 0usize;
        for _case in 0..300 {
            let n = 3 + (rnd(&mut seed) * 6.0) as usize;
            let (a_max, j_max) = (3600.0f32, if rnd(&mut seed) < 0.5 { 60000.0 } else { 360000.0 });
            let pinned = rnd(&mut seed) < 0.5;
            let mut segs = std::vec::Vec::new();
            for k in 0..n {
                let mut seg = create_test_segment(0.15 + 1.85 * rnd(&mut seed), 600.0, a_max, j_max);
                seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
                seg.proj_prev = Real::from_f32(if k == 0 { 1.0 } else { 0.2 + 0.8 * rnd(&mut seed) });
                seg.speed_target_su_s = Real::from_f32(30.0 + 570.0 * rnd(&mut seed));
                seg.speed_exit_su_s = if k == n - 1 { Real::zero() } else { seg.speed_target_su_s };
                if k == 0 && pinned {
                    // Exit state of a decelerating segment the stepper is executing
                    seg.speed_enter_su_s = Real::from_f32(5.0 + 20.0 * rnd(&mut seed));
                    seg.accel_enter_su_s = Real::from_f32(-600.0 * rnd(&mut seed));
                }
                segs.push(seg);
            }
            for (mode, chaining_tol) in [Real::zero(), tol].into_iter().enumerate() {
                let mut rb = RingBuffer::new();
                for (k, seg) in segs.iter().enumerate() {
                    rb.data[k] = PlanEntry::PlannedMove(*seg, Default::default(), Default::default(), false, k as u32);
                }
                rb.used = n as u8;
                rb.head = 0;
                let mutex = Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
                let mut guard = mutex.lock().await;
                perform_motion_chaining(&mut guard, chaining_tol).unwrap();
                for i in 1..=n as u8 {
                    let seg = *guard.planned_segment_from_tail(i).unwrap();
                    if i == n as u8 && pinned {
                        // Only the exit of the pinned segment can be repaired: covered above
                        continue;
                    }
                    for s in [seg.displacement_su - tol, seg.displacement_su, seg.displacement_su + tol] {
                        if !(s > Real::zero()) {
                            continue;
                        }
                        checked += mode;
                        let p = HybridMotionProfile::compute(
                            s,
                            seg.speed_enter_su_s,
                            seg.speed_exit_su_s,
                            seg.accel_enter_su_s,
                            seg.accel_exit_su_s,
                            &seg.constraints,
                            false,
                        )
                        .expect("profile must be computable");
                        let altered = p.kind == ProfileKind::Polynomial
                            && (p.a_o != seg.accel_exit_su_s
                                || !crate::motion::blend_is_admissible(p.s_o, p.v_i, p.v_o, p.a_i, p.a_o));
                        violations[mode] += altered as usize;
                    }
                }
            }
        }
        std::println!(
            "profiles altered by the stepper: {} without tolerance, {} with tolerance ({} checked)",
            violations[0], violations[1], checked
        );
        assert_eq!(violations[1], 0, "the planner must absorb the displacement correction");
    }

    #[futures_test::test]
    #[cfg(feature = "with-motion-chaining")]
    async fn test_chained_profiles_are_continuous() {
        use crate::motion::{HybridMotionProfile, MotionProfile};
        for &(n, len, v_max, a_max, j_max) in &[
            (5u8, 50.0f32, 50.0f32, 500.0f32, 5000.0f32),
            (6, 10.0, 100.0, 1500.0, 75000.0),
            (8, 2.0, 150.0, 3000.0, 60000.0),
            (8, 0.5, 200.0, 1500.0, 19620.0),
        ] {
            let mutex = make_collinear_rb(n, len, v_max, a_max, j_max).await;
            let mut guard = mutex.lock().await;
            perform_motion_chaining(&mut guard, Real::zero()).unwrap();
            let mut prev: Option<HybridMotionProfile> = None;
            for i in (1..=n).rev() {
                let seg = *guard.planned_segment_from_tail(i).unwrap();
                let p = HybridMotionProfile::compute(
                    seg.displacement_su,
                    seg.speed_enter_su_s,
                    seg.speed_exit_su_s,
                    seg.accel_enter_su_s,
                    seg.accel_exit_su_s,
                    &seg.constraints,
                    false,
                )
                .expect("profile must be computable");
                if p.kind == crate::motion::ProfileKind::Polynomial {
                    assert_eq!(p.a_o, seg.accel_exit_su_s, "the stepper must not alter planned BCs");
                    assert!(crate::motion::blend_is_admissible(p.s_o, p.v_i, p.v_o, p.a_i, p.a_o),
                        "blend must not reverse velocity: {:?}", p);
                }
                if let Some(q) = prev {
                    let tol_v = 1e-3 * v_max;
                    let tol_a = 1e-3 * a_max;
                    let dv = (q.eval_velocity(q.end_time()) - p.eval_velocity(math::ZERO)).abs();
                    let da = (q.eval_acceleration(q.end_time()) - p.eval_acceleration(math::ZERO)).abs();
                    assert!(dv.to_f64() as f32 <= tol_v, "velocity jump {:?} between {:?} and {:?}", dv, q, p);
                    assert!(da.to_f64() as f32 <= tol_a, "acceleration jump {:?} between {:?} and {:?}", da, q, p);
                }
                prev = Some(p);
            }
        }
    }

    /// B1: chaining incrementally (as the planner does, the window only) gives the same plan as
    /// chaining the whole queue after every append, on a pen-test-like curve of 0.55 mm segments
    /// with an accelerating start, gentle turns and a sharper corner. Invariants hold throughout.
    #[futures_test::test]
    async fn test_windowed_chaining_matches_full_chaining() {
        const N: usize = <Contract as HwiContract>::SEGMENT_QUEUE_SIZE as usize;
        let (v_t, a_max, j_max) = (116.7f32, 2400.0f32, 50000.0f32);
        let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
        let rest = Real::from_f32(2.5);
        let mut segs = std::vec::Vec::new();
        for k in 0..N {
            let mut seg = create_test_segment(0.55, v_t, a_max, j_max);
            seg.speed_target_su_s = Real::from_f32(v_t);
            seg.speed_enter_su_s = rest;
            seg.speed_exit_su_s = rest;
            // 3° turns, and a 60° corner in the middle
            seg.proj_prev = Real::from_f32(if k == 0 { 1.0 } else if k == N / 2 { 0.5 } else { 0.9986 });
            segs.push(seg);
        }
        let new_rb = || {
            let mut rb = RingBuffer::new();
            rb.head = 0;
            rb.used = 0;
            rb
        };
        let (incremental, full) = (Mutex::<hwa::AsyncNoopMutexType, _>::new(new_rb()), Mutex::<hwa::AsyncNoopMutexType, _>::new(new_rb()));
        let mut max_dv = 0.0f32;
        for k in 0..N {
            for (windowed, mutex) in [(true, &incremental), (false, &full)] {
                let mut guard = mutex.lock().await;
                // As the planner: the previous tail may now exit up to its target speed
                if k > 0 {
                    let prev = guard.mut_planned_segment_from_tail(1).unwrap();
                    prev.speed_exit_su_s = prev.speed_target_su_s;
                }
                guard.data[k] = PlanEntry::PlannedMove(segs[k], Default::default(), Default::default(), false, k as u32);
                guard.used += 1;
                if windowed {
                    perform_motion_chaining_after_append(&mut guard, tol).unwrap();
                } else {
                    perform_motion_chaining(&mut guard, tol).unwrap();
                }
            }
            let (gi, gf) = (incremental.lock().await, full.lock().await);
            let used = gi.used;
            if std::env::var("CHAIN_DEBUG").is_ok() {
                let vi: std::vec::Vec<f32> = (1..=used).rev().map(|i| gi.planned_segment_from_tail(i).unwrap().speed_exit_su_s.to_f64() as f32).collect();
                let vf: std::vec::Vec<f32> = (1..=used).rev().map(|i| gf.planned_segment_from_tail(i).unwrap().speed_exit_su_s.to_f64() as f32).collect();
                std::println!("k={} win={:?}\n     inc={:.1?}\n    full={:.1?}", k, chain_window(&gi, used), vi, vf);
            }
            for i in 1..=used {
                let (si, sf) = (gi.planned_segment_from_tail(i).unwrap(), gf.planned_segment_from_tail(i).unwrap());
                max_dv = max_dv.max((si.speed_exit_su_s - sf.speed_exit_su_s).abs().to_f64() as f32);
                if i < used {
                    let older = gi.planned_segment_from_tail(i + 1).unwrap();
                    assert_eq!(older.speed_exit_su_s, si.speed_enter_su_s, "velocity continuity");
                    assert_eq!(older.accel_exit_su_s, si.accel_enter_su_s, "acceleration continuity");
                }
                assert!(
                    si.accel_enter_su_s.is_zero() && si.accel_exit_su_s.is_zero()
                        || segment_admissible(si.displacement_su, si.speed_enter_su_s, si.speed_exit_su_s,
                            si.accel_enter_su_s, si.accel_exit_su_s, si.constraints.a_max, tol)
                        || i == used,
                    "blend admissibility"
                );
            }
        }
        std::println!("windowed vs full chaining: max junction speed difference {} mm/s", max_dv);
        assert!(max_dv <= 0.01 * v_t, "windowed chaining deviates {} mm/s from the full one", max_dv);
    }

    /// As [test_windowed_chaining_matches_full_chaining], with the appends of the planner: the
    /// appended segment keeps its rest-to-rest plan (the previous tail still exits at rest) and is
    /// the unchained one. 20 segments of `seg_len` (whatever the queue size), 3° turns and a 60°
    /// corner at the entry of segment `corner_at`, chained after every append (the planner
    /// batches them). Returns the largest junction speed difference of the windowed chaining
    /// against the chaining of the whole queue, after any append (the invariants are asserted).
    async fn windowed_vs_full_as_planner(seg_len: f32, corner_at: usize) -> f32 {
        const N: usize = 20;
        let (v_t, a_max, j_max) = (116.7f32, 2400.0f32, 50000.0f32);
        let tol = Real::from_f32(core::f32::consts::SQRT_2 / 80.0);
        let rest = Real::from_f32(2.5);
        let mut segs = std::vec::Vec::new();
        for k in 0..N {
            let mut seg = create_test_segment(seg_len, v_t, a_max, j_max);
            seg.speed_target_su_s = Real::from_f32(v_t);
            seg.speed_enter_su_s = rest;
            seg.speed_exit_su_s = rest;
            seg.proj_prev = Real::from_f32(if k == 0 { 1.0 } else if k == corner_at { 0.5 } else { 0.9986 });
            segs.push(seg);
        }
        let new_rb = || {
            let mut rb = RingBuffer::new();
            rb.head = 0;
            rb.used = 0;
            rb
        };
        let (incremental, full) = (Mutex::<hwa::AsyncNoopMutexType, _>::new(new_rb()), Mutex::<hwa::AsyncNoopMutexType, _>::new(new_rb()));
        let mut max_dv = 0.0f32;
        for k in 0..N {
            for (windowed, mutex) in [(true, &incremental), (false, &full)] {
                let mut guard = mutex.lock().await;
                guard.data[k] = PlanEntry::PlannedMove(segs[k], Default::default(), Default::default(), false, k as u32);
                guard.used += 1;
                guard.unchained = 1;
                if windowed {
                    perform_motion_chaining_after_append(&mut guard, tol).unwrap();
                } else {
                    perform_motion_chaining(&mut guard, tol).unwrap();
                }
                guard.unchained = 0;
            }
            let (gi, gf) = (incremental.lock().await, full.lock().await);
            let used = gi.used;
            for i in 1..=used {
                let (si, sf) = (gi.planned_segment_from_tail(i).unwrap(), gf.planned_segment_from_tail(i).unwrap());
                max_dv = max_dv.max((si.speed_exit_su_s - sf.speed_exit_su_s).abs().to_f64() as f32);
                if i < used {
                    let older = gi.planned_segment_from_tail(i + 1).unwrap();
                    assert_eq!(older.speed_exit_su_s, si.speed_enter_su_s, "velocity continuity");
                    assert_eq!(older.accel_exit_su_s, si.accel_enter_su_s, "acceleration continuity");
                }
            }
        }
        max_dv
    }

    /// Regression: the window started at any zero-acceleration junction one stopping distance from
    /// the end of the plan, assuming that older junctions can no longer change. A plateau below
    /// its cap, set by a corner inside the window, can: the corner speed rises as the end of the
    /// plan moves away (it was braking to that end), and the full chaining raises the junctions
    /// before it. Frozen, they stayed up to 12.5 mm/s slower (1.1 mm segments, corner at segment
    /// 10); also the cause of [test_windowed_chaining_matches_full_chaining] failing with
    /// `SEGMENT_QUEUE_SIZE=40`. The window now starts at a junction at its cap.
    #[futures_test::test]
    async fn test_windowed_chaining_relaxes_plateau_before_corner() {
        let max_dv = windowed_vs_full_as_planner(1.1, 10).await;
        assert!(max_dv <= 0.01 * 116.7, "windowed chaining deviates {} mm/s from the full one", max_dv);
    }

    /// Regression: the window was pinned at a cruise junction from which the ramp model can not
    /// brake into the corner within the window (the previous plan does it with blends): the plan
    /// needed braking floors, and the previous-plan fallback kept all but the appended segment, so
    /// nothing was committed (`w - m < 2` in [chain]). The appended segment kept its rest-to-rest
    /// plan: a full stop (2.5 mm/s) right after the corner, where the full chaining crosses at
    /// 57-75 mm/s, kept by the next fallbacks for good (2 mm segments, corner at segment 10). Such
    /// a window is now planned again over the whole queue before the fallback.
    #[futures_test::test]
    async fn test_windowed_chaining_never_freezes_a_stop_after_a_corner() {
        let max_dv = windowed_vs_full_as_planner(2.0, 10).await;
        assert!(max_dv <= 0.01 * 116.7, "windowed chaining deviates {} mm/s from the full one", max_dv);
    }

    /// Boundary conditions for the oldest segment: accel_enter == 0.
    #[futures_test::test]
    async fn test_oldest_segment_zero_accel_enter() {
        const N: u8 = 4;
        let mutex = make_collinear_rb(N, 50.0, 50.0, 500.0, 5000.0).await;
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        let oldest = guard.planned_segment_from_tail(N).unwrap();
        assert!(
            oldest.accel_enter_su_s.abs() < Real::from_f32(1e-6),
            "oldest accel_enter should be 0, got {:?}",
            oldest.accel_enter_su_s
        );
    }

    /// Boundary conditions for the newest segment: accel_exit == 0.
    #[futures_test::test]
    async fn test_newest_segment_zero_accel_exit() {
        const N: u8 = 4;
        let mutex = make_collinear_rb(N, 50.0, 50.0, 500.0, 5000.0).await;
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        let newest = guard.planned_segment_from_tail(1).unwrap();
        assert!(
            newest.accel_exit_su_s.abs() < Real::from_f32(1e-6),
            "newest accel_exit should be 0, got {:?}",
            newest.accel_exit_su_s
        );
    }

    /// Ring buffer with a SetPosition entry at the head (simulating G92 before G1 moves).
    /// perform_motion_chaining must only consider the consecutive PlannedMove entries
    /// from the tail, not the SetPosition entry, and must succeed.
    #[futures_test::test]
    async fn test_chaining_skips_setposition_at_head() {
        use hwa::controllers::motion_control::Position;
        let mut rb = RingBuffer::new();

        // Slot 0: SetPosition (simulates G92)
        rb.data[0] = PlanEntry::SetPosition(
            hwa::CommChannel::Internal,
            Position::new(),
            false,
            0,
        );
        // Slots 1-4: 4 collinear PlannedMoves
        for k in 1..=4usize {
            let mut seg = create_test_segment(10.0, 50.0, 500.0, 5000.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.proj_prev = Real::from_f32(1.0);
            seg.proj_next = Real::from_f32(1.0);
            seg.speed_target_su_s = Real::from_f32(50.0);
            seg.speed_enter_su_s = Real::from_f32(0.0);
            seg.speed_exit_su_s = if k == 4 {
                Real::from_f32(0.0)
            } else {
                Real::from_f32(50.0)
            };
            rb.data[k] = PlanEntry::PlannedMove(
                seg,
                Default::default(),
                Default::default(),
                false,
                k as u32,
            );
        }
        rb.used = 5; // 1 SetPosition + 4 PlannedMoves
        rb.head = 0;

        let mutex = embassy_sync::mutex::Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;

        // Must succeed (not Err) even with SetPosition at head
        let result = perform_motion_chaining(&mut guard, Real::zero());
        assert!(result.is_ok(), "chaining must succeed with SetPosition at head");

        // Velocity continuity for the 4 PlannedMove segments (offsets 1..=4 from tail)
        // tail(1)=data[4], tail(2)=data[3], tail(3)=data[2], tail(4)=data[1]
        for i in 1..4u8 {
            let v_exit = guard.planned_segment_from_tail(i + 1).unwrap().speed_exit_su_s;
            let v_enter = guard.planned_segment_from_tail(i).unwrap().speed_enter_su_s;
            assert!(
                (v_exit - v_enter).abs() < Real::from_f32(1e-3),
                "velocity discontinuity at junction {}: v_exit={:?} v_enter={:?}",
                i, v_exit, v_enter
            );
        }
    }

    /// After chaining with SetPosition at head, the inner junction velocities
    /// must be greater than the minimum speed (chaining actually improves them).
    #[futures_test::test]
    async fn test_chaining_with_setposition_raises_junction_speeds() {
        use hwa::controllers::motion_control::Position;
        let mut rb = RingBuffer::new();

        rb.data[0] = PlanEntry::SetPosition(
            hwa::CommChannel::Internal,
            Position::new(),
            false,
            0,
        );
        let min_speed = Real::from_f32(0.5);
        for k in 1..=4usize {
            let mut seg = create_test_segment(50.0, 50.0, 500.0, 5000.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.proj_prev = Real::from_f32(1.0);
            seg.proj_next = Real::from_f32(1.0);
            seg.speed_target_su_s = Real::from_f32(50.0);
            seg.speed_enter_su_s = min_speed;
            seg.speed_exit_su_s = if k == 4 { min_speed } else { Real::from_f32(50.0) };
            rb.data[k] = PlanEntry::PlannedMove(
                seg,
                Default::default(),
                Default::default(),
                false,
                k as u32,
            );
        }
        rb.used = 5;
        rb.head = 0;

        let mutex = embassy_sync::mutex::Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        // Inner junctions (between the 4 PlannedMoves): speeds must be above min_speed
        // tail(1)=data[4]=last, tail(4)=data[1]=first
        for i in 1..=3u8 {
            let v_junction = guard.planned_segment_from_tail(i + 1).unwrap().speed_exit_su_s;
            assert!(
                v_junction > min_speed,
                "inner junction {} speed {:?} should exceed min_speed {:?} after chaining",
                i, v_junction, min_speed
            );
        }
    }

    /// Verify that compute_maximum_enter_and_exit_speed physically clamps accelerations
    /// so the quintic velocity profile cannot go below zero: |a_i| <= v_i*(v_i+v_o)/s.
    #[test]
    fn test_physical_accel_bound_prevents_negative_velocity() {
        // Parameters that previously caused a_i=-189 with v_i=7.14 (nearly zero):
        // s~=0.546mm, v_i=7.14mm/s, v_o=22.77mm/s, a_max=1500, j_max=19620
        let mut seg = create_test_segment(0.546, 200.0, 1500.0, 19620.0);
        seg.speed_enter_su_s = Real::from_f32(7.14);
        seg.speed_exit_su_s = Real::from_f32(22.77);
        seg.accel_enter_su_s = Real::from_f32(0.0);
        seg.accel_exit_su_s = Real::from_f32(0.0);

        compute_maximum_enter_and_exit_speed(false, 0, &mut seg);

        let v_i = seg.speed_enter_su_s;
        let v_o = seg.speed_exit_su_s;
        let s = seg.displacement_su;
        let v_sum = v_i + v_o;
        let a_i = seg.accel_enter_su_s;
        let a_o = seg.accel_exit_su_s;

        // Physical bound: |a_i| <= v_i*(v_i+v_o)/s
        let a_i_bound = v_i * v_sum / s;
        let a_o_bound = v_o * v_sum / s;

        assert!(
            a_i >= -a_i_bound && a_i <= a_i_bound,
            "a_i={:?} violates physical bound +-{:?} for v_i={:?}, v_o={:?}, s={:?}",
            a_i, a_i_bound, v_i, v_o, s
        );
        assert!(
            a_o >= -a_o_bound && a_o <= a_o_bound,
            "a_o={:?} violates physical bound +-{:?}",
            a_o, a_o_bound
        );
    }

    /// Verify that acceleration continuity holds even with SetPosition at head.
    #[futures_test::test]
    async fn test_accel_continuity_with_setposition_at_head() {
        use hwa::controllers::motion_control::Position;
        let mut rb = RingBuffer::new();

        rb.data[0] = PlanEntry::SetPosition(
            hwa::CommChannel::Internal,
            Position::new(),
            false,
            0,
        );
        for k in 1..=4usize {
            let mut seg = create_test_segment(50.0, 50.0, 500.0, 5000.0);
            seg.unit_vector_dir.x = Some(Real::from_f32(1.0));
            seg.proj_prev = Real::from_f32(1.0);
            seg.proj_next = Real::from_f32(1.0);
            seg.speed_target_su_s = Real::from_f32(50.0);
            seg.speed_enter_su_s = Real::from_f32(0.0);
            seg.speed_exit_su_s = if k == 4 { Real::from_f32(0.0) } else { Real::from_f32(50.0) };
            rb.data[k] = PlanEntry::PlannedMove(
                seg,
                Default::default(),
                Default::default(),
                false,
                k as u32,
            );
        }
        rb.used = 5;
        rb.head = 0;

        let mutex = embassy_sync::mutex::Mutex::<hwa::AsyncNoopMutexType, _>::new(rb);
        let mut guard = mutex.lock().await;
        perform_motion_chaining(&mut guard, Real::zero()).unwrap();

        for i in 1..4u8 {
            let a_exit = guard.planned_segment_from_tail(i + 1).unwrap().accel_exit_su_s;
            let a_enter = guard.planned_segment_from_tail(i).unwrap().accel_enter_su_s;
            assert!(
                (a_exit - a_enter).abs() < Real::from_f32(1e-3),
                "accel discontinuity at junction {}: a_exit={:?} a_enter={:?}",
                i, a_exit, a_enter
            );
        }
    }

    /// Regression test: exact values from runtime error log.
    ///
    /// A start-from-rest segment with negligible floating-point residuals in both
    /// speeds and acceleration used to trigger NumericalError because
    /// `a_o = 1.4e-6` (not IEEE-754 zero) routed the segment into the quintic
    /// path, where `qa ≈ -1.17e-7` and `qb ≈ 5.4e-8` are both below f32::epsilon
    /// and were treated as negligible → "qa and qb are negligible" → NumericalError.
    /// The fix uses `is_negligible()` instead of `is_zero()` for the routing check.
    #[test]
    #[cfg(feature = "with-motion-chaining")]
    fn test_near_zero_accel_residual_no_numerical_error() {
        use crate::motion::motion_profile_traits::MotionProfile;
        use crate::motion::Constraints;
        use crate::motion::HybridMotionProfile;

        let constraints = Constraints {
            v_max: math::Real::from_f32(600.0),
            a_max: math::Real::from_f32(3600.0),
            j_max: math::Real::from_f32(7200.0),
        };
        // Exact values from the error log: start-from-rest, large displacement,
        // tiny floating-point residuals in v_i, v_o, and a_o.
        let result = HybridMotionProfile::compute(
            math::Real::from_f32(17.310705),
            math::Real::from_f32(7.673863e-11_f32),
            math::Real::from_f32(1.0839385e-7_f32),
            math::Real::from_f32(0.0),
            math::Real::from_f32(0.0000014089387_f32),
            &constraints,
            false,
        );
        assert!(
            result.is_ok(),
            "Start-from-rest with residual a_o must not produce NumericalError, got: {:?}",
            result.err()
        );
        let profile = result.unwrap();
        assert!(
            profile.end_time() > math::Real::from_f32(0.0),
            "end_time must be positive, got {:?}",
            profile.end_time()
        );
    }

    /// Regression test: exact values from runtime error log.
    ///
    /// A segment at cruise speed (v_i ≈ v_o ≈ v_max) with a tiny displacement
    /// and a small non-zero exit acceleration used to trigger NumericalError because
    /// the quadratic root formula `-qb + sqrt(delta)` suffered catastrophic
    /// cancellation in f32 when qb ≈ sqrt(delta) ≈ 600.
    /// The fix uses Vieta's stable formula: T = -2*qc / (qb + sqrt_delta). Ref: https://en.wikipedia.org/wiki/Vieta%27s_formulas
    #[test]
    #[cfg(feature = "with-motion-chaining")]
    fn test_quintic_vieta_stable_no_numerical_error() {
        use crate::motion::motion_profile_traits::MotionProfile;
        use crate::motion::HybridMotionProfile;
        use crate::motion::Constraints;

        let constraints = Constraints {
            v_max: math::Real::from_f32(600.00006),
            a_max: math::Real::from_f32(3600.0002),
            j_max: math::Real::from_f32(7200.0005),
        };
        // Exact values from the error log.
        // v_i ≈ v_o ≈ v_max, s tiny, a_o slightly non-zero.
        // Before the Vieta fix: total_time collapsed to 0 → NumericalError.
        let result = HybridMotionProfile::compute(
            math::Real::from_f32(0.093312435),
            math::Real::from_f32(600.00006),
            math::Real::from_f32(599.9999),
            math::Real::from_f32(0.0),
            math::Real::from_f32(-0.4061374),
            &constraints,
            false,
        );
        assert!(
            result.is_ok(),
            "Quintic compute must succeed for near-cruise segment, got: {:?}",
            result.err()
        );
        let profile = result.unwrap();
        // Time must be positive and physically sensible (~s/v ≈ 0.000155 s).
        assert!(
            profile.end_time() > math::Real::from_f32(0.0),
            "end_time must be positive, got {:?}",
            profile.end_time()
        );
        assert!(
            (profile.end_time() - math::Real::from_f32(0.000155)).abs()
                < math::Real::from_f32(1e-4),
            "end_time should be ~0.000155 s, got {:?}",
            profile.end_time()
        );
    }
}
