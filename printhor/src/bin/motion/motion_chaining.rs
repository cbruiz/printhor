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


/// Boundary accelerations below this fraction of `a_max` are routed to the S-Curve
/// (same threshold as `HybridMotionProfile`).
#[inline]
fn accel_negligible(a: Real, a_max: Real) -> bool {
    a.abs() <= a_max * Real::from_f32(1.0e-4)
}


/// A segment is ramp-limited when its velocity change needs at least 90% of its displacement.
fn is_ramp_limited(seg: &hwa::controllers::Segment) -> bool {
    let d_min = crate::motion::scurve_min_distance(
        seg.speed_enter_su_s,
        seg.speed_exit_su_s,
        &seg.constraints,
    );
    d_min >= seg.displacement_su * Real::from_f32(0.9)
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
fn segment_admissible(
    s: Real,
    v_i: Real,
    v_o: Real,
    a_i: Real,
    a_o: Real,
    a_max: Real,
    tol: Real,
) -> bool {
    if !(s > math::ZERO)
        || (v_i + v_o).is_negligible()
        || (accel_negligible(a_i, a_max) && accel_negligible(a_o, a_max))
    {
        return true;
    }
    if !(tol > math::ZERO) {
        return crate::motion::blend_is_admissible(s, v_i, v_o, a_i, a_o);
    }
    s > tol * math::TWO
        && crate::motion::blend_is_admissible(s + tol, v_i, v_o, a_i, a_o)
        && crate::motion::blend_is_admissible(s, v_i, v_o, a_i, a_o)
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
        let f = ((j6 * t + a2) * t + v_i) * t - s;
        let df = (j2 * t + a_i) * t + v_i;
        if !(df > math::ZERO) {
            break;
        }
        let dt = f / df;
        t = t - dt;
        if !(dt > t * math::EPSILON * math::FOUR) {
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

/// Braking floor: the lowest exit speed a segment entering at `v_i` can reach over its
/// displacement `s`, braking as hard as its constraints allow, with exit acceleration `a_o`.
///
/// It is the smallest `v_o` whose braking limit (`max_speed_pass(s, v_o, -a_o)`, the highest entry
/// speed from which `v_o` can be reached) is at least `v_i`. The braking limit grows with `v_o`, so
/// it is found by bisection. Any exit at or above the floor (and reachable, see
/// `speed_exit_constrained_su_s`) is feasible from that entry.
///
/// Used for the oldest planned segment, whose entry is pinned (see `perform_motion_chaining`):
/// its exit must never be planned below the floor, as its entry can no longer be lowered.
fn braking_floor(
    s: Real,
    v_i: Real,
    a_o: Real,
    constraints: &crate::motion::Constraints,
) -> Real {
    let braking_limit = |v_o: Real| max_speed_pass(s, v_o, -a_o, constraints).3;
    if !(s > math::ZERO) || !(v_i > math::ZERO) || braking_limit(math::ZERO) >= v_i {
        return math::ZERO;
    }
    if braking_limit(v_i) < v_i {
        // Not even keeping the entry speed is feasible with that exit acceleration
        return v_i;
    }
    let (mut lo, mut hi) = (math::ZERO, v_i);
    for _ in 0..32 {
        let mid = (lo + hi) * HALF;
        if braking_limit(mid) >= v_i {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

/// `true` when a segment entering at `v_i` can brake to the exit state (`v_o`, `a_o`) within its
/// displacement. The braking limit is not monotone in `a_o` (S-Curve ramps), so every exit
/// acceleration assigned to a segment with a fixed entry has to be checked.
fn can_brake(seg: &hwa::controllers::Segment, v_i: Real, v_o: Real, a_o: Real) -> bool {
    if seg.displacement_su.is_negligible() {
        return true;
    }
    let (_, _, _, v_limit, _) = max_speed_pass(seg.displacement_su, v_o, -a_o, &seg.constraints);
    v_i <= v_limit
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

    let (t1, t2, t3, v_exit_constr, a_exit_constr) = max_speed_pass(
        curr_segment.displacement_su,
        curr_segment.speed_enter_su_s,
        curr_segment.accel_enter_su_s,
        &curr_segment.constraints,
    );
    curr_segment.speed_exit_constrained_su_s = v_exit_constr;
    curr_segment.t_1 = t1;
    curr_segment.t_2 = t2;
    curr_segment.t_3 = t3;

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

/// Performs the motion chaining (look-ahead) algorithm on the ring buffer.
///
/// Uses a classic bidirectional planning approach:
///
/// 1. **Backward pass** (newest → oldest): propagates the boundary condition that
///    the last segment must slow down (its exit was initialised to `min_speed`).
///    Each older segment's exit is reduced so the newer segment can physically
///    brake from that speed to its own constrained exit speed.
///    Corner geometry (the `proj_prev` projection) further reduces the junction
///    at non-collinear transitions.
///
/// 2. **Forward pass** (oldest → newest): propagates the boundary condition that
///    the oldest segment starts from rest (its enter was initialised to `min_speed`).
///    For each junction the speed is set to the minimum of what the backward pass
///    allowed and what the forward kinematics can actually achieve from the entry.
///    This pass CAN raise junction speeds on long straight chains.
///
/// 3. **Finalization**: copies the committed exit speed and acceleration from each
///    older segment onto the entry of the adjacent newer segment to guarantee
///    strict continuity.
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
    let tol = displacement_tolerance.max(Real::zero());
    let used = rb.used;
    if used < 2 {
        return Ok(());
    }

    // Count consecutive PlannedMove segments from the tail (newest first).
    // Stop at the first non-PlannedMove entry — SetPosition, Homing, Dwell, Executing,
    // or Empty entries act as a chain boundary and must not be included in the chaining window.
    // Using rb.used directly would cause the backward pass to access tail(used) which may be
    // a SetPosition (e.g. from G92 preceding the moves), returning Err(()) and aborting silently.
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

    // The entry state of the oldest planned segment is *pinned*: it is either the rest state it
    // was created with, or the exit state of the segment the stepper is executing right now
    // (which can no longer change). Modifying it would create a velocity/acceleration
    // discontinuity at that junction, so it is restored after every pass that may touch it.
    let (pinned_v_enter, pinned_a_enter) = {
        let oldest = rb.planned_segment_from_tail(used)?;
        (oldest.speed_enter_su_s, oldest.accel_enter_su_s)
    };

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!("[motion_chaining] Cornering algorithm START. Segments: {}", used);

    // ── Backward pass ────────────────────────────────────────────────────────
    // tail(1) = newest (exit already set to min_speed at segment creation).
    // tail(used) = oldest.
    // For each pair (older = tail(i+1), newer = tail(i)):
    //   • recompute newer's constraints so speed_enter_constrained reflects
    //     the maximum entry speed from which it can brake to its exit speed.
    //   • reduce older's exit to stay within that limit, scaled by the corner
    //     projection factor.
    for i in 1..used {
        // Recompute newer's constraints with its current committed speeds.
        {
            let newer_seg = rb.mut_planned_segment_from_tail(i)?;
            compute_maximum_enter_and_exit_speed(false, i as u32, newer_seg);
        }
        // A junction can not exceed the nominal (target) speed of either side: entering the newer
        // segment above its own target would require braking inside it that its displacement may
        // not allow (and, once it becomes the pinned oldest segment, can no longer be corrected).
        let newer_enter_constrained = {
            let newer = rb.planned_segment_from_tail(i)?;
            newer.speed_enter_constrained_su_s.min(newer.speed_target_su_s)
        };
        let proj = rb.planned_segment_from_tail(i)?.proj_prev;

        let older_seg = rb.mut_planned_segment_from_tail(i + 1)?;

        // Backward constraint: compute the maximum exit speed the older segment
        // is allowed, given what the newer segment can kinematically accept at
        // its entry (braking from that speed to newer's exit in its distance),
        // scaled by the corner projection factor (proj≈1 collinear, proj=0 at
        // 90° corner).
        // Always apply unconditionally — previous incremental passes may have
        // over-constrained middle segments; with more context now available we
        // need to be able to raise them as well as lower them.
        let new_exit = newer_enter_constrained
            .min(older_seg.speed_target_su_s)
            * proj.max(Real::zero());
        // The pinned (oldest) segment can not brake below its floor
        let new_exit = if i + 1 == used {
            new_exit.max(braking_floor(
                older_seg.displacement_su,
                pinned_v_enter,
                math::ZERO,
                &older_seg.constraints,
            ))
        } else {
            new_exit
        };

        older_seg.speed_exit_su_s = new_exit;
        compute_maximum_enter_and_exit_speed(false, (i + 1) as u32, older_seg);
    }

    // ── Forward pass ─────────────────────────────────────────────────────────
    // Iterate from oldest (tail(used)) toward newest (tail(1)).
    // The oldest segment's entry was initialised to min_speed (start from rest).
    // At each junction we propagate the maximum speed the older segment can
    // actually reach at its exit (forward kinematics from its entry), bounded
    // by the backward-pass exit already stored in speed_exit_su_s, and apply
    // the corner projection.
    //
    // Note: `newer_entry_limit` (speed_enter_constrained of the newer segment)
    // was computed during the backward pass based on newer's BACKWARD exit speed.
    // The forward pass may lower newer's exit further (at a later iteration i−1),
    // which means the actual physical braking limit can be smaller than
    // newer_entry_limit.  The Correction Sweep below fixes any residual
    // violations after all passes are done.
    for i in (1..used).rev() {
        // Recompute older's constraints with its current committed enter speed.
        {
            let older_seg = rb.mut_planned_segment_from_tail(i + 1)?;
            compute_maximum_enter_and_exit_speed(false, (i + 1) as u32, older_seg);
        }
        let older_exit_su_s =
            rb.planned_segment_from_tail(i + 1)?.speed_exit_su_s;
        let older_exit_constrained =
            rb.planned_segment_from_tail(i + 1)?.speed_exit_constrained_su_s;
        let proj = rb.planned_segment_from_tail(i)?.proj_prev;
        // Physical braking limit for the newer segment given its
        // backward-constrained exit speed (unchanged since the backward pass).
        // (and its target speed, see the backward pass).
        let newer_entry_limit = {
            let newer = rb.planned_segment_from_tail(i)?;
            newer.speed_enter_constrained_su_s.min(newer.speed_target_su_s)
        };

        // Junction = min(backward limit, forward achievable) × corner factor,
        // then clamped to the newer segment's physical braking limit.
        let junction = (older_exit_constrained.min(older_exit_su_s) * proj.max(Real::zero()))
            .min(newer_entry_limit);
        // ...but never below the braking floor of the pinned (oldest) segment: its entry can not
        // be lowered. If that exceeds what the newer segment accepts, the newer one brakes more
        // (its exit is raised by the feasibility propagation after the correction sweep).
        let junction = if i + 1 == used {
            let older = rb.planned_segment_from_tail(i + 1)?;
            junction.max(braking_floor(
                older.displacement_su,
                pinned_v_enter,
                math::ZERO,
                &older.constraints,
            ))
        } else {
            junction
        };

        // Commit the junction on both sides and recompute newer's constraints.
        {
            let older_seg = rb.mut_planned_segment_from_tail(i + 1)?;
            older_seg.speed_exit_su_s = junction;
        }
        {
            let newer_seg = rb.mut_planned_segment_from_tail(i)?;
            newer_seg.speed_enter_su_s = junction;
            compute_maximum_enter_and_exit_speed(false, i as u32, newer_seg);
        }
    }

    // ── Finalization ─────────────────────────────────────────────────────────
    // Guarantee strict continuity: newer.enter == older.exit for both speed
    // and acceleration (needed for the quintic polynomial boundary conditions).
    for i in 1..used {
        let (older_exit, older_accel_exit) = {
            let older = rb.planned_segment_from_tail(i + 1)?;
            (older.speed_exit_su_s, older.accel_exit_su_s)
        };
        let newer = rb.mut_planned_segment_from_tail(i)?;
        newer.speed_enter_su_s = older_exit;
        newer.accel_enter_su_s = older_accel_exit;
    }

    // ── Global Boundary Conditions ───────────────────────────────────────────
    // Restore the pinned entry state of the oldest segment (rest, or the exit state of the
    // segment being executed) and enforce zero acceleration at the end of the last segment.
    // After recomputing each boundary segment the updated acceleration at the interior edge must be
    // propagated to the adjacent segment so that continuity (a_o[n] == a_i[n+1]) is preserved.
    {
        let oldest_seg = rb.mut_planned_segment_from_tail(used)?;
        oldest_seg.speed_enter_su_s = pinned_v_enter;
        oldest_seg.accel_enter_su_s = pinned_a_enter;
        compute_maximum_enter_and_exit_speed(false, used as u32, oldest_seg);
        oldest_seg.speed_enter_su_s = pinned_v_enter;
        oldest_seg.accel_enter_su_s = pinned_a_enter;
    }
    // Propagate the (possibly updated) a_exit of the oldest segment to the next segment's a_enter.
    if used >= 2 {
        let oldest_accel_exit = rb.planned_segment_from_tail(used)?.accel_exit_su_s;
        let second_oldest = rb.mut_planned_segment_from_tail(used - 1)?;
        second_oldest.accel_enter_su_s = oldest_accel_exit;
    }
    {
        let newest_seg = rb.mut_planned_segment_from_tail(1)?;
        newest_seg.accel_exit_su_s = Real::zero();
        compute_maximum_enter_and_exit_speed(false, 1, newest_seg);
        newest_seg.accel_exit_su_s = Real::zero();
    }
    // Propagate the recomputed a_enter of the newest segment back to the previous segment's a_exit.
    // Without this, the independent backward pass overwrites a_enter with a value inconsistent with
    // what Finalization copied from the previous segment, breaking the a_o[n] == a_i[n+1] invariant.
    if used >= 2 {
        let newest_accel_enter = rb.planned_segment_from_tail(1)?.accel_enter_su_s;
        let second_newest = rb.mut_planned_segment_from_tail(2)?;
        second_newest.accel_exit_su_s = newest_accel_enter;
    }

    // ── Correction Sweep ─────────────────────────────────────────────────────
    // The forward pass sets each segment's entry speed based on the backward-
    // pass exit limit for that segment.  However, the forward pass also lowers
    // segment exits (when forward kinematics are the binding constraint), and
    // that lowering happens AFTER the entry is already committed.  A lower exit
    // requires more braking, so the actual maximum feasible entry (vcin) shrinks
    // below the value that was used as the clamp — leaving vin > vcin.
    //
    // Fix: sweep newest → oldest, recompute vcin from the FINAL committed exit,
    // and clamp vin when the violation is found.  Propagate the clamped vin
    // backward to the older segment's exit to maintain the continuity invariant
    // (older.speed_exit == newer.speed_enter for every adjacent pair).
    for i in 1..=used {
        let (v_o, a_o, s) = {
            let seg = rb.planned_segment_from_tail(i)?;
            (seg.speed_exit_su_s, seg.accel_exit_su_s, seg.displacement_su)
        };
        if s.is_negligible() {
            continue;
        }
        // Feasible with the current exit acceleration and with zero: the passes below keep it only
        // when it is feasible, and replace it by zero otherwise (see `can_brake`)
        let v_entry_constr = {
            let seg = rb.planned_segment_from_tail(i)?;
            let (_, _, _, v, _) = max_speed_pass(s, v_o, -a_o, &seg.constraints);
            let (_, _, _, v0, _) = max_speed_pass(s, v_o, math::ZERO, &seg.constraints);
            v.min(v0)
        };
        rb.mut_planned_segment_from_tail(i)?.speed_enter_constrained_su_s = v_entry_constr;
        let v_i = rb.planned_segment_from_tail(i)?.speed_enter_su_s;
        if v_i > v_entry_constr {
            if i == used {
                // Pinned entry: lowering it would create a velocity discontinuity with the
                // segment being executed. Its exit is raised to the braking floor instead
                // (handled with the feasibility propagation below).
                continue;
            }
            // Never lower the exit of the pinned segment below its braking floor
            let v_new = if i + 1 == used {
                let older = rb.planned_segment_from_tail(i + 1)?;
                v_entry_constr.max(braking_floor(
                    older.displacement_su,
                    pinned_v_enter,
                    math::ZERO,
                    &older.constraints,
                ))
            } else {
                v_entry_constr
            };
            rb.mut_planned_segment_from_tail(i)?.speed_enter_su_s = v_new;
            // Propagate to the older segment's exit so continuity is maintained.
            rb.mut_planned_segment_from_tail(i + 1)?.speed_exit_su_s = v_new;
        }
    }

    // ── Feasibility Propagation (braking floor) ──────────────────────────────
    // The sweep above can only lower entries, so it can not fix a segment whose entry is fixed:
    // the pinned oldest one, or one whose entry the floor below just raised. Going oldest →
    // newest, every segment that can not brake from its entry to its exit gets its exit raised to
    // its braking floor (with zero exit acceleration, so the floor holds whatever the junction
    // acceleration pass would assign), which becomes the entry of the next one. The exit of the
    // newest segment (the stop at the end of the plan) is never raised: it should not be needed,
    // as the plan that pinned the entry could already stop there.
    for i in (1..=used).rev() {
        let (s, v_i, v_o, a_o, v_entry_constr) = {
            let seg = rb.planned_segment_from_tail(i)?;
            // Feasible with zero exit acceleration: the passes below may replace the current one
            // by zero, and only keep a non-zero one that is also feasible (see `can_brake`)
            let (_, _, _, v, _) = max_speed_pass(
                seg.displacement_su,
                seg.speed_exit_su_s,
                math::ZERO,
                &seg.constraints,
            );
            (seg.displacement_su, seg.speed_enter_su_s, seg.speed_exit_su_s, seg.accel_exit_su_s, v)
        };
        if s.is_negligible() || !(v_i > v_entry_constr) {
            continue;
        }
        if i == 1 {
            hwa::warn!(
                "[motion_chaining] Newest segment can not brake from {:?} to {:?} (a_o={:?})",
                v_i,
                v_o,
                a_o
            );
            continue;
        }
        let floor = {
            let seg = rb.planned_segment_from_tail(i)?;
            braking_floor(s, v_i, math::ZERO, &seg.constraints)
        };
        {
            let seg = rb.mut_planned_segment_from_tail(i)?;
            seg.speed_exit_su_s = floor;
            seg.accel_exit_su_s = math::ZERO;
            seg.speed_enter_constrained_su_s = v_i;
        }
        let newer = rb.mut_planned_segment_from_tail(i - 1)?;
        newer.speed_enter_su_s = floor;
        newer.accel_enter_su_s = math::ZERO;
    }

    // ── Junction Acceleration Pass ───────────────────────────────────────────
    // With the speeds final, assign every interior junction the acceleration implied by the
    // velocity profile itself: a = v·dv/ds, with dv/ds estimated by the monotonicity-preserving
    // PCHIP formula (Fritsch–Butland harmonic mean with Brodlie weights) over the two adjacent
    // segments. It is zero at every local velocity extremum (cruise plateau, peak, full stop),
    // has the sign of the velocity trend and is bounded by 3× the smallest adjacent secant.
    //
    // The accelerations left by the passes above are "maximum reachable" values and frequently
    // had the opposite sign of the velocity change at the junction, which forced the blend
    // polynomials into large jerk excursions (up to 5×j_max on a tessellated curve, vs. ≈2× with
    // this estimate; the rest comes from the junction speeds themselves).
    //
    // A non-zero junction acceleration is only assigned when *both* adjacent segments are
    // ramp-limited (see `scurve_min_distance`). Otherwise the segment with slack peaks or cruises
    // inside (S-Curve), the junction is a velocity extremum of the executed profile, and forcing a
    // polynomial blend there would only slow the segment down (a blend cannot cruise at v_max).
    //
    // Both sides of each junction get the same value, so a_o[n] == a_i[n+1] holds by construction.
    // The pinned entry of the oldest segment and the zero exit of the newest are not touched.
    for i in 1..used {
        let (s1, v0, v1, a_max1, tight1) = {
            let older = rb.planned_segment_from_tail(i + 1)?;
            (
                older.displacement_su,
                older.speed_enter_su_s,
                older.speed_exit_su_s,
                older.constraints.a_max,
                is_ramp_limited(older),
            )
        };
        let (s2, v2, a_max2, tight2) = {
            let newer = rb.planned_segment_from_tail(i)?;
            (
                newer.displacement_su,
                newer.speed_exit_su_s,
                newer.constraints.a_max,
                is_ramp_limited(newer),
            )
        };
        let a_junction = if tight1 && tight2 && s1 > math::ZERO && s2 > math::ZERO {
            let m1 = (v1 - v0) / s1;
            let m2 = (v2 - v1) / s2;
            if m1 * m2 > math::ZERO {
                let w1 = math::TWO * s2 + s1;
                let w2 = s2 + math::TWO * s1;
                let slope = (w1 + w2) / (w1 / m1 + w2 / m2);
                let a_lim = a_max1.min(a_max2);
                (v1 * slope).clamp(-a_lim, a_lim)
            } else {
                Real::zero()
            }
        } else {
            Real::zero()
        };
        // The junction acceleration changes the braking ability of the older segment: if it could
        // no longer brake from its entry to the junction speed with it, keep it zero (the
        // correction sweep and the feasibility propagation guaranteed the zero acceleration case)
        let a_junction = if !a_junction.is_zero() {
            let older = rb.planned_segment_from_tail(i + 1)?;
            if can_brake(older, v0, v1, a_junction) { a_junction } else { Real::zero() }
        } else {
            a_junction
        };
        rb.mut_planned_segment_from_tail(i + 1)?.accel_exit_su_s = a_junction;
        rb.mut_planned_segment_from_tail(i)?.accel_enter_su_s = a_junction;
    }

    // ── Blend Admissibility Pass ─────────────────────────────────────────────
    // Runs last, once speeds and junction accelerations are final. Every segment with non-zero
    // boundary accelerations is executed as a polynomial blend; guarantee *here* (with the same
    // predicate the profile uses) that the blend exists and never reverses its velocity, so that
    // `HybridMotionProfile::compute` never needs to alter the boundary conditions — which would
    // break the a_o[n] == a_i[n+1] invariant (acceleration discontinuity).
    //
    // Repair strategy: shrink the *free* boundary accelerations of every non-admissible segment
    // towards zero (the pinned entry of the oldest segment and the zero exit of the newest are
    // never touched), always writing the same value on both sides of a junction. Zero boundary
    // accelerations are always admissible (the blend velocity is then a monotone smoothstep
    // between v_i and v_o), which is the final fallback.
    let mut plan_admissible = false;
    for _round in 0..4 {
        let mut all_ok = true;
        for i in (1..=used).rev() {
            let (v_i, v_o, a_i, a_o, s, a_max) = {
                let seg = rb.planned_segment_from_tail(i)?;
                (
                    seg.speed_enter_su_s,
                    seg.speed_exit_su_s,
                    seg.accel_enter_su_s,
                    seg.accel_exit_su_s,
                    seg.displacement_su,
                    seg.constraints.a_max,
                )
            };
            if segment_admissible(s, v_i, v_o, a_i, a_o, a_max, tol) {
                continue;
            }
            let entry_free = i < used;
            let exit_free = i > 1;
            // Segments dominated by the step quantization go straight to zero accelerations
            // (halving would never converge for them).
            let quantized = !(s > tol * math::TWO);
            let a_o_new = if exit_free {
                let cap = admissible_exit_accel_cap(s, v_i, v_o, a_i, tol);
                Some(if quantized {
                    Real::zero().min(cap)
                } else if a_o > cap {
                    cap
                } else {
                    a_o * HALF
                })
            } else {
                None
            };
            // A pinned segment (entry = exit state of the segment being executed) whose exit
            // acceleration is already zero is left as is, even when its blend is not admissible:
            // moving that exit below the feasibility cap would push a non-zero (decelerating)
            // acceleration into the next segment, which then usually has to accelerate and cannot
            // be repaired either (checked with `test_chaining_robust_to_displacement_correction`).
            // The stepper caps it instead: a small, bounded acceleration step at that junction.
            if !entry_free && (a_o.is_zero() || a_o_new.is_none_or(|a| a == a_o)) {
                // Only the pinned entry acceleration is left: nothing else can be done here.
                #[cfg(feature = "debug-motion-planning")]
                hwa::warn!("[motion_chaining] Pinned segment not admissible (a_i={:?})", a_i);
                continue;
            }
            all_ok = false;
            // Every segment must still be able to brake with its new exit acceleration
            // (zero always can, see the correction sweep and the feasibility propagation)
            let a_o_new = match a_o_new {
                Some(a) => {
                    let seg = rb.planned_segment_from_tail(i)?;
                    Some(if can_brake(seg, v_i, v_o, a) { a } else { Real::zero() })
                }
                None => None,
            };
            if let Some(a_o_new) = a_o_new {
                rb.mut_planned_segment_from_tail(i)?.accel_exit_su_s = a_o_new;
                rb.mut_planned_segment_from_tail(i - 1)?.accel_enter_su_s = a_o_new;
            }
            if entry_free {
                let a_i_new = if quantized { Real::zero() } else { a_i * HALF };
                // It is the exit acceleration of the older segment: it must still brake with it
                let a_i_new = {
                    let older = rb.planned_segment_from_tail(i + 1)?;
                    if can_brake(older, older.speed_enter_su_s, older.speed_exit_su_s, a_i_new) {
                        a_i_new
                    } else {
                        Real::zero()
                    }
                };
                rb.mut_planned_segment_from_tail(i)?.accel_enter_su_s = a_i_new;
                rb.mut_planned_segment_from_tail(i + 1)?.accel_exit_su_s = a_i_new;
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
        for i in 1..used {
            rb.mut_planned_segment_from_tail(i + 1)?.accel_exit_su_s = Real::zero();
            rb.mut_planned_segment_from_tail(i)?.accel_enter_su_s = Real::zero();
        }
    }

    // The pinned segment must be able to brake from its (fixed) entry to its final exit state
    #[cfg(feature = "assert-motion")]
    {
        let oldest = rb.planned_segment_from_tail(used)?;
        let (_, _, _, v_limit, _) = max_speed_pass(
            oldest.displacement_su,
            oldest.speed_exit_su_s,
            -oldest.accel_exit_su_s,
            &oldest.constraints,
        );
        if !oldest.displacement_su.is_negligible()
            && oldest.speed_enter_su_s > v_limit + v_limit * Real::from_f32(1.0e-4)
        {
            hwa::warn!(
                "[motion_chaining] Pinned entry speed {:?} exceeds the braking limit {:?}",
                oldest.speed_enter_su_s,
                v_limit
            );
        }
    }

    #[cfg(feature = "debug-motion-planning")]
    hwa::info!("[motion_chaining] Cornering algorithm END");
    Ok(())
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
            t_1: Real::from_f32(0.0),
            t_2: Real::from_f32(0.0),
            t_3: Real::from_f32(0.0),
            proj_prev: Real::from_f32(1.0),
            proj_next: Real::from_f32(1.0),
            unit_vector_dir: Default::default(),
            src_pos: Default::default(),
            dest_pos: Default::default(),
            dest_world_pos: Default::default(),
            tool_power: Default::default(),
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
