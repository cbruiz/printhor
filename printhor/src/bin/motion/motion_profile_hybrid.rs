//! Hybrid motion profile that selects the optimal trajectory algorithm for each
//! segment based on kinematic feasibility and boundary conditions.
//!
//! Two algorithms are available and dispatched by [`HybridMotionProfile::compute`]:
//!
//! - **Double S-Curve Velocity Profile** ([`ProfileKind::SCurve`]) — a 7-phase, jerk-limited
//!   piecewise profile with an optional cruise phase.  Used when both boundary accelerations are
//!   (numerically) zero and the displacement can accommodate the requested velocity change.
//!   Based on §3.4 of \[1\], but the peak velocity is found with an exact, bounded
//!   (safeguarded Newton) solver instead of the iterative `a_max ← γ·a_max` reduction of the book:
//!   the result is time-optimal for the given limits and takes a handful of iterations.
//!
//! - **Polynomial Hermite Blend** ([`ProfileKind::Polynomial`]) — the velocity is the
//!   quintic Hermite interpolant of `(v, a, j=0)` at both ends (position is a degree-6
//!   polynomial).  Used when motion-chaining imposes non-zero boundary accelerations that must be
//!   exactly matched, or as graceful degradation when the S-Curve is infeasible.
//!
//! ## Smoothness guarantees
//!
//! * Velocity and acceleration are continuous everywhere inside a profile, and the boundary
//!   values `(v_i, a_i)` / `(v_o, a_o)` are reproduced exactly, so chaining profiles whose
//!   boundary conditions match yields continuous acceleration across junctions.
//! * With [`JerkShape::Smooth`] (the default) the jerk is also continuous and **zero at every
//!   phase boundary and at both ends of every profile** (S-Curve and Polynomial alike):
//!   the acceleration is therefore continuous **and differentiable** (C¹) along the whole path.
//! * [`JerkShape::Linear`] reproduces the classic piecewise-constant-jerk double S
//!   (acceleration C⁰ only).
//!
//! Both paths implement the same [`motion::MotionProfile`] trait so that the stepper
//! task and segment sampler are unaware of which algorithm was chosen.
//!
//! \[1\] Biagiotti, L., Melchiorri, C.: *Trajectory Planning for Automatic Machines and Robots*.
//! Springer, Heidelberg (2008). [DOI:10.1007/978-3-540-85629-0](https://doi.org/10.1007/978-3-540-85629-0)

use crate::hwa;
use crate::motion;
use crate::processing;
use hwa::math;
use math::Real;

// ─────────────────────────────────────────────────────────────────────────────
// Tunables
// ─────────────────────────────────────────────────────────────────────────────

/// Shape of the jerk during the jerk-limited phases (1, 3, 5, 7) of the S-Curve.
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum JerkShape {
    /// Piecewise-constant jerk (classic double S).
    /// Acceleration is continuous (C⁰) but not differentiable at the phase boundaries.
    Linear,
    /// Polynomial acceleration ramps `a(τ) = A·(3τ² − 2τ³)` ("smoothstep").
    ///
    /// The ramp has the same velocity increment (`∫ = ½`) as a linear ramp and, over each
    /// up/down ramp pair, the same displacement, so phase durations are computed with the
    /// classic closed forms. The jerk `6·A·τ(1−τ)/T_j` is continuous and vanishes at both ends of
    /// every ramp → acceleration is C¹. Its peak is `1.5·A/T_j`, hence the profile is planned
    /// with `j = ⅔·j_max` (see [`effective_jerk`]) so that the real peak jerk equals `j_max`.
    Smooth,
}

/// Jerk shape used by [`HybridMotionProfile::compute`] and by the motion-chaining planner.
pub const DEFAULT_JERK_SHAPE: JerkShape = JerkShape::Smooth;

/// The jerk value the S-Curve timing must be planned with so that the evaluated profile
/// never exceeds `j_max` for the given [`JerkShape`].
///
/// The planner (motion chaining) and the profile must use the same value, otherwise the
/// planner would request velocity changes that the profile cannot perform in the available
/// displacement.
#[inline]
pub fn effective_jerk(j_max: Real, shape: JerkShape) -> Real {
    match shape {
        JerkShape::Linear => j_max,
        JerkShape::Smooth => j_max * math::TWO_THIRDS,
    }
}

/// Relative tolerance used for *decisions* (routing / feasibility), so that results that differ
/// only by rounding are treated consistently. ~80 ulp in f32, far below a micro-step.
#[inline]
fn rel_tol() -> Real {
    Real::from_f32(1.0e-5)
}

/// Boundary accelerations below this fraction of `a_max` are considered zero.
#[inline]
fn accel_bc_tol(a_max: Real) -> Real {
    a_max * Real::from_f32(1.0e-4)
}

/// Upper bound of iterations of the peak-velocity solver (safeguarded Newton converges in 3-8).
const PEAK_VELOCITY_MAX_ITER: u32 = 48;

/// Number of intervals used to check that a polynomial blend never reverses its velocity.
const BLEND_ADMISSIBILITY_SAMPLES: i64 = 16;

// ─────────────────────────────────────────────────────────────────────────────
// Public data types
// ─────────────────────────────────────────────────────────────────────────────

#[derive(PartialEq, Debug, Copy, Clone)]
pub enum ProfileKind {
    /// Double S-Curve (7 phases, zero boundary accelerations)
    SCurve,
    /// Polynomial Hermite blend matching `(v, a)` at both ends with zero boundary jerk.
    Polynomial,
}

/// Struct representing the motion profile configuration.
///
/// # Fields
///
/// * `constraints` - The motion profile constraints.
/// * `times` - The time parameters for different phases of the motion profile.
/// * `initial_velocity` - Initial velocity for the motion profile (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
/// * `final_velocity` - Final velocity for the motion profile (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
/// * `error_correction` - An optional error correction factor.
pub struct ProfileConfig {
    /// The motion profile constraints.
    pub constraints: Constraints,
    /// The time parameters of the motion profile.
    pub times: Times,
    /// Initial velocity for the motion profile.
    pub initial_velocity: math::Real,
    /// Final velocity for the motion profile.
    pub final_velocity: math::Real,
    /// An optional error correction factor.
    pub error_correction: Option<math::Real>,
}

/// The `Constraints` struct represents the limits of the motion profile in terms of velocity, acceleration, and jerk.
///
/// * `v_max` - The maximum velocity (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
/// * `a_max` - The maximum acceleration (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²).
/// * `j_max` - The maximum jerk (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s³).
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub struct Constraints {
    /// The maximum velocity (v_max) (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
    pub v_max: math::Real,
    /// The maximum acceleration (a_max) (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²).
    pub a_max: math::Real,
    /// The maximum jerk (j_max) (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s³).
    pub j_max: math::Real,
}

impl core::fmt::Debug for Constraints {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "v_{{max}}={:?} a_{{max}}={:?} j_{{max}}={:?}",
            self.v_max,
            self.a_max,
            self.j_max
        )
    }
}

/// Cumulative phase-boundary timestamps.
///
/// Each field stores the absolute time (in seconds from motion start) at which
/// the profile transitions from one interval to the next.  For the Polynomial path
/// all seven fields are set to the same value `T` (the total duration), so that
/// interval-boundary queries degenerate gracefully.
///
/// | Interval | Phase |
/// |---|---|
/// | 1 `[0, t_2s]` | Acceleration — increasing acceleration |
/// | 2 `[t_2s, t_3s]` | Acceleration — constant acceleration |
/// | 3 `[t_3s, t_4s]` | Acceleration — decreasing acceleration |
/// | 4 `[t_4s, t_5s]` | Cruise at `v_lim` |
/// | 5 `[t_5s, t_6s]` | Deceleration — increasing deceleration |
/// | 6 `[t_6s, t_7s]` | Deceleration — constant deceleration |
/// | 7 `[t_7s, t_8s]` | Deceleration — decreasing deceleration back to `v_o` |
#[derive(PartialEq, Copy, Clone)]
pub struct Times {
    pub t_2s: math::Real,
    pub t_3s: math::Real,
    pub t_4s: math::Real,
    pub t_5s: math::Real,
    pub t_6s: math::Real,
    pub t_7s: math::Real,
    pub t_8s: math::Real,
}

impl Times {
    pub const fn new(
        t_2s: math::Real,
        t_3s: math::Real,
        t_4s: math::Real,
        t_5s: math::Real,
        t_6s: math::Real,
        t_7s: math::Real,
        t_8s: math::Real,
    ) -> Self {
        Self {
            t_2s,
            t_3s,
            t_4s,
            t_5s,
            t_6s,
            t_7s,
            t_8s,
        }
    }

    /// Time spent in the first jerk phase (in seconds).
    #[inline]
    pub fn t_j1(&self) -> math::Real {
        self.t_2s
    }
    #[inline]
    pub fn t_a(&self) -> math::Real {
        self.t_4s
    }
    #[inline]
    pub fn t_v(&self) -> math::Real {
        self.t_5s - self.t_4s
    }
    #[inline]
    pub fn t_j2(&self) -> math::Real {
        self.t_8s - self.t_7s
    }
    #[inline]
    pub fn t_d(&self) -> math::Real {
        self.t_8s - self.t_5s
    }

    /// Start time of the given phase (1..=7)
    #[inline]
    fn phase_start(&self, phase: usize) -> Real {
        match phase {
            1 => math::ZERO,
            2 => self.t_2s,
            3 => self.t_3s,
            4 => self.t_4s,
            5 => self.t_5s,
            6 => self.t_6s,
            _ => self.t_7s,
        }
    }

    /// Phase (1..=7) containing `t`, assuming `0 < t < t_8s`
    #[inline]
    fn phase_of(&self, t: Real) -> usize {
        if t < self.t_2s {
            1
        } else if t < self.t_3s {
            2
        } else if t < self.t_4s {
            3
        } else if t < self.t_5s {
            4
        } else if t < self.t_6s {
            5
        } else if t < self.t_7s {
            6
        } else {
            7
        }
    }
}

impl core::fmt::Debug for Times {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "t_j1={:?} t_a={:?} t_v={:?} t_j2={:?} t_d={:?} T={:?}",
            self.t_j1(),
            self.t_a(),
            self.t_v(),
            self.t_j2(),
            self.t_d(),
            self.t_8s
        )
    }
}

/// Pre-computed values that keep the per-sample evaluation cheap (no divisions, no sqrt).
#[derive(Copy, Clone, PartialEq, Default)]
pub struct Cache {
    /// S-Curve: velocity at the start of each phase (`v[k]` = start of phase `k+1`, `v[7]` = end)
    pub v: [Real; 8],
    /// S-Curve: position at the start of each phase (`s[k]` = start of phase `k+1`, `s[7]` = end)
    pub s: [Real; 8],
    /// S-Curve: `1/T_j1` (zero when the phase is empty)
    pub inv_tj1: Real,
    /// S-Curve: `1/T_j2` (zero when the phase is empty)
    pub inv_tj2: Real,
    /// Polynomial: `1/T`
    pub inv_t: Real,
    /// Polynomial: velocity coefficients `b_n` of `v(τ) = Σ b_n·τⁿ` with `τ = t/T` (`b_2 == 0`)
    pub poly_v: [Real; 6],
    /// Polynomial: position coefficients `p_n` of `s(τ) = Σ p_n·τⁿ` (`p_0 == p_3 == 0`), in space units
    pub poly_s: [Real; 7],
    /// Polynomial: acceleration coefficients `[b1, 3·b3, 4·b4, 5·b5] / T`
    pub poly_a: [Real; 4],
}

/// Hybrid motion profile that covers a scalar displacement `s_o` connecting two
/// kinematic boundary states `(v_i, a_i)` → `(v_o, a_o)`.
#[derive(PartialEq, Clone, Copy)]
pub struct HybridMotionProfile {
    /// The algorithm used for this profile
    pub kind: ProfileKind,
    /// Shape of the jerk in the S-Curve ramps
    pub jerk_shape: JerkShape,
    /// Phase boundaries
    pub times: Times,
    /// Initial velocity.
    pub v_i: math::Real,
    /// Final velocity.
    pub v_o: math::Real,
    /// Initial acceleration (as actually used by the profile).
    pub a_i: math::Real,
    /// Final acceleration (as actually used by the profile).
    pub a_o: math::Real,
    /// Jerk the S-Curve timing was planned with ([`effective_jerk`]).
    pub j_max: math::Real,
    /// Peak acceleration reached during the acceleration phase (magnitude).
    pub a_lim_a: math::Real,
    /// Peak deceleration reached during the deceleration phase (magnitude).
    pub a_lim_d: math::Real,
    /// Peak (cruise) velocity. Zero for the polynomial path.
    pub v_lim: math::Real,
    /// Displacement to be achieved.
    pub s_o: math::Real,
    /// Constraints applied to the motion profile.
    pub constraints: Constraints,
    /// Cached values for cheap evaluation.
    pub cache: Cache,
}

// ─────────────────────────────────────────────────────────────────────────────
// Velocity ramps (closed forms of a jerk-limited monotone velocity change)
// ─────────────────────────────────────────────────────────────────────────────

/// A jerk-limited velocity change `v0 → v1` with zero acceleration at both ends.
#[derive(Clone, Copy)]
struct Ramp {
    /// Duration of each jerk sub-phase
    t_j: Real,
    /// Total duration
    t: Real,
    /// Peak acceleration (magnitude)
    a_peak: Real,
    /// Covered distance
    dist: Real,
}

impl Ramp {
    const NONE: Ramp = Ramp {
        t_j: math::ZERO,
        t: math::ZERO,
        a_peak: math::ZERO,
        dist: math::ZERO,
    };

    /// Closed form of [1] eq. 3.19/3.20 (the acceleration profile is symmetric in time, hence the
    /// distance is the mean velocity times the duration — valid for both [`JerkShape`]s).
    fn new(v0: Real, v1: Real, a_max: Real, j: Real) -> Self {
        let dv = (v1 - v0).abs();
        if !(dv > math::ZERO) {
            return Self::NONE;
        }
        let (t_j, t, a_peak) = if dv * j >= a_max * a_max {
            let t_j = a_max / j;
            (t_j, t_j + dv / a_max, a_max)
        } else {
            let t_j = (dv / j).sqrt().unwrap_or(math::ZERO);
            (t_j, t_j + t_j, j * t_j)
        };
        Ramp {
            t_j,
            t,
            a_peak,
            dist: (v0 + v1) * math::HALF * t,
        }
    }

    /// `∂dist/∂v_peak` of a ramp between a fixed velocity `v_fix` and a peak `v_peak > v_fix`.
    /// `None` when singular (`v_peak == v_fix`).
    fn ddist_dpeak(v_fix: Real, v_peak: Real, a_max: Real, j: Real) -> Option<Real> {
        let dv = v_peak - v_fix;
        if !(dv > math::ZERO) {
            return None;
        }
        if dv * j >= a_max * a_max {
            Some(a_max * math::HALF / j + v_peak / a_max)
        } else {
            let r = (dv / j).sqrt()?;
            if !(r > math::ZERO) {
                return None;
            }
            Some(r + (v_fix + v_peak) * math::HALF / (j * r))
        }
    }
}

// Normalised ramp shape functions (τ ∈ [0, 1]):
//   χ(τ): acceleration / A,  ψ(τ): Δv / (A·T),  φ(τ): Δs_acc / (A·T²)
// "up" ramps go from 0 to A, "down" ramps from A to 0; χ_d = 1 − χ_u, ψ_d = τ − ψ_u,
// φ_d = τ²/2 − φ_u.

#[inline]
fn ramp_chi(shape: JerkShape, up: bool, tau: Real) -> Real {
    let u = match shape {
        JerkShape::Linear => tau,
        JerkShape::Smooth => tau * tau * (math::THREE - math::TWO * tau),
    };
    if up { u } else { math::ONE - u }
}

#[inline]
fn ramp_psi(shape: JerkShape, up: bool, tau: Real) -> Real {
    let u = match shape {
        JerkShape::Linear => math::HALF * tau * tau,
        JerkShape::Smooth => tau * tau * tau * (math::ONE - math::HALF * tau),
    };
    if up { u } else { tau - u }
}

#[inline]
fn ramp_phi(shape: JerkShape, up: bool, tau: Real) -> Real {
    let tau2 = tau * tau;
    let u = match shape {
        JerkShape::Linear => math::SIXTH * tau2 * tau,
        JerkShape::Smooth => tau2 * tau2 * (math::QUARTER - math::TENTH * tau),
    };
    if up { u } else { math::HALF * tau2 - u }
}

/// dχ/dτ (jerk shape)
#[inline]
fn ramp_chi_dot(shape: JerkShape, up: bool, tau: Real) -> Real {
    let u = match shape {
        JerkShape::Linear => math::ONE,
        JerkShape::Smooth => math::SIX * tau * (math::ONE - tau),
    };
    if up { u } else { -u }
}

// ─────────────────────────────────────────────────────────────────────────────
// Polynomial blend helpers (shared with the motion-chaining planner)
// ─────────────────────────────────────────────────────────────────────────────

/// Velocity of the blend in normalised time (Horner, `b_2 == 0`)
#[inline]
fn blend_velocity(b: &[Real; 6], tau: Real) -> Real {
    b[0] + tau * (b[1] + tau * tau * (b[3] + tau * (b[4] + tau * b[5])))
}

/// Minimum velocity of the blend over `[0, 1]` (sampled)
fn blend_min_velocity(b: &[Real; 6]) -> Real {
    let inv_n = Real::from_lit(BLEND_ADMISSIBILITY_SAMPLES, 0).recip();
    let mut v_min = b[0];
    // τ_k = k/n is exact both multiplied and accumulated (n is a power of two)
    let mut tau = math::ZERO;
    for _ in 0..BLEND_ADMISSIBILITY_SAMPLES {
        tau += inv_n;
        let v = blend_velocity(b, tau);
        if v < v_min {
            v_min = v;
        }
    }
    v_min
}

/// `true` when the blend velocity never drops below `v_floor` on `[0, 1]` (sampled criterion of
/// [`blend_min_velocity`]).
///
/// Fast path: the velocity is a quintic, and by the convex-hull property of its Bernstein
/// (Bézier) form it is bounded below by its smallest control point. With `b_2 == 0` the control
/// points are `c_j = Σ_i C(j,i)/C(5,i)·b_i`. When all of them are `>= v_floor` the sampled
/// minimum is too, so the expensive sampling is only needed when the bound is inconclusive
/// (decisions are identical to sampling alone).
fn blend_velocity_above(b: &[Real; 6], v_floor: Real) -> bool {
    let c1 = b[0] + b[1] * math::FIFTH;
    let c2 = b[0] + b[1] * math::TWO * math::FIFTH;
    let c3 = b[0] + b[1] * math::THREE * math::FIFTH + b[3] * math::TENTH;
    let c4 = b[0] + b[1] * math::FOUR * math::FIFTH + b[3] * math::FOUR * math::TENTH + b[4] * math::FIFTH;
    let c5 = b[0] + b[1] + b[3] + b[4] + b[5];
    let c_min = b[0].min(c1).min(c2).min(c3).min(c4).min(c5);
    c_min >= v_floor || blend_min_velocity(b) >= v_floor
}

/// Duration and velocity coefficients of the polynomial blend, or `None` when no positive
/// duration satisfies the boundary conditions.
///
/// The velocity is the quintic Hermite interpolant with `v(0)=v_i, v'(0)=a_i, v''(0)=0` and
/// `v(T)=v_o, v'(T)=a_o, v''(T)=0`. Integrating its basis functions gives the displacement
///
/// `s_o = T·(v_i + v_o)/2 + T²·(a_i − a_o)/10`,
///
/// a quadratic in `T` whose smallest positive root is computed with the cancellation-free form
/// `T = 2·s_o / (q_b + √Δ)`, `q_b = (v_i+v_o)/2`, `Δ = q_b² + 0.4·(a_i − a_o)·s_o`.
fn blend_coefficients(
    s_o: Real,
    v_i: Real,
    v_o: Real,
    a_i: Real,
    a_o: Real,
) -> Option<(Real, [Real; 6])> {
    let qb = (v_i + v_o) * math::HALF;
    let qb2 = qb * qb;
    let mut delta = qb2 + math::FOUR * math::TENTH * (a_i - a_o) * s_o;
    if delta < math::ZERO {
        // Accept rounding-level negatives (e.g. right at the feasibility boundary)
        if delta >= -(qb2 * rel_tol()) {
            delta = math::ZERO;
        } else {
            return None;
        }
    }
    let denom = qb + delta.sqrt()?;
    if !(denom > math::ZERO) {
        return None;
    }
    let t = math::TWO * s_o / denom;
    if !(t > math::ZERO) {
        return None;
    }
    let dv = v_o - v_i;
    let ta_i = t * a_i;
    let ta_o = t * a_o;
    Some((
        t,
        [
            v_i,
            ta_i,
            math::ZERO,
            math::TEN * dv - math::SIX * ta_i - math::FOUR * ta_o,
            math::EIGHT * ta_i + math::SEVEN * ta_o - math::FIFTEEN * dv,
            math::SIX * dv - math::THREE * (ta_i + ta_o),
        ],
    ))
}

/// Largest exit acceleration for which a polynomial blend exists given the (fixed) entry state:
/// the value that makes the discriminant zero, `a_o = a_i + 5·(v_i+v_o)²/(8·s_o)`.
pub fn blend_accel_exit_cap(s_o: Real, v_i: Real, v_o: Real, a_i: Real) -> Real {
    let v_sum = v_i + v_o;
    a_i + math::FIVE * v_sum * v_sum / (math::EIGHT * s_o)
}

/// Minimum displacement an S-Curve ([`DEFAULT_JERK_SHAPE`], zero boundary accelerations) needs
/// to change the velocity from `v_i` to `v_o` within `constraints`.
///
/// A segment whose displacement is close to this value is *ramp-limited*: its whole length is
/// used by the velocity change, so carrying a non-zero acceleration through its junctions
/// (polynomial blend) costs nothing. A segment with slack instead peaks (or cruises) inside, so
/// its junctions are velocity extrema and should keep zero acceleration (S-Curve).
pub fn scurve_min_distance(v_i: Real, v_o: Real, constraints: &Constraints) -> Real {
    Ramp::new(
        v_i.max(math::ZERO),
        v_o.max(math::ZERO),
        constraints.a_max,
        effective_jerk(constraints.j_max, DEFAULT_JERK_SHAPE),
    )
    .dist
}

/// `true` when the polynomial blend for these boundary conditions exists and never reverses
/// its velocity (the stepper cannot change direction within a segment).
///
/// The planner uses this predicate so that the profile computed later by the stepper never needs
/// to alter the boundary conditions (which would break acceleration continuity).
pub fn blend_is_admissible(s_o: Real, v_i: Real, v_o: Real, a_i: Real, a_o: Real) -> bool {
    match blend_coefficients(s_o, v_i, v_o, a_i, a_o) {
        None => false,
        Some((_, b)) => blend_velocity_above(&b, -((v_i + v_o) * rel_tol())),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Profile computation
// ─────────────────────────────────────────────────────────────────────────────

impl HybridMotionProfile {
    /// Compute the hybrid motion profile for a scalar displacement with the
    /// [`DEFAULT_JERK_SHAPE`].
    ///
    /// | Condition | Algorithm used |
    /// |---|---|
    /// | `a_i ≈ a_o ≈ 0` (or rest-to-rest) and displacement long enough | S-Curve |
    /// | `a_i ≈ a_o ≈ 0` but displacement too short for the velocity change | Polynomial (degraded) |
    /// | Non-negligible `a_i` or `a_o` | Polynomial |
    ///
    /// The S-Curve is time-optimal: it accelerates towards `v_max` whenever the displacement allows
    /// it (a segment whose boundary speeds already equal `v_max` is an exact pure cruise).
    ///
    /// # Arguments
    ///
    /// * `s_o` — Scalar displacement in \[[hwa::HwiContract::SPACE_UNIT_MAGNITUDE]\].
    /// * `v_i` — Entry velocity in \[[hwa::HwiContract::SPACE_UNIT_MAGNITUDE]\]/s.
    /// * `v_o` — Exit velocity in \[[hwa::HwiContract::SPACE_UNIT_MAGNITUDE]\]/s.
    /// * `a_i` *(feature `with-motion-chaining`)* — Entry acceleration.
    /// * `a_o` *(feature `with-motion-chaining`)* — Exit acceleration.
    /// * `constraints` — Kinematic limits `{v_max, a_max, j_max}`.
    /// * `_error_correction` — Kept for API compatibility: the rounding residual of the final
    ///   position is now always absorbed in the cruise phase.
    pub fn compute(
        s_o: math::Real,
        v_i: math::Real,
        v_o: math::Real,
        #[cfg(feature = "with-motion-chaining")] a_i: math::Real,
        #[cfg(feature = "with-motion-chaining")] a_o: math::Real,
        constraints: &Constraints,
        _error_correction: bool,
    ) -> Result<HybridMotionProfile, processing::CodeExecutionFailure> {
        cfg_if::cfg_if! {
            if #[cfg(not(feature = "with-motion-chaining"))] {
                let a_i = math::ZERO;
                let a_o = math::ZERO;
            }
        }
        Self::compute_with_shape(s_o, v_i, v_o, a_i, a_o, constraints, DEFAULT_JERK_SHAPE)
    }

    /// Same as [`Self::compute`] with an explicit [`JerkShape`].
    pub fn compute_with_shape(
        s_o: Real,
        v_i: Real,
        v_o: Real,
        a_i: Real,
        a_o: Real,
        constraints: &Constraints,
        shape: JerkShape,
    ) -> Result<HybridMotionProfile, processing::CodeExecutionFailure> {
        if !(constraints.v_max > math::ZERO)
            || !(constraints.a_max > math::ZERO)
            || !(constraints.j_max > math::ZERO)
            || constraints.v_max.is_negligible()
            || constraints.a_max.is_negligible()
            || constraints.j_max.is_negligible()
        {
            hwa::warn!("[HybridMotionProfile] Unable to perform movement: constraints are unset");
            return Err(processing::CodeExecutionFailure::NumericalError);
        }
        if !(s_o > math::ZERO) {
            return Err(processing::CodeExecutionFailure::NumericalError);
        }
        // Rounding residuals may produce tiny negative speeds
        let v_i = v_i.max(math::ZERO);
        let v_o = v_o.max(math::ZERO);

        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[HybridMotionProfile] s_o={:?} v_i={:?} v_o={:?} a_i={:?} a_o={:?} {:?}",
            s_o, v_i, v_o, a_i, a_o, constraints
        );

        let a_tol = accel_bc_tol(constraints.a_max);
        let rest_to_rest = (v_i + v_o) <= constraints.v_max * rel_tol();
        let zero_accel_bc = a_i.abs() <= a_tol && a_o.abs() <= a_tol;

        if zero_accel_bc || rest_to_rest {
            if !zero_accel_bc {
                // A velocity minimum (rest) has zero acceleration; non-zero values are planner noise.
                hwa::warn!(
                    "[HybridMotionProfile] Ignoring boundary accelerations a_i={:?} a_o={:?} of a rest-to-rest segment",
                    a_i,
                    a_o
                );
            }
            // Rest residuals (e.g. 2e-10) are rest: a "cruise" at such a speed would last forever.
            let (v_i, v_o) = if rest_to_rest {
                (math::ZERO, math::ZERO)
            } else {
                (v_i, v_o)
            };
            // Time-optimal: accelerate up to v_max whenever the displacement allows it. A segment
            // whose boundary speeds already equal v_max is executed as an exact pure cruise.
            // (The former "v_i ≈ v_o ⇒ cruise at v_i" rule made every isolated move, which starts
            // and ends at min_speed, crawl along at min_speed: e.g. 141 mm in 9.8 s at 14 mm/s
            // with a 848 mm/s target.)
            let v_cap = constraints.v_max.max(v_i.max(v_o));
            if let Some(profile) =
                Self::compute_scurve(s_o, v_i, v_o, v_cap, constraints, shape)?
            {
                return Ok(profile);
            }
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!(
                "[HybridMotionProfile] Displacement too short for the velocity change: polynomial blend"
            );
            return Self::compute_blend(s_o, v_i, v_o, math::ZERO, math::ZERO, constraints, shape);
        }
        let blend = Self::compute_blend(s_o, v_i, v_o, a_i, a_o, constraints, shape)?;
        // Safety net: with a tiny speed sum and a_i ≲ a_o the blend duration T ≈ 4·s/(v_i+v_o)
        // grows without bound (the stepper task would be stuck executing it). When the mean speed
        // of the blend is negligible w.r.t. v_max, use the S-Curve instead: the junction
        // acceleration then jumps (bounded, piecewise continuous) instead of stalling the motion.
        if s_o < blend.times.t_8s * constraints.v_max * Real::from_f32(1.0e-3) {
            if let Some(sc) = Self::compute_scurve(
                s_o,
                v_i,
                v_o,
                constraints.v_max.max(v_i.max(v_o)),
                constraints,
                shape,
            )? {
                hwa::warn!(
                    "[HybridMotionProfile] Degenerate blend (T={:?} for s={:?}); using the S-Curve (T={:?})",
                    blend.times.t_8s,
                    s_o,
                    sc.times.t_8s
                );
                return Ok(sc);
            }
        }
        Ok(blend)
    }

    /// Double S-Curve with zero boundary accelerations, or `None` when the displacement is too
    /// short to perform the velocity change `v_i → v_o` within the limits.
    fn compute_scurve(
        s_o: Real,
        v_i: Real,
        v_o: Real,
        v_cap: Real,
        constraints: &Constraints,
        shape: JerkShape,
    ) -> Result<Option<Self>, processing::CodeExecutionFailure> {
        let a_max = constraints.a_max;
        let j = effective_jerk(constraints.j_max, shape);
        let v_lo = v_i.max(v_o);
        let v_cap = v_cap.max(v_lo);
        if !(v_cap > math::ZERO) {
            return Err(processing::CodeExecutionFailure::NumericalError);
        }

        // [1] §3.4: feasibility — the displacement must cover the direct transition v_i → v_o
        let direct = Ramp::new(v_i, v_o, a_max, j);
        if s_o < direct.dist * (math::ONE - rel_tol()) {
            return Ok(None);
        }

        let (v_p, t_v) = {
            let full = Ramp::new(v_i, v_cap, a_max, j).dist + Ramp::new(v_cap, v_o, a_max, j).dist;
            if full <= s_o {
                // v_cap is reached: cruise for the remaining distance
                (v_cap, (s_o - full) / v_cap)
            } else if s_o <= direct.dist {
                (v_lo, math::ZERO)
            } else {
                (
                    Self::solve_peak_velocity(s_o, v_i, v_o, v_lo, v_cap, a_max, j),
                    math::ZERO,
                )
            }
        };

        let r_a = Ramp::new(v_i, v_p, a_max, j);
        let r_d = Ramp::new(v_p, v_o, a_max, j);
        let t_5 = r_a.t + t_v;
        let mut profile = HybridMotionProfile {
            kind: ProfileKind::SCurve,
            jerk_shape: shape,
            times: Times::new(
                r_a.t_j,
                r_a.t - r_a.t_j,
                r_a.t,
                t_5,
                t_5 + r_d.t_j,
                t_5 + r_d.t - r_d.t_j,
                t_5 + r_d.t,
            ),
            v_i,
            v_o,
            a_i: math::ZERO,
            a_o: math::ZERO,
            j_max: j,
            a_lim_a: r_a.a_peak,
            a_lim_d: r_d.a_peak,
            v_lim: v_p,
            s_o,
            constraints: *constraints,
            cache: Cache::default(),
        };
        profile.build_scurve_cache();
        // Absorb the rounding residual of the final position in the cruise phase
        let residual = s_o - profile.cache.s[7];
        let _ = profile.extend(residual);

        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[HybridMotionProfile] S-Curve: {:?}", profile);
        Ok(Some(profile))
    }

    /// Peak velocity `v ∈ [lo, hi]` such that the jerk-limited ramps `v_i → v → v_o` cover exactly
    /// `s`. The covered distance is continuous and strictly increasing in `v`, so a safeguarded
    /// Newton iteration (bisection fallback) always converges, typically in 3-8 iterations.
    ///
    /// This replaces the book's `a_max ← γ·a_max` loop, which could take ~100 iterations and was
    /// sub-optimal whenever only one side of the profile could reach `a_max`.
    fn solve_peak_velocity(
        s: Real,
        v_i: Real,
        v_o: Real,
        lo: Real,
        hi: Real,
        a_max: Real,
        j: Real,
    ) -> Real {
        let mut lo = lo;
        let mut hi = hi;
        let tol_s = s * rel_tol() * math::TENTH;
        let mut v = (lo + hi) * math::HALF;
        for _ in 0..PEAK_VELOCITY_MAX_ITER {
            let d = Ramp::new(v_i, v, a_max, j).dist + Ramp::new(v, v_o, a_max, j).dist;
            let f = d - s;
            if f.abs() <= tol_s {
                return v;
            }
            if f > math::ZERO {
                hi = v;
            } else {
                lo = v;
            }
            let mid = (lo + hi) * math::HALF;
            let newton = match (
                Ramp::ddist_dpeak(v_i, v, a_max, j),
                Ramp::ddist_dpeak(v_o, v, a_max, j),
            ) {
                (Some(d1), Some(d2)) => Some(v - f / (d1 + d2)),
                _ => None,
            };
            v = match newton {
                Some(n) if n > lo && n < hi => n,
                _ => mid,
            };
            if !(hi - lo > hi * math::EPSILON * math::FOUR) {
                break;
            }
        }
        // The lower bracket never overshoots the displacement
        lo
    }

    /// Polynomial Hermite blend. The entry state `(v_i, a_i)` is always preserved (it is the exit
    /// state of the segment being executed). If no blend exists for `a_o` (the planner should have
    /// prevented it, see `motion_chaining`), `a_o` is capped at the feasibility boundary.
    fn compute_blend(
        s_o: Real,
        v_i: Real,
        v_o: Real,
        a_i: Real,
        a_o: Real,
        constraints: &Constraints,
        shape: JerkShape,
    ) -> Result<Self, processing::CodeExecutionFailure> {
        let (a_o, (t, b)) = match blend_coefficients(s_o, v_i, v_o, a_i, a_o) {
            Some(c) => (a_o, c),
            None => {
                let a_o_cap = blend_accel_exit_cap(s_o, v_i, v_o, a_i);
                hwa::warn!(
                    "[HybridMotionProfile] Infeasible exit acceleration a_o={:?} capped to {:?} (s={:?} v_i={:?} v_o={:?} a_i={:?})",
                    a_o, a_o_cap, s_o, v_i, v_o, a_i
                );
                match blend_coefficients(s_o, v_i, v_o, a_i, a_o_cap) {
                    Some(c) => (a_o_cap, c),
                    None => return Err(processing::CodeExecutionFailure::NumericalError),
                }
            }
        };
        if !blend_velocity_above(&b, -((v_i + v_o) * rel_tol())) {
            hwa::warn!(
                "[HybridMotionProfile] Polynomial blend reverses velocity (s={:?} v_i={:?} v_o={:?} a_i={:?} a_o={:?})",
                s_o, v_i, v_o, a_i, a_o
            );
        }
        let inv_t = t.recip();
        let cache = Cache {
            inv_t,
            poly_v: b,
            poly_s: [
                math::ZERO,
                t * b[0],
                t * b[1] * math::HALF,
                math::ZERO,
                t * b[3] * math::QUARTER,
                t * b[4] * math::FIFTH,
                t * b[5] * math::SIXTH,
            ],
            poly_a: [
                b[1] * inv_t,
                math::THREE * b[3] * inv_t,
                math::FOUR * b[4] * inv_t,
                math::FIVE * b[5] * inv_t,
            ],
            ..Cache::default()
        };
        let profile = HybridMotionProfile {
            kind: ProfileKind::Polynomial,
            jerk_shape: shape,
            times: Times::new(t, t, t, t, t, t, t),
            v_i,
            v_o,
            a_i,
            a_o,
            j_max: constraints.j_max,
            a_lim_a: math::ZERO,
            a_lim_d: math::ZERO,
            v_lim: math::ZERO,
            s_o,
            constraints: *constraints,
            cache,
        };
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[HybridMotionProfile] Polynomial: {:?}", profile);
        Ok(profile)
    }

    /// Velocity and position at every phase boundary of the S-Curve.
    fn build_scurve_cache(&mut self) {
        let tj1 = self.times.t_j1();
        let tj2 = self.times.t_j2();
        self.cache.inv_tj1 = if tj1 > math::ZERO { tj1.recip() } else { math::ZERO };
        self.cache.inv_tj2 = if tj2 > math::ZERO { tj2.recip() } else { math::ZERO };
        self.cache.v[0] = self.v_i;
        self.cache.s[0] = math::ZERO;
        for phase in 1..=7 {
            let t0 = self.times.phase_start(phase);
            let t1 = if phase == 7 {
                self.times.t_8s
            } else {
                self.times.phase_start(phase + 1)
            };
            let dt = t1 - t0;
            let v0 = self.cache.v[phase - 1];
            let (ds, v, _) = self.phase_state(phase, dt, v0);
            self.cache.v[phase] = v;
            self.cache.s[phase] = self.cache.s[phase - 1] + ds;
        }
    }

    /// `(ds, v, a)` of an S-Curve phase `dt` seconds after its start, given its initial velocity.
    #[inline]
    fn phase_state(&self, phase: usize, dt: Real, v0: Real) -> (Real, Real, Real) {
        let (up, amp, t_len, inv) = match phase {
            1 => (true, self.a_lim_a, self.times.t_j1(), self.cache.inv_tj1),
            3 => (false, self.a_lim_a, self.times.t_j1(), self.cache.inv_tj1),
            5 => (true, -self.a_lim_d, self.times.t_j2(), self.cache.inv_tj2),
            7 => (false, -self.a_lim_d, self.times.t_j2(), self.cache.inv_tj2),
            _ => {
                let a = match phase {
                    2 => self.a_lim_a,
                    6 => -self.a_lim_d,
                    _ => math::ZERO,
                };
                return (v0 * dt + math::HALF * a * dt * dt, v0 + a * dt, a);
            }
        };
        let tau = dt * inv;
        let s = self.jerk_shape;
        (
            v0 * dt + amp * t_len * t_len * ramp_phi(s, up, tau),
            v0 + amp * t_len * ramp_psi(s, up, tau),
            amp * ramp_chi(s, up, tau),
        )
    }

    /// Position-only variant of [`Self::phase_state`] for the sampling hot path.
    #[inline]
    fn phase_position(&self, phase: usize, dt: Real, v0: Real) -> Real {
        let (up, amp, t_len, inv) = match phase {
            1 => (true, self.a_lim_a, self.times.t_j1(), self.cache.inv_tj1),
            3 => (false, self.a_lim_a, self.times.t_j1(), self.cache.inv_tj1),
            5 => (true, -self.a_lim_d, self.times.t_j2(), self.cache.inv_tj2),
            7 => (false, -self.a_lim_d, self.times.t_j2(), self.cache.inv_tj2),
            2 => return (v0 + math::HALF * self.a_lim_a * dt) * dt,
            6 => return (v0 - math::HALF * self.a_lim_d * dt) * dt,
            _ => return v0 * dt,
        };
        v0 * dt + amp * t_len * t_len * ramp_phi(self.jerk_shape, up, dt * inv)
    }

    /// Extends the cruise phase so that the final position grows by `delta_e`.
    /// Has no effect when the cruise phase would become negative or on polynomial profiles.
    pub fn extend(&mut self, delta_e: math::Real) -> Result<(), ()> {
        if self.kind != ProfileKind::SCurve || !(self.v_lim > math::ZERO) {
            return Ok(());
        }
        let extra_time = delta_e / self.v_lim;
        if self.times.t_v() + extra_time >= math::ZERO {
            self.times.t_5s += extra_time;
            self.times.t_6s += extra_time;
            self.times.t_7s += extra_time;
            self.times.t_8s += extra_time;
            for k in 4..8 {
                self.cache.s[k] += delta_e;
            }
        }
        Ok(())
    }

    /// The time at the end of the profile
    #[inline]
    pub const fn i7_end(&self) -> math::Real {
        self.times.t_8s
    }

    /// Jerk at the given relative instant (not part of [`motion::MotionProfile`]; used for
    /// diagnostics and tests).
    pub fn eval_jerk(&self, t: Real) -> Real {
        if !(t > math::ZERO) || t >= self.times.t_8s {
            return math::ZERO;
        }
        match self.kind {
            ProfileKind::Polynomial => {
                let b = &self.cache.poly_v;
                let tau = t * self.cache.inv_t;
                tau * (math::SIX * b[3] + tau * (math::TWELVE * b[4] + math::TWENTY * b[5] * tau))
                    * self.cache.inv_t
                    * self.cache.inv_t
            }
            ProfileKind::SCurve => {
                let phase = self.times.phase_of(t);
                let dt = t - self.times.phase_start(phase);
                let (up, amp, inv) = match phase {
                    1 => (true, self.a_lim_a, self.cache.inv_tj1),
                    3 => (false, self.a_lim_a, self.cache.inv_tj1),
                    5 => (true, -self.a_lim_d, self.cache.inv_tj2),
                    7 => (false, -self.a_lim_d, self.cache.inv_tj2),
                    _ => return math::ZERO,
                };
                amp * inv * ramp_chi_dot(self.jerk_shape, up, dt * inv)
            }
        }
    }
}

impl motion::MotionProfile for HybridMotionProfile {
    #[inline(always)]
    fn end_time(&self) -> math::Real {
        self.times.t_8s
    }

    #[inline(always)]
    fn end_pos(&self) -> math::Real {
        self.s_o
    }

    fn eval_interval(&self, t: math::Real) -> usize {
        if !(t > math::ZERO) {
            0
        } else if t > self.times.t_8s {
            8
        } else if self.kind == ProfileKind::Polynomial {
            1
        } else if t == self.times.t_8s {
            7
        } else {
            self.times.phase_of(t)
        }
    }

    /// Position in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE] at the given relative instant (secs).
    /// Returns exactly `0` for `t ≤ 0` and exactly `s_o` for `t ≥ T`.
    fn eval_position(&self, t: math::Real) -> math::Real {
        if !(t > math::ZERO) {
            return math::ZERO;
        }
        if t >= self.times.t_8s {
            return self.s_o;
        }
        match self.kind {
            ProfileKind::Polynomial => {
                let p = &self.cache.poly_s;
                let tau = t * self.cache.inv_t;
                tau * (p[1] + tau * (p[2] + tau * tau * (p[4] + tau * (p[5] + tau * p[6]))))
            }
            ProfileKind::SCurve => {
                let phase = self.times.phase_of(t);
                let dt = t - self.times.phase_start(phase);
                self.cache.s[phase - 1] + self.phase_position(phase, dt, self.cache.v[phase - 1])
            }
        }
    }

    /// Returns exactly `v_i` for `t ≤ 0` and exactly `v_o` for `t ≥ T`.
    fn eval_velocity(&self, t: math::Real) -> math::Real {
        if !(t > math::ZERO) {
            return self.v_i;
        }
        if t >= self.times.t_8s {
            return self.v_o;
        }
        match self.kind {
            ProfileKind::Polynomial => blend_velocity(&self.cache.poly_v, t * self.cache.inv_t),
            ProfileKind::SCurve => {
                let phase = self.times.phase_of(t);
                let dt = t - self.times.phase_start(phase);
                self.phase_state(phase, dt, self.cache.v[phase - 1]).1
            }
        }
    }

    /// Returns exactly `a_i` for `t ≤ 0` and exactly `a_o` for `t ≥ T`.
    fn eval_acceleration(&self, t: math::Real) -> math::Real {
        if !(t > math::ZERO) {
            return self.a_i;
        }
        if t >= self.times.t_8s {
            return self.a_o;
        }
        match self.kind {
            ProfileKind::Polynomial => {
                let c = &self.cache.poly_a;
                let tau = t * self.cache.inv_t;
                c[0] + tau * tau * (c[1] + tau * (c[2] + tau * c[3]))
            }
            ProfileKind::SCurve => {
                let phase = self.times.phase_of(t);
                let dt = t - self.times.phase_start(phase);
                self.phase_state(phase, dt, self.cache.v[phase - 1]).2
            }
        }
    }
}

impl core::fmt::Debug for HybridMotionProfile {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "HybridMotionProfile {{ kind = {:?}, shape = {:?}, times = {{{:?}}} j = {:?}, a_i = {:?} v_i = {:?}, v_lim = {:?}, v_o = {:?}, a_o = {:?}, s_o = {:?} }}",
            self.kind,
            self.jerk_shape,
            self.times,
            self.j_max,
            self.a_i,
            self.v_i,
            self.v_lim,
            self.v_o,
            self.a_o,
            self.s_o
        )
    }
}

#[cfg(test)]
pub mod test {
    use super::{JerkShape, ProfileKind};
    use crate::{hwa, motion, processing};
    use hwa::math;
    use math::Real;
    use motion::{Constraints, HybridMotionProfile, MotionProfile};

    /// Classic (book) profile: linear jerk, zero boundary accelerations.
    pub fn do_compute(
        q1: f32,
        v_0: f32,
        v_1: f32,
        v_max: f32,
        a_max: f32,
        j_max: f32,
        _error_correction: bool,
    ) -> Result<HybridMotionProfile, processing::CodeExecutionFailure> {
        let constraints = Constraints {
            v_max: Real::from_f32(v_max),
            a_max: Real::from_f32(a_max),
            j_max: Real::from_f32(j_max),
        };
        HybridMotionProfile::compute_with_shape(
            Real::from_f32(q1),
            Real::from_f32(v_0),
            Real::from_f32(v_1),
            math::ZERO,
            math::ZERO,
            &constraints,
            JerkShape::Linear,
        )
    }

    fn approx_equal(what: &str, v: Real, expected: f32, tolerance: f32) {
        let v1 = v.to_f64() as f32;
        let ok = (v1 - expected).abs() < tolerance;
        assert!(ok, "{} = {} but should be = {}", what, v1, expected);
    }

    fn f(v: Real) -> f64 {
        v.to_f64()
    }

    /// Checks the physical invariants of a profile by dense sampling:
    /// exact boundary values, monotone position, |a| ≤ a_max, |j| ≤ j_max,
    /// and Lipschitz continuity of velocity (by a_max) and acceleration (by j_max).
    fn check_invariants(p: &HybridMotionProfile, enforce_limits: bool) {
        let t_end = f(p.end_time());
        assert!(t_end > 0.0, "positive duration: {:?}", p);
        assert_eq!(p.eval_position(p.end_time()), p.s_o, "exact final position");
        assert_eq!(p.eval_position(math::ZERO), math::ZERO);
        assert_eq!(p.eval_velocity(p.end_time()), p.v_o);
        assert_eq!(p.eval_acceleration(p.end_time()), p.a_o);

        let a_max = f(p.constraints.a_max);
        let j_max = f(p.constraints.j_max);
        let s_o = f(p.s_o);
        let n = 4000;
        let h = t_end / n as f64;
        let mut prev_s = 0.0f64;
        let mut prev_v = f(p.v_i);
        let mut prev_a = f(p.a_i);
        for k in 1..=n {
            let t = Real::from_f32((h * k as f64) as f32);
            let s = f(p.eval_position(t));
            let v = f(p.eval_velocity(t));
            let a = f(p.eval_acceleration(t));
            let j = f(p.eval_jerk(t));
            let tol_s = 1e-5 * s_o.max(1.0);
            assert!(s + tol_s >= prev_s, "position must not decrease: t={} s={} prev={} {:?}", f(t), s, prev_s, p);
            assert!(s <= s_o + tol_s, "no overshoot: s={} s_o={}", s, s_o);
            if enforce_limits {
                assert!(a.abs() <= a_max * (1.0 + 1e-3), "|a|={} > a_max={} {:?}", a, a_max, p);
                assert!(j.abs() <= j_max * (1.0 + 1e-3), "|j|={} > j_max={} {:?}", j, j_max, p);
            }
            let dt = f(t) - (h * (k - 1) as f64);
            let dt = dt.max(h);
            // Continuity: increments bounded by the derivative bound (with numeric slack)
            let a_bound = a.abs().max(prev_a.abs()) + j_max * dt;
            assert!((v - prev_v).abs() <= a_bound * dt * 1.05 + 1e-4 * (1.0 + v.abs()), "velocity jump at t={}: {} -> {} {:?}", f(t), prev_v, v, p);
            if enforce_limits {
                assert!((a - prev_a).abs() <= j_max * dt * 1.05 + 1e-3 * a_max, "acceleration jump at t={}: {} -> {} {:?}", f(t), prev_a, a, p);
            }
            prev_s = s;
            prev_v = v;
            prev_a = a;
        }
    }

    #[test]
    fn ex_3_9() {
        // With: q_1 = 10, v_0 = 1, v_1 = 0. Given: v_max = 5, a_max = 10, j_max = 30
        // Exp: T_a = 0.7333, T_v = 1.1433, T_d = 0.8333, T_j1 = 0.3333, T_j2 = 0.3333
        for ec in [false, true] {
            let r = do_compute(10., 1., 0., 5., 10., 30., ec).unwrap();
            approx_equal("T_a", r.times.t_a(), 0.7333, 0.001);
            approx_equal("T_d", r.times.t_d(), 0.8333, 0.001);
            approx_equal("T_j1", r.times.t_j1(), 0.3333, 0.001);
            approx_equal("T_j2", r.times.t_j2(), 0.3333, 0.001);
            approx_equal("T_v", r.times.t_v(), 1.1433, 0.001);
            check_invariants(&r, true);
        }
    }

    #[test]
    fn ex_3_10() {
        // With: q_1 = 10, v_0 = 1, v_1 = 0. Given: v_max = 10, a_max = 10, j_max = 30
        // Exp: Ta = 1.0747, T_v = 0.0, T_d = 1.1747, T_j1 = 0.3333, T_j2 = 0.3333, vlim = 8.4136
        let r = do_compute(10., 1., 0., 10., 10., 30., false).unwrap();
        approx_equal("T_a", r.times.t_a(), 1.0747, 0.001);
        approx_equal("T_v", r.times.t_v(), 0.0, 0.001);
        approx_equal("T_d", r.times.t_d(), 1.1747, 0.001);
        approx_equal("T_j1", r.times.t_j1(), 0.3333, 0.001);
        approx_equal("T_j2", r.times.t_j2(), 0.3333, 0.001);
        approx_equal("v_lim", r.v_lim, 8.4136, 0.001);
        check_invariants(&r, true);
    }

    #[test]
    fn ex_3_11() {
        // With: q_1 = 10, v_0 = 7, v_1 = 0. Given: v_max = 10, a_max = 10, j_max = 30
        // The book (γ-reduction of a_max) gives T = 1.938 (Ta = 0.4666, Td = 1.4718, vlim = 8.6329).
        // The exact peak-velocity solution keeps a_max on the deceleration side and is faster:
        // T = 1.7804 (Ta = 0.5336, Td = 1.2469, Tj1 = 0.2668, Tj2 = 0.3333, vlim = 9.1353).
        let r = do_compute(10., 7., 0., 10., 10., 30., false).unwrap();
        approx_equal("T_a", r.times.t_a(), 0.5336, 0.001);
        approx_equal("T_v", r.times.t_v(), 0.0, 0.001);
        approx_equal("T_d", r.times.t_d(), 1.2469, 0.001);
        approx_equal("T_j1", r.times.t_j1(), 0.2668, 0.001);
        approx_equal("T_j2", r.times.t_j2(), 0.3333, 0.001);
        approx_equal("vlim", r.v_lim, 9.1353, 0.001);
        assert!(r.end_time().to_f64() < 1.938, "must be faster than the book's sub-optimal solution");
        check_invariants(&r, true);
    }

    #[test]
    fn ex_3_12() {
        // With: q_1 = 10, v_0 = 7.5, v_1 = 0. Given: v_max = 10, a_max = 10, j_max = 30
        // The book gives a deceleration-only profile with T = 2.6667. The exact solution
        // accelerates to vlim = 9.3042 first: T = 1.7542.
        let r = do_compute(10., 7.5, 0., 10., 10., 30., false).unwrap();
        approx_equal("T_a", r.times.t_a(), 0.4905, 0.001);
        approx_equal("T_v", r.times.t_v(), 0.0, 0.001);
        approx_equal("T_d", r.times.t_d(), 1.2638, 0.001);
        approx_equal("T_j1", r.times.t_j1(), 0.2452, 0.001);
        approx_equal("T_j2", r.times.t_j2(), 0.3333, 0.001);
        approx_equal("v_lim", r.v_lim, 9.3042, 0.001);
        check_invariants(&r, true);
    }

    #[test]
    fn ex_3_13() {
        // With: q_1 = 10, v_0 = 0, v_1 = 0. Given: v_max = 10, a_max = 20, j_max = 30
        // Exp: Ta = 1.1006, T_v = 0.0, T_d = 1.1006, T_j1 = 0.5503, T_j2 = 0.5503, vlim = 9.0826
        // (the book's value 9.0826 is the γ-iteration approximation of the exact 9.0856)
        let r = do_compute(10., 0.0, 0., 10., 20., 30., false).unwrap();
        approx_equal("T_a", r.times.t_a(), 1.1006, 0.001);
        approx_equal("T_v", r.times.t_v(), 0.0, 0.001);
        approx_equal("T_d", r.times.t_d(), 1.1006, 0.001);
        approx_equal("T_j1", r.times.t_j1(), 0.5503, 0.001);
        approx_equal("T_j2", r.times.t_j2(), 0.5503, 0.001);
        approx_equal("v_lim", r.v_lim, 9.0856, 0.001);
        check_invariants(&r, true);
    }

    #[test]
    fn test_limits() {
        let r = do_compute(10., 1., 0., 10., 10., 30., false).unwrap();
        assert_eq!(r.eval_position(math::ZERO - math::ONE), math::ZERO);
        assert_eq!(r.eval_position(math::ONE_HUNDRED), r.s_o);
        approx_equal("pos", r.eval_position(math::ONE_HUNDRED), 10.0, 0.0001);
    }

    #[test]
    fn test_bad_args() {
        let profile = do_compute(10., 0., 0., 10., 0., 0., false);
        assert_eq!(profile, Err(processing::CodeExecutionFailure::NumericalError));
        let profile = do_compute(0., 0., 0., 10., 10., 10., false);
        assert_eq!(profile, Err(processing::CodeExecutionFailure::NumericalError));
    }

    /// Smooth jerk: acceleration is C¹ (jerk continuous and zero at every phase boundary),
    /// limits are respected and duration/displacement are those of a linear profile planned
    /// with ⅔·j_max.
    #[test]
    fn smooth_scurve_is_c1_and_within_limits() {
        let constraints = Constraints {
            v_max: Real::from_f32(10.0),
            a_max: Real::from_f32(10.0),
            j_max: Real::from_f32(30.0),
        };
        for &(q, v0, v1) in &[(10.0f32, 1.0f32, 0.0f32), (10.0, 0.0, 0.0), (10.0, 7.0, 0.0), (40.0, 2.0, 3.0), (0.5, 0.0, 0.0)] {
            let p = HybridMotionProfile::compute_with_shape(
                Real::from_f32(q), Real::from_f32(v0), Real::from_f32(v1),
                math::ZERO, math::ZERO, &constraints, JerkShape::Smooth,
            ).unwrap();
            assert_eq!(p.kind, ProfileKind::SCurve);
            check_invariants(&p, true);
            // Jerk vanishes at every phase boundary (from both sides)
            let tiny = Real::from_f32(1.0e-6);
            for tb in [p.times.t_2s, p.times.t_3s, p.times.t_4s, p.times.t_5s, p.times.t_6s, p.times.t_7s] {
                if tb > tiny && tb < p.end_time() - tiny {
                    for t in [tb - tiny, tb + tiny] {
                        assert!(f(p.eval_jerk(t)).abs() < 1e-3 * 30.0, "jerk at boundary {} = {:?} {:?}", f(tb), p.eval_jerk(t), p);
                    }
                }
            }
            // Same timing as the linear profile with 2/3 j_max
            let lin = HybridMotionProfile::compute_with_shape(
                Real::from_f32(q), Real::from_f32(v0), Real::from_f32(v1),
                math::ZERO, math::ZERO,
                &Constraints { j_max: Real::from_f32(20.0), ..constraints },
                JerkShape::Linear,
            ).unwrap();
            approx_equal("T", p.end_time(), f(lin.end_time()) as f32, 1e-4);
        }
    }

    /// Deterministic random sweep: compute never fails or panics for physically valid inputs and
    /// the S-Curve always satisfies its invariants.
    #[test]
    fn randomized_scurve_sweep() {
        let mut seed: u64 = 0x2545F4914F6CDD1D;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..300 {
            let v_max = 5.0 + 300.0 * rnd();
            let a_max = 50.0 + 5000.0 * rnd();
            let j_max = a_max * (5.0 + 200.0 * rnd());
            let s = 0.01 + 200.0 * rnd() * rnd();
            let v0 = v_max * rnd() * rnd();
            let v1 = v_max * rnd() * rnd();
            let c = Constraints {
                v_max: Real::from_f32(v_max as f32),
                a_max: Real::from_f32(a_max as f32),
                j_max: Real::from_f32(j_max as f32),
            };
            for shape in [JerkShape::Linear, JerkShape::Smooth] {
                let p = HybridMotionProfile::compute_with_shape(
                    Real::from_f32(s as f32), Real::from_f32(v0 as f32), Real::from_f32(v1 as f32),
                    math::ZERO, math::ZERO, &c, shape,
                ).expect("compute must succeed");
                // Polynomial fallback only happens when the velocity change does not fit;
                // limits are only guaranteed for the S-Curve.
                check_invariants(&p, p.kind == ProfileKind::SCurve);
            }
        }
    }

    /// Reproduce the exact values from a logged error (large opposite accelerations in a short
    /// decelerating segment): compute must succeed keeping the entry state.
    #[test]
    #[cfg(feature = "with-motion-chaining")]
    fn test_large_opposite_accels_short_decel_segment() {
        let constraints = Constraints {
            v_max: Real::from_f32(50.0),
            a_max: Real::from_f32(1302.75),
            j_max: Real::from_f32(65137.5),
        };
        let a_i = Real::from_f32(-1341.6409);
        let profile = HybridMotionProfile::compute(
            Real::from_f32(6.9999995),
            Real::from_f32(14.282858),
            Real::from_f32(0.0),
            a_i,
            Real::from_f32(1302.7523),
            &constraints,
            false,
        )
        .expect("compute should succeed");
        assert!(profile.times.t_8s > math::ZERO, "total time must be positive");
        assert_eq!(profile.a_i, a_i, "entry acceleration must never be altered");
    }

    #[test]
    #[cfg(feature = "with-motion-chaining")]
    fn test_max_opposite_accels_feasibility() {
        let a_max = 1000.0_f32;
        let constraints = Constraints {
            v_max: Real::from_f32(100.0),
            a_max: Real::from_f32(a_max),
            j_max: Real::from_f32(a_max * 100.0),
        };
        let result = HybridMotionProfile::compute(
            Real::from_f32(5.0),
            Real::from_f32(20.0),
            Real::from_f32(0.0),
            Real::from_f32(-a_max),
            Real::from_f32(a_max),
            &constraints,
            false,
        );
        assert!(result.is_ok(), "compute should succeed, got {:?}", result.err());
    }

    #[test]
    #[cfg(feature = "with-motion-chaining")]
    fn test_zero_accels_unchanged() {
        let constraints = Constraints {
            v_max: Real::from_f32(5.0),
            a_max: Real::from_f32(10.0),
            j_max: Real::from_f32(30.0),
        };
        let result = HybridMotionProfile::compute(
            Real::from_f32(10.0),
            Real::from_f32(1.0),
            Real::from_f32(0.0),
            math::ZERO,
            math::ZERO,
            &constraints,
            false,
        );
        assert_eq!(result.unwrap().kind, ProfileKind::SCurve);
    }

    // ── Boundary-condition tests ─────────────────────────────────────────────

    /// A polynomial blend reproduces its boundary conditions exactly and has zero jerk at
    /// both ends (C¹ acceleration across junctions).
    #[test]
    fn test_blend_boundary_conditions() {
        let constraints = Constraints {
            v_max: Real::from_f32(100.0),
            a_max: Real::from_f32(500.0),
            j_max: Real::from_f32(5000.0),
        };
        let (v_i, v_o, a_i, a_o, s_o) = (5.0_f32, 10.0_f32, 0.0_f32, 2.0_f32, 50.0_f32);
        let profile = HybridMotionProfile::compute_with_shape(
            Real::from_f32(s_o), Real::from_f32(v_i), Real::from_f32(v_o),
            Real::from_f32(a_i), Real::from_f32(a_o), &constraints, JerkShape::Smooth,
        )
        .expect("blend must succeed");
        assert_eq!(profile.kind, ProfileKind::Polynomial);

        let t_end = profile.end_time();
        let eps = Real::from_f32(1.0e-5) * t_end;
        let tol = 1e-3;
        // Limits from inside the interval (not the exact boundary shortcut)
        assert!((f(profile.eval_velocity(eps)) - v_i as f64).abs() < tol);
        assert!((f(profile.eval_velocity(t_end - eps)) - v_o as f64).abs() < tol);
        assert!((f(profile.eval_acceleration(eps)) - a_i as f64).abs() < 1e-2);
        assert!((f(profile.eval_acceleration(t_end - eps)) - a_o as f64).abs() < 1e-2);
        assert!(f(profile.eval_jerk(eps)).abs() < 1e-2);
        assert!(f(profile.eval_jerk(t_end - eps)).abs() < 1e-2);
        assert!((f(profile.eval_position(t_end - eps)) - s_o as f64).abs() < 1e-3);
        check_invariants(&profile, false);
    }

    /// An S-curve profile (a_i = a_o = 0) must reproduce v_i and v_o at the
    /// evaluation boundaries.
    #[test]
    fn test_scurve_eval_boundary_conditions() {
        let profile = do_compute(10.0, 1.0, 0.0, 5.0, 10.0, 30.0, false)
            .expect("S-curve compute should succeed");
        assert_eq!(profile.kind, ProfileKind::SCurve);
        let t_end = profile.end_time();
        let tol = 1e-2;
        assert!((f(profile.eval_velocity(math::ZERO)) - 1.0).abs() < tol);
        assert!(f(profile.eval_velocity(t_end)).abs() < tol);
        assert!(f(profile.eval_acceleration(math::ZERO)).abs() < tol);
        assert!(f(profile.eval_acceleration(t_end)).abs() < tol);
    }

    /// Two consecutive blends sharing a junction are continuous in velocity, acceleration
    /// and jerk (= 0) at that junction.
    #[test]
    fn test_consecutive_blends_continuity() {
        let constraints = Constraints {
            v_max: Real::from_f32(100.0),
            a_max: Real::from_f32(500.0),
            j_max: Real::from_f32(5000.0),
        };
        // Δ1 = 7.5² + 0.4·(0 − 2)·60 = 8.25 > 0 (feasible, no capping)
        let p1 = HybridMotionProfile::compute_with_shape(
            Real::from_f32(60.0), math::ZERO, Real::from_f32(15.0),
            math::ZERO, Real::from_f32(2.0), &constraints, JerkShape::Smooth,
        ).unwrap();
        assert_eq!(p1.a_o, Real::from_f32(2.0), "no capping expected");
        let p2 = HybridMotionProfile::compute_with_shape(
            Real::from_f32(80.0), Real::from_f32(15.0), Real::from_f32(15.0),
            Real::from_f32(2.0), math::ZERO, &constraints, JerkShape::Smooth,
        ).unwrap();
        let eps1 = p1.end_time() * Real::from_f32(1.0e-5);
        let eps2 = p2.end_time() * Real::from_f32(1.0e-5);
        let t1 = p1.end_time() - eps1;
        assert!((f(p1.eval_velocity(t1)) - f(p2.eval_velocity(eps2))).abs() < 1e-2);
        assert!((f(p1.eval_acceleration(t1)) - f(p2.eval_acceleration(eps2))).abs() < 1e-2);
        assert!((f(p1.eval_jerk(t1)) - f(p2.eval_jerk(eps2))).abs() < 1e-1);
    }

    /// Infeasible acceleration BCs: compute caps the exit acceleration but never touches the
    /// entry state.
    #[test]
    fn test_blend_symmetric_accel_large_segment() {
        let constraints = Constraints {
            v_max: Real::from_f32(200.0),
            a_max: Real::from_f32(1500.0),
            j_max: Real::from_f32(75000.0),
        };
        let p = HybridMotionProfile::compute_with_shape(
            Real::from_f32(500.0), Real::from_f32(100.0), Real::from_f32(100.0),
            Real::from_f32(-50.0), Real::from_f32(50.0), &constraints, JerkShape::Smooth,
        ).expect("must succeed (exit acceleration capped)");
        assert!(p.times.t_8s > math::ZERO);
        assert_eq!(p.a_i, Real::from_f32(-50.0));
        assert!(super::blend_is_admissible(p.s_o, p.v_i, p.v_o, p.a_i, p.a_o) || p.a_o < Real::from_f32(50.0));
    }

    /// Feasible BCs must not be modified.
    #[test]
    fn test_blend_feasible_bcs_no_clamping() {
        let constraints = Constraints {
            v_max: Real::from_f32(100.0),
            a_max: Real::from_f32(500.0),
            j_max: Real::from_f32(5000.0),
        };
        let p = HybridMotionProfile::compute_with_shape(
            Real::from_f32(100.0), Real::from_f32(10.0), Real::from_f32(10.0),
            math::ZERO, math::ONE, &constraints, JerkShape::Smooth,
        ).unwrap();
        assert_eq!(p.a_o, math::ONE);
        check_invariants(&p, false);
    }

    /// Boundary speeds equal to v_max: exact pure cruise. Equal boundary speeds below v_max:
    /// the profile accelerates (it used to crawl along at v_i).
    #[test]
    fn test_equal_boundary_speeds() {
        let c = Constraints {
            v_max: Real::from_f32(10.0),
            a_max: Real::from_f32(500.0),
            j_max: Real::from_f32(5000.0),
        };
        let cruise = HybridMotionProfile::compute_with_shape(
            Real::from_f32(20.0), Real::from_f32(10.0), Real::from_f32(10.0),
            math::ZERO, math::ZERO, &c, JerkShape::Smooth,
        ).unwrap();
        approx_equal("T", cruise.end_time(), 2.0, 1e-5);
        approx_equal("v", cruise.eval_velocity(Real::from_f32(0.66)), 10.0, 1e-5);
        check_invariants(&cruise, true);

        let c = Constraints { v_max: Real::from_f32(200.0), ..c };
        let fast = HybridMotionProfile::compute_with_shape(
            Real::from_f32(141.4), Real::from_f32(14.4), Real::from_f32(14.4),
            math::ZERO, math::ZERO, &c, JerkShape::Smooth,
        ).unwrap();
        assert!(fast.v_lim > Real::from_f32(150.0), "must accelerate: {:?}", fast);
        assert!(fast.end_time() < Real::from_f32(2.0), "141 mm must not crawl at 14 mm/s: {:?}", fast);
        check_invariants(&fast, true);
    }

    /// Rest residuals must be treated as rest (no endless cruise at ~1e-10 mm/s).
    #[test]
    fn test_rest_residual_speeds() {
        let c = Constraints {
            v_max: Real::from_f32(600.0),
            a_max: Real::from_f32(3600.0),
            j_max: Real::from_f32(7200.0),
        };
        let p = HybridMotionProfile::compute_with_shape(
            Real::from_f32(9.1482), Real::from_f32(2.1316286e-10), Real::from_f32(2.1316282e-10),
            math::ZERO, math::ZERO, &c, JerkShape::Smooth,
        ).unwrap();
        assert!(p.end_time() < Real::from_f32(2.0), "{:?}", p);
        check_invariants(&p, true);
        // Degenerate blend: tiny speed sum with a_i ≈ a_o (≠ 0)
        let p = HybridMotionProfile::compute_with_shape(
            Real::from_f32(9.0), Real::from_f32(0.01), Real::from_f32(0.01),
            Real::from_f32(5.0), Real::from_f32(5.0), &c, JerkShape::Smooth,
        ).unwrap();
        assert!(p.end_time() < Real::from_f32(5.0), "{:?}", p);
    }
}
