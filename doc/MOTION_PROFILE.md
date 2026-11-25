# Hybrid motion profile

Every move (segment) is executed along a **motion profile**: the path position $s(t)$, from $0$ to
the length $D$ of the segment, with its velocity $v = \dot s$, acceleration $a = \dot v$ and jerk
$j = \dot a$. printhor's profile is **hybrid**: each segment is executed with whichever of two
algorithms fits its boundary conditions,

* a **double S-curve** (jerk limited, seven phases) when the segment starts and ends with zero
  acceleration, the time-optimal profile for the limits $v_{max}$, $a_{max}$, $j_{max}$;
* a **polynomial blend** (quintic Hermite velocity) when the look-ahead planner carries a non-zero
  acceleration through its junctions, matched exactly at both ends.

Both have continuous velocity and acceleration, and with the default jerk shape the jerk is also
continuous and zero at both ends of every segment: the acceleration is **C¹ along the whole path**.

`motion/motion_profile_hybrid.rs` (`HybridMotionProfile`), used through the `MotionProfile` trait
by the segment sampler and the stepper task.

## 1. Boundary conditions

A segment is planned (by the motion chaining, `motion/motion_chaining.rs`) with its length $D$,
the entry and exit speeds $v_i$, $v_o$, the entry and exit accelerations $a_i$, $a_o$, and the
limits $v_{max}$, $a_{max}$, $j_{max}$ along its direction (the most restrictive of the axes that
move).

* **Junction speeds**: a junction between two moves is limited by the projection of their
  directions, $\cos\theta$ (none above 90°), and by the largest velocity jump of the corner
  (`M205 X`): the velocity vector changes direction at once there, $|\Delta \vec v| =
  2v\sin(\theta/2) \le \Delta v_{max}$. Then the speeds are propagated backwards and forwards so
  that every segment can brake (and accelerate) within its length.
* **Junction accelerations**: once the junction speeds are final, each junction gets the
  acceleration implied by the velocity along the path, $a = v\,\frac{dv}{ds}$, with $\frac{dv}{ds}$
  estimated by the monotonicity-preserving PCHIP formula (Fritsch & Carlson). It is zero at every
  velocity extremum (a cruise, a peak, a full stop) and keeps the sign of the velocity trend. The
  exit state of a segment is the entry state of the next one, $a_o^{(n)} = a_i^{(n+1)}$: chained
  profiles have a continuous acceleration across the junctions too.

The dispatch:

| Boundary conditions | Profile |
|---|---|
| $a_i \approx a_o \approx 0$ (or rest to rest), $D$ long enough | S-curve |
| $a_i \approx a_o \approx 0$, $D$ too short for the change of speed | polynomial blend (with $a_i = a_o = 0$) |
| $a_i$ or $a_o$ not negligible | polynomial blend |
| blend whose mean speed is negligible (it would last for ever) | S-curve (the junction acceleration jumps, bounded) |

## 2. The double S-curve

The classic seven phases (Biagiotti & Melchiorri, §3.4): increasing acceleration (1), constant
acceleration (2), decreasing acceleration (3), cruise (4), increasing deceleration (5), constant
deceleration (6) and decreasing deceleration (7). Phases 1-3 are an acceleration **ramp**
$v_i \to v_p$, phase 4 a cruise at the peak speed $v_p$ (possibly of zero duration) and phases 5-7
a deceleration ramp $v_p \to v_o$.

### 2.1 Velocity ramps

A jerk-limited change of speed $v_0 \to v_1$, $\Delta v = |v_1 - v_0|$, with zero acceleration at
both ends:

$$
\text{if } \Delta v \ge \frac{a_{max}^2}{j}:\quad T_j = \frac{a_{max}}{j},\quad T = T_j + \frac{\Delta v}{a_{max}},\quad a_{peak} = a_{max}
$$

$$
\text{otherwise}:\quad T_j = \sqrt{\frac{\Delta v}{j}},\quad T = 2T_j,\quad a_{peak} = j\,T_j
$$

($T_j$ the duration of each jerk phase, $T$ of the whole ramp). The acceleration is symmetric in
time, so the ramp covers $d = \frac{v_0 + v_1}{2}\,T$.

### 2.2 The jerk shape

* `JerkShape::Linear` (the classic double S): piecewise-constant jerk, linear acceleration ramps.
  The acceleration is continuous, but its derivative jumps at the phase boundaries.
* `JerkShape::Smooth` (**the default**): the acceleration ramps are the "smoothstep" polynomial,

  $$
  a(\tau) = A\,(3\tau^2 - 2\tau^3), \qquad \tau = t/T_j \in [0, 1]
  $$

  whose jerk $j(\tau) = \frac{6A}{T_j}\tau(1-\tau)$ is continuous and zero at both ends of every
  ramp: the acceleration is **C¹**. Its mean is the same as the linear ramp's ($\int_0^1 = ½$), and
  over each rising/falling pair it covers the same distance, so the durations of §2.1 still apply.
  Its peak jerk is $1.5\,A/T_j$: the profile is planned with the **effective jerk**
  $j = \frac{2}{3}j_{max}$ (the planner uses the same value) so that the real peak is $j_{max}$.

Normalized ramp functions ($\chi$ acceleration, $\psi$ speed change, $\varphi$ distance; "down" ramps
are $1-\chi$, $\tau - \psi$, $\tau^2/2 - \varphi$):

| Shape | $\chi(\tau)$ | $\psi(\tau)$ | $\varphi(\tau)$ | $\varphi(1)$ |
|---|---|---|---|---|
| Linear | $\tau$ | $\tau^2/2$ | $\tau^3/6$ | 1/6 |
| Smooth | $\tau^2(3 - 2\tau)$ | $\tau^3(1 - \tau/2)$ | $\tau^4(\frac14 - \frac{\tau}{10})$ | 3/20 |

so that within a ramp phase of duration $T_j$ starting at $(s_0, v_0)$:

$$
a = A\,\chi(\tau), \qquad v = v_0 + A\,T_j\,\psi(\tau), \qquad s = s_0 + v_0\,T_j\tau + A\,T_j^2\,\varphi(\tau)
$$

### 2.3 Time optimality: the peak speed

* **Feasibility**: the length must cover the direct transition $v_i \to v_o$,
  $D \ge d(v_i \to v_o)$; otherwise the S-curve is infeasible (polynomial blend).
* If $D$ is exactly that transition: no cruise, the peak is $\max(v_i, v_o)$.
* If $d(v_i \to v_{max}) + d(v_{max} \to v_o) \le D$: the peak is $v_{max}$, and the cruise lasts
  $(D - d(v_i\to v_{max}) - d(v_{max}\to v_o))/v_{max}$.
* Otherwise the peak $v_p \in (\max(v_i, v_o), v_{max})$ is the one whose ramps cover exactly $D$:

  $$
  d(v_i \to v_p) + d(v_p \to v_o) = D
  $$

  Instead of Biagiotti & Melchiorri's iterative reduction of $a_{max}$, it is solved **exactly** with
  Newton's method, in the variable $x$ (the jerk phase duration of the ramp from the higher boundary
  speed $v_{lo}$, $v_p = v_{lo} + j x^2$): both ramp distances are convex and increasing in $x$, so
  Newton started above the root decreases monotonically to it (2-4 iterations, from a closed-form
  start near the root of a lower bound $2j x^3 + (3v_{lo} + v_{mn})\,x = D$).

The **end position is exact**: the rounding residual of the solution (a few ppm of $D$) is absorbed
by the cruise phase, never by overshooting.

## 3. The polynomial blend

When the planner carries accelerations through the junctions, the velocity is the quintic Hermite
interpolant of $(v, a, \dot a = 0)$ at both ends, in normalized time $\tau = t/T$:

$$
v(0) = v_i,\ \ \dot v(0) = a_i,\ \ \ddot v(0) = 0, \qquad v(T) = v_o,\ \ \dot v(T) = a_o,\ \ \ddot v(T) = 0
$$

$$
v(\tau) = b_0 + b_1\tau + b_3\tau^3 + b_4\tau^4 + b_5\tau^5
$$

with ($\Delta v = v_o - v_i$)

$$
b_0 = v_i,\quad b_1 = T a_i,\quad b_3 = 10\,\Delta v - 6Ta_i - 4Ta_o,\quad
b_4 = 8Ta_i + 7Ta_o - 15\,\Delta v,\quad b_5 = 6\,\Delta v - 3T(a_i + a_o)
$$

(the position is a sextic, the acceleration a quartic: all evaluated with Horner's rule). The jerk is
zero at both ends: the blend chains with C¹ acceleration with the S-curves and the other blends.

**Duration.** Integrating the velocity,

$$
D = T\,\frac{v_i + v_o}{2} + T^2\,\frac{a_i - a_o}{10}
$$

a quadratic in $T$ whose smallest positive root is computed without cancellation:

$$
T = \frac{2D}{q + \sqrt{\Delta}}, \qquad q = \frac{v_i + v_o}{2},\quad \Delta = q^2 + \frac{2}{5}(a_i - a_o)\,D
$$

It exists while $\Delta \ge 0$, i.e. while the exit acceleration does not exceed

$$
a_o \le a_i + \frac{5\,(v_i + v_o)^2}{8D}
$$

**Admissibility.** The steppers can not reverse inside a segment: the blend velocity must not drop
below zero. Writing the quintic in the Bernstein (Bézier) basis, it is bounded below by its smallest
control point (convex hull property),

$$
c_k = \sum_{i} \frac{\binom{k}{i}}{\binom{5}{i}}\,b_i, \qquad v(\tau) \ge \min_k c_k
$$

a few operations; only when that bound is inconclusive the velocity is sampled (16 points). The
planner uses the same predicate, so the profile computed by the stepper never needs to alter the
planned boundary conditions.

**Trade-off** (a policy of the planner): the blend reproduces the boundary conditions exactly but
does not itself bound the jerk and the acceleration inside the segment. The aggressive look-ahead
plans the junction speeds with the carried acceleration; on short segments a blend can exceed
$j_{max}$ (several times) and occasionally $a_{max}$.

## 4. Evaluation and execution

* **S-curve**: the speed and the position at every phase boundary are cached when the profile is
  computed (closed forms per phase: a whole ramp changes the speed by $A T_j/2$ and covers
  $v_0 T_j + A T_j^2\,\varphi(1)$). At an instant, the phase is found by comparisons and the state
  is evaluated with the ramp functions of §2.2 (a constant-acceleration or cruise phase is a
  parabola or a line).
* **Polynomial**: Horner's rule in $\tau = t/T$ (the coefficients of position and acceleration are
  derived from those of the velocity once).

The **segment sampler** (`motion/segment_sampling.rs`) evaluates the profile in windows of the
micro-segment period (`MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY`), with time kept in integer
microseconds: the windows add up exactly to the duration, the positions are monotone, the last one
is exactly $D$, and a short tail is merged into the previous window. In each window the steps of
every axis are computed from the position at its end (half-step rule), and the step rate ramps
between the velocities of the profile at its ends (second-order DDA,
`micro_segment_interpolation.rs`), offset so that the window covers exactly its steps: the motors
follow a piecewise-linear velocity that interpolates the profile.

The profile is computed in the configured arithmetic (`f32`, `f64` or 128-bit fixed point): the
divisions are hoisted (one reciprocal per ramp constant) because each one is a soft-float call on
the Cortex-M0+ boards.

## 5. Verification

* `motion_profile_hybrid.rs` tests:
  * `smooth_scurve_is_c1_and_within_limits`: with the smooth jerk the acceleration is C¹ (the jerk
    continuous and zero at every phase boundary), the limits are respected, and the duration and
    displacement are those of a linear profile planned with $\frac{2}{3}j_{max}$;
  * `randomized_scurve_sweep`: a deterministic random sweep; the computation never fails for
    physically valid inputs and the S-curves keep their invariants (the limits are only guaranteed
    for the S-curve);
  * `test_blend_boundary_conditions`, `test_consecutive_blends_continuity`: the blends reproduce
    their boundary conditions exactly, with zero jerk at both ends, and are continuous in velocity,
    acceleration and jerk at a shared junction;
  * `test_blend_feasible_bcs_no_clamping`, `test_max_opposite_accels_feasibility`,
    `test_large_opposite_accels_short_decel_segment`: feasible boundary conditions are never
    modified, and the entry state never is;
  * `test_equal_boundary_speeds`, `test_rest_residual_speeds`: a pure cruise at $v_{max}$, and
    equal boundary speeds below it accelerate; rest residuals are rest;
  * `peak_velocity_solver_converges_fast`, `peak_solver_iterations_distribution`: the peak solver
    takes a few iterations and the end position is exact.
* `motion_chaining.rs` tests: `test_acceleration_continuity_after_chaining_collinear`,
  `test_chained_profiles_are_continuous` (continuity across the junctions,
  $a_o^{(n)} = a_i^{(n+1)}$), `test_admissibility_covers_displacement_correction` (admissible
  blends), `test_windowed_chaining_matches_full_chaining` (the windowed look-ahead against the full
  chaining).
* `segment_sampling.rs`: `windows_cover_the_whole_segment_exactly`.
* `s_plot`: plots the profile of a whole job (`pen-test.gcode` by default): position, velocity,
  acceleration, the motion in machine axes and the spectrum of the acceleration the motors follow.

The per-phase derivation of the classic (linear jerk) double S, with initial and final
accelerations, is in [`printhor/src/bin/motion/README.md`](../printhor/src/bin/motion/README.md).

## References

1. L. Biagiotti and C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots*,
   Springer, 2008, §3.4 (double S velocity profile), https://doi.org/10.1007/978-3-540-85629-0.
2. P. Lambrechts, M. Boerlage and M. Steinbuch, "Trajectory planning and feedforward design for
   electromechanical motion systems," *Control Engineering Practice*, vol. 13, no. 2, pp. 145-157,
   2005.
3. S. Macfarlane and E. A. Croft, "Jerk-bounded manipulator trajectory planning: design for
   real-time applications," *IEEE Transactions on Robotics and Automation*, vol. 19, no. 1,
   pp. 42-52, 2003 (quintic splines).
4. F. N. Fritsch and R. E. Carlson, "Monotone piecewise cubic interpolation," *SIAM Journal on
   Numerical Analysis*, vol. 17, no. 2, pp. 238-246, 1980 (PCHIP).
5. R. T. Farouki, "The Bernstein polynomial basis: a centennial retrospective," *Computer Aided
   Geometric Design*, vol. 29, no. 6, pp. 379-419, 2012 (convex hull bound).
