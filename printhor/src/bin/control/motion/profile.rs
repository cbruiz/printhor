//! The Motion profile trait and current implementation: Trajectory with Double S Velocity Profile \[1\]
//!
//! \[1\] Biagiotti, L., Melchiorri, C.: Trajectory Planning for Automatic Machines and Robots. Springer, Heidelberg (2008). [DOI:10.1007/978-3-540-85629-0](https://doi.org/10.1007/978-3-540-85629-0)

use crate::control::CodeExecutionFailure;
use crate::hwa;
use hwa::math;
use hwa::math::*;

/// The `MotionProfile` trait provides methods for evaluating motion profiles.
pub trait MotionProfile {
    /// Returns the end time of the motion profile.
    fn end_time(&self) -> Real;

    /// Returns the end position of the motion profile.
    fn end_pos(&self) -> Real;

    /// Evaluates the position at a given time `t` within the motion profile.
    ///
    /// # Parameters
    ///
    /// - `t`: The time at which to evaluate the position.
    ///
    /// # Returns
    ///
    /// An `Option` containing a tuple with:
    /// - The evaluated position as `Real`.
    /// - A status as `u8` (implementation-specific).
    fn eval_position(&self, t: Real) -> Option<Real>;
}

/// Struct representing the motion profile configuration.
///
/// # Fields
///
/// * `constraints` - The motion profile constraints.
/// * `times` - The time parameters for different phases of the motion profile.
/// * `initial_velocity` - Initial velocity for the motion profile (in mm/s).
/// * `final_velocity` - Final velocity for the motion profile (in mm/s).
/// * `error_correction` - An optional error correction factor.
pub struct ProfileConfig {
    /// The motion profile constraints.
    ///
    /// # Units
    ///
    /// - `v_max`: mm/s
    /// - `a_max`: mm/s²
    /// - `j_max`: mm/s³
    pub constraints: Constraints,

    /// The time parameters of the motion profile.
    ///
    /// # Units
    ///
    /// - `t_j1`: s
    /// - `t_a`: s
    /// - `t_v`: s
    /// - `t_d`: s
    /// - `t_j2`: s
    pub times: Times,

    /// Initial velocity for the motion profile.
    ///
    /// # Units
    ///
    /// - Initial velocity: mm/s
    pub initial_velocity: Real,

    /// Final velocity for the motion profile.
    ///
    /// # Units
    ///
    /// - Final velocity: mm/s
    pub final_velocity: Real,

    /// An optional error correction factor.
    ///
    /// # Units
    ///
    /// - Error correction factor: Implementation-specific units
    pub error_correction: Option<Real>,
}

/// The `Constraints` struct represents the limits of the motion profile in terms of velocity, acceleration, and jerk.
///
/// # Fields
///
/// * `v_max` - The maximum velocity (in mm/s). This defines the upper limit of speed that can be achieved.
/// * `a_max` - The maximum acceleration (in mm/s²). This represents the highest rate of change of velocity that is permissible.
/// * `j_max` - The maximum jerk (in mm/s³). This is the maximum rate of change of acceleration.
#[derive(Clone, Copy)]
pub struct Constraints {
    /// The maximum velocity (v_max) (in mm/s).
    /// This defines the upper limit of speed that can be achieved.
    pub v_max: Real,

    /// The maximum acceleration (a_max) (in mm/s²).
    /// This represents the highest rate of change of velocity that is permissible.
    pub a_max: Real,

    /// The maximum jerk (j_max) (in mm/s³).
    /// This is the maximum rate of change of acceleration.
    pub j_max: Real,
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

/// Struct representing the time parameters for different phases of the S-curve motion profile.
///
/// This struct is used to define the specific time segments allocated to different phases of the S-curve.
///
/// # Fields
///
/// * `t_j1` - Time spent in the first jerk phase (in seconds).
/// * `t_a` - Time spent in the acceleration phase (in seconds).
/// * `t_v` - Time spent at constant velocity (in seconds).
/// * `t_d` - Time spent in the deceleration phase (in seconds).
/// * `t_j2` - Time spent in the second jerk phase (in seconds).
pub struct Times {
    /// Time spent in the first jerk phase (in seconds).
    pub t_j1: Real,

    /// Time spent in the acceleration phase (in seconds).
    pub t_a: Real,

    /// Time spent at constant velocity (in seconds).
    pub t_v: Real,

    /// Time spent in the deceleration phase (in seconds).
    pub t_d: Real,

    /// Time spent in the second jerk phase (in seconds).
    pub t_j2: Real,
}

/// Struct representing the S-curve motion profile with various parameters.
///
/// The S-curve motion profile is used to define the motion of an object
/// where the transition of velocity includes a jerk phase, an acceleration
/// phase, a constant velocity phase, a deceleration phase, and a final jerk
/// phase to achieve smooth motion.
///
/// This profile is characterized by the following parameters:
///
/// # Fields
///
/// * `t_j1` - Time spent in the first jerk phase (in seconds).
/// * `t_a` - Time spent in the acceleration phase (in seconds).
/// * `t_v` - Time spent at constant velocity (in seconds).
/// * `t_d` - Time spent in the deceleration phase (in seconds).
/// * `t_j2` - Time spent in the second jerk phase (in seconds).
/// * `v_0` - Initial velocity (in mm/s).
/// * `v_1` - Final velocity (in mm/s).
/// * `j_max` - Maximum jerk (in mm/s³).
/// * `a_lim_a` - Acceleration limit during the acceleration phase (in mm/s²).
/// * `a_lim_d` - Deceleration limit during the deceleration phase (in mm/s²).
/// * `v_lim` - Velocity limit (in mm/s).
/// * `q1` - Displacement or position to be achieved (in mm).
/// * `constraints` - Constraints applied to the motion profile.
/// * `cache` - Cached values for optimization and repeated calculations.
pub struct SCurveMotionProfile {
    /// Time spent in the first jerk phase.
    pub t_j1: Real,

    /// Time spent in the acceleration phase.
    pub t_a: Real,

    /// Time spent at constant velocity.
    pub t_v: Real,

    /// Time spent in the deceleration phase.
    pub t_d: Real,

    /// Time spent in the second jerk phase.
    pub t_j2: Real,

    /// Initial velocity.
    pub v_0: Real,

    /// Final velocity.
    pub v_1: Real,

    /// Maximum jerk.
    pub j_max: Real,

    /// Acceleration limit during the acceleration phase.
    pub a_lim_a: Real,

    /// Deceleration limit during the deceleration phase.
    pub a_lim_d: Real,

    /// Velocity limit.
    pub v_lim: Real,

    /// Displacement or position to be achieved.
    pub q1: Real,

    /// Constraints applied to the motion profile.
    pub constraints: Constraints,

    /// Cached values for optimization and repeated calculations.
    pub cache: Cache,
}

#[allow(unused)]
impl SCurveMotionProfile {
    /// Compute the S-curve motion profile.
    ///
    /// # Arguments
    ///
    /// * `q_1` - Displacement or position to be achieved (in mm).
    /// * `v_0` - Initial velocity (in mm/s).
    /// * `v_1` - Final velocity (in mm/s).
    /// * `constraints` - [Constraints] applied to the motion profile.
    /// * `error_correction` - Flag to enable error correction.
    ///
    /// # Returns
    ///
    /// `Result<SCurveMotionProfile, CodeExecutionFailure>` - The computed S-curve motion profile or an error.
    ///
    /// # Description
    ///
    /// This method computes the S-curve motion profile considering the initial and final velocities,
    /// the displacement to be achieved, and the provided constraints. The method ensures that the
    /// computed trajectory respects the given constraints and returns a result containing the motion
    /// profile or an error if the motion cannot be achieved with the specified constraints.
    ///
    /// # Examples
    ///
    /// ```
    /// let constraints = Constraints {
    ///     v_max: Real::from_f32(10.0),
    ///     a_max: Real::from_f32(2.0),
    ///     j_max: Real::from_f32(1.0),
    /// };
    /// let profile = SCurveMotionProfile::compute(Real::from_f32(20.0), Real::from_f32(0.0), Real::from_f32(5.0), &constraints, false)?;
    /// println!("{}", profile);
    /// ```
    ///
    pub fn compute(
        q_1: Real,
        v_0: Real,
        v_1: Real,
        constraints: &Constraints,
        error_correction: bool,
    ) -> Result<SCurveMotionProfile, CodeExecutionFailure> {
        if constraints.v_max.is_negligible()
            || constraints.a_max.is_negligible()
            || constraints.j_max.is_negligible()
        {
            hwa::warn!("Unable to perform movement: constraints are unset");
            return Err(CodeExecutionFailure::NumericalError);
        }
        // Clamp v_ma to be equal or higher than v_0 and v_1
        let v_min = v_0.max(v_1);
        let mut v_max = v_min.max(constraints.v_max);

        // First, compute the displacement

        // [[1]] First necessary to verify whether a trajectory can be actually performed or not.
        // As a matter of fact, there are several cases in which a trajectory cannot be computed
        // with the given constraints. For example, if the desired displacement h is small with
        // respect to the difference between the initial and final velocities v0 and v1, it might be not possible
        // to change the velocity (with the given limits on jerk and acceleration), while accomplishing the displacement h.

        let j_max_inv = constraints.j_max.recip();
        let t_jmax = constraints.a_max * j_max_inv;
        // [eq 3.17]
        let t_jstar =
            Real::vmin((((v_1 - v_0).abs()) * j_max_inv).sqrt(), Some(t_jmax)).unwrap_or(ZERO);

        // [[1]] If t_jstar = amax/jmax, the acceleration reaches its maximum value and a segment with zero jerk may exist.

        if (t_jstar - t_jmax).is_negligible() {
            // the acceleration reaches its maximum value and a segment with zero jerk may exist
            hwa::debug!("The acceleration reaches its maximum value");
        } else {
            hwa::debug!("The acceleration does not reach its maximum value");
        }

        hwa::debug!("t_jstar = {:?} t_jmax = {:?}", t_jstar, t_jmax);

        // [[1]] (3.18)
        let q_lim = if t_jstar < t_jmax {
            t_jstar * (v_0 + v_1) // H
        } else {
            (HALF * (v_0 + v_1)) * (t_jstar + ((v_1 - v_0).abs() * j_max_inv))
        };

        let not_feasible = q_1 <= q_lim;

        if not_feasible {
            #[cfg(feature = "verbose-timings")]
            hwa::warn!("[SCurveMotionProfile] Movement NOT FEASIBLE. Performing unconstrained parabolic blends");

            // y := j_max
            // e1: q1 = (y * t^3) / 6 + v0 * t
            // e2: v1 = v0 + y * t^2 / 2 => 2*(v1 - v0) = y * t^2 => y = 2*(v1-v0) / t^2
            // e1|e2: q1 = ((2*(v1-v0) / t^2) * t^3) / 6 + v0 * t
            // t = 3*q1 / 2*v0 + v1 | v0+v1 >0 and q1 > 0

            // v1(t) = v0 + j_max * t^2 / 2
            // s1(t) = (y * t^3) / 6 + v0*t
            // v2(t) = 2*v1(t_j1) - v0 - j_max * (t - 2t_j1)^2 / 2
            //  2*v0 + j_max * t_j1^2 / 2
            // v2(t) = 2*(v0 + j_max * t^2 / 2) - v0 - j_max * (t - 2t_j1)^2 / 2

            // e1: q1 = (y * t^3) / 6 + v0*t
            // e2: q1 = (- y * t^3) / 6 + a*t^2*T_j1 - a * t * T_j1^2 + t*v_0
            // e3: T_j1 = t/2
            // y = q1/x

            // solving -1/6 a x^3 + d/x *x^3/2 - d/x* x^3/4 + v x = d for x
            // x = (2 (sqrt(d^2 + v^2) - v))/d y d!=0 y a = 0

            // v1(t) = v0 + a * (t^2 / 2)
            // s1(t) = (a * t^3) / 6 + v0 * t

            // v2(t) = v_1 - a * (((T - t)^2)/2)
            // s2(t) = v_{1} * t - (1/6) * a * t (t^2 - 3*t*T + 3*T^2)

            // q_b = Int{T/2, T} ( v_1 - a * (((T - t)^2)/2) ) = s2(T) - s2(T/2)
            // q_b = v_1 * T - (1/6) * a * T (T^2 - 3*T*T + 3*T^2) - ( v_1 * T/2 - (1/6) * a * T/2 ((T/2)^2 - 3*(T/2)*T + 3*T^2) )
            //   = 1/2 T v_1 - (a T^3)/48
            //
            // q_a = Int{0, T/2} (v_0 + a * (t^2 / 2)) = s1(T/2) - s1(0)
            // q_a = (a * (T/2)^3) / 6 + v_0 * T/2 - ( (a * 0^3) / 6 + v_0 * 0 )
            // = 1/48 T (a T^2 + 24 * v_0)

            // s = q_a + q_b = 1/2 * T * v_1 - (a * T^3) / 48 + 1/48 * T * (-a T^2 + 24 * v_0) =
            //   = 1/2 T (v_0 + v_1)
            // T = (2 * s) / (v_0 + v_1)

            // q_a = 1/48 T (a T^2 + 24 * v_0)
            // q_b = 1/2 T v_1 - (a T^3)/48
            // simplifiying q_a = 1/48 ((2 * s) / (v_0 + v_1)) (a ((2 * s) / (v_0 + v_1))^2 + 24 * v_0)
            // q_a = (s (a s^2 + 6 v_0 (v_0 + v_1)^2))/(6 (v_0 + v_1)^3)
            // simplifiying q_b = 1/2 ((2 * s) / (v_0 + v_1)) v_1 - (a ((2 * s) / (v_0 + v_1))^3)/48
            // q_b = (s v_1)/(v_0 + v_1) - (a s^3)/(6 (v_0 + v_1)^3)

            // (q_1 (q_1 q_1^2 + 6 v_0 (v_0 + v_1)^2))/(6 (v_0 + v_1)^3)
            // (q_1 v_1)/(v_0 + v_1) - (a q_1^3)/(6 (v_0 + v_1)^3)

            // Compute the time need to accel
            let t_a = TWO * (q_1 / (v_0 + v_1));

            // Compute the accel
            let a = (v_1 - v_0) / t_a;
            let t_j1 = HALF * t_a;

            // Constructively define the two parabolas:
            // The first one (smooth accel from v_0):
            // v_{1}(t) := a_{v1} * (t - 0)^2 + v_{0}
            // with a_{v_1} as:
            //  a_{1} = \frac{a \frac{T}{2} - 0}{(0 - \frac{T}{2})^2}
            // The second one (smooth decel to v_1); TBD
            //  a_{2} = \frac{-a \frac{T}{2} - 0}{(0 - \frac{T}{2})^2}

            let a_1 = (a * t_j1) / (t_j1 * t_j1);

            // Finally, reformulating to meet the paper convention it is trivial to determine that:
            // In \dot{q}(t) = v_{0} + j_{max} \frac{t^2}{2}:
            // a_{v1} = j_max / 2, hence j_max = 2 * a_{v1}

            let j_max = TWO * a_1;

            let mut profile = SCurveMotionProfile {
                t_j1,
                t_a,
                t_v: math::ZERO,
                t_d: math::ZERO,
                t_j2: math::ZERO,
                v_0: v_0,
                v_1: v_1,
                j_max,
                a_lim_a: math::ZERO,
                a_lim_d: math::ZERO,
                v_lim: v_1.max(v_0),
                q1: q_1,
                cache: Cache::default(),
                constraints: *constraints,
            };
            profile.compute_cache();
            if error_correction {
                let final_pos = profile.s_i7(&profile.i7_end());
                let delta_e = profile.q1 - final_pos;
                profile
                    .extend(delta_e)
                    .map_err(|_| CodeExecutionFailure::ERR)?;
            }
            return Ok(profile);
            /*

            if t_jstar < t_jmax {
                hwa::error!(" => cannot accel to travel {} mm from {} to {} ({}). Will take {} mm ({} * (v0+v1))", q_1, v_0, v_1, (v_1 - v_0).abs(), q_lim, t_jstar);
            }
            else {
                hwa::error!("=> cannot accel to travel {} mm from {} to {}. Will take {} mm (((v_0 + v_1) / TWO) * ({} + ((v_1 - v_0).abs() / {})))", q_1, v_0, v_1, q_lim, t_jstar, constraints.a_max);
            }
            return Err(CodeExecutionFailure::ERR)

             */
        } else {
            //Procedure:
            // Assuming that vmax and amax are reached (*case_1*) compute the time intervals:
            // if (vmax − v0) * jmax < a_max^2 =⇒ a_max is not reached (3.19) and hence:
            //      T_j1 = sqrt( (v_max - v_0) / j_max ) , Ta = 2 * T_j1,
            // else
            //      T_j1 = a_max / j_max, T_a = T_j1 + (v_max - v_0) / a_max
            // if (vmax − v1) * jmax < a_max^2 =⇒ a_max is not reached (3.20) and hence:
            //      T_j2 = sqrt( (v_max - v_1) / j_max ) , Td = 2 * T_j2,
            // else
            //      T_j1 = a_max / j_max, T_a = T_j2 + (v_max - v_1) / a_max

            let mut a_max = constraints.a_max;
            let mut a_max_inv = a_max.recip();
            let mut a_max_squared = a_max * a_max;
            // aj_ratio = a_max / j_max
            let mut aj_ratio = a_max * j_max_inv;

            let mut prev_vmax = v_max;
            let gamma = Real::from_f32(0.9);
            loop {
                match Self::compute_case1(
                    q_1,
                    v_0,
                    v_1,
                    v_max,
                    a_max,
                    constraints.j_max,
                    aj_ratio,
                    a_max_squared,
                ) {
                    Ok(times) => {
                        if times.t_v <= ZERO {
                            // TODO: lagrange multipliers or something better to find a solution with less iterations

                            let t_j = aj_ratio;

                            let sqrt_delta =
                                ((a_max * a_max * a_max * a_max * j_max_inv * j_max_inv)
                                    + (TWO * ((v_0 * v_0) + (v_1 * v_1))
                                        + (a_max * ((FOUR * q_1) - (TWO * (t_j) * (v_0 + v_1))))))
                                    .sqrt()
                                    .unwrap();
                            let aj = (a_max * a_max) * j_max_inv;
                            let mut t_a_2 = ((aj - (v_0 + v_0) + sqrt_delta) * HALF * a_max_inv);
                            let mut t_d_2 = ((aj - (v_1 + v_1) + sqrt_delta) * HALF * a_max_inv);
                            let mut t_j1 = t_j;
                            let mut t_j2 = t_j;

                            if t_a_2 < math::ZERO || t_d_2 < math::ZERO {
                                // it may happen that Ta or Td becomes negative. In this case, only one of the acceleration or deceleration phase is necessary
                                let qd = (q_1 / (v_0 + v_1));
                                let vsum_sq = (v_0 + v_1) * (v_0 + v_1);
                                if t_a_2 < math::ZERO {
                                    t_a_2 = math::ZERO;
                                    t_j1 = math::ZERO;

                                    t_d_2 = qd * TWO;
                                    t_j2 = ((constraints.j_max * q_1)
                                        - (constraints.j_max
                                            * (constraints.j_max * q_1 * q_1
                                                + (v_1 + v_0) * (v_1 + v_0) * (v_1 - v_0)))
                                            .sqrt()
                                            .unwrap())
                                        / (constraints.j_max * (v_1 + v_0));
                                } else if t_d_2 < math::ZERO {
                                    t_d_2 = math::ZERO;
                                    t_j2 = math::ZERO;
                                    t_a_2 = qd * TWO;
                                    t_j1 = ((constraints.j_max * q_1)
                                        - (constraints.j_max
                                            * (constraints.j_max * q_1 * q_1
                                                + (v_1 + v_0) * (v_1 + v_0) * (v_0 - v_1)))
                                            .sqrt()
                                            .unwrap())
                                        / (constraints.j_max * (v_1 + v_0));
                                }
                            } else {
                                t_j1 = t_j;
                                t_j2 = t_j;
                            }

                            /*
                            #[cfg(feature = "native")]
                            std::println!("gamma={} -> a={} -> t_j1={}, t_a_2={} t_j2={} t_d_2={}",
                                          gamma.rdp(4),
                                a_max, t_j1, t_a_2, t_j2, t_d_2
                            );
                            */

                            if t_a_2 >= (t_j1 + t_j1) && t_d_2 >= (t_j2 + t_j2) {
                                let a_lim_a = constraints.j_max * t_j1;
                                let a_lim_d = -constraints.j_max * t_j2;

                                let v_lim = v_0 + (t_a_2 - t_j1) * a_lim_a;
                                let t = t_a_2 + math::ZERO + t_d_2;
                                let mut profile = SCurveMotionProfile {
                                    t_j1,
                                    t_a: t_a_2,
                                    t_v: math::ZERO,
                                    t_d: t_d_2,
                                    t_j2,
                                    v_0: v_0,
                                    v_1: v_1,
                                    j_max: constraints.j_max,
                                    a_lim_a,
                                    a_lim_d,
                                    v_lim,
                                    q1: q_1,
                                    cache: Cache::default(),
                                    constraints: *constraints,
                                };
                                profile.compute_cache();
                                if error_correction {
                                    let final_pos =
                                        profile.s_i7(&profile.i7_end()) + Real::epsilon();
                                    let delta_e = profile.q1 - final_pos;
                                    profile
                                        .extend(delta_e)
                                        .map_err(|_| CodeExecutionFailure::ERR)?;
                                }
                                return Ok(profile);
                            }
                            a_max *= gamma;
                            a_max_inv = a_max.recip();
                            a_max_squared = a_max * a_max;
                            aj_ratio = a_max * j_max_inv;
                            if a_max < Real::from_f32(0.1) {
                                return if (v_1 - v_0).abs().is_negligible() {
                                    let t_v = TWO * (q_1 / (v_0 + v_1));
                                    let mut profile = SCurveMotionProfile {
                                        t_j1: math::ZERO,
                                        t_a: math::ZERO,
                                        t_v,
                                        t_d: math::ZERO,
                                        t_j2: math::ZERO,
                                        v_0: v_0,
                                        v_1: v_1,
                                        j_max: constraints.j_max,
                                        a_lim_a: math::ZERO,
                                        a_lim_d: math::ZERO,
                                        v_lim: v_1,
                                        q1: q_1,
                                        cache: Cache::default(),
                                        constraints: *constraints,
                                    };
                                    profile.compute_cache()?;
                                    if error_correction {
                                        let final_pos = profile.s_i7(&profile.i7_end());
                                        let delta_e = profile.q1 - final_pos;
                                        profile
                                            .extend(delta_e)
                                            .map_err(|_| CodeExecutionFailure::ERR)?;
                                    }
                                    Ok(profile)
                                } else {
                                    // Compute the time need to accel
                                    let t_a = TWO * (q_1 / (v_0 + v_1));

                                    // Compute the accel
                                    let a = (v_1 - v_0) / t_a;
                                    let t_j1 = HALF * t_a;

                                    // Constructively define the two parabolas:
                                    // The first one (smooth accel from v_0):
                                    // v_{1}(t) := a_{v1} * (t - 0)^2 + v_{0}
                                    // with a_{v_1} as:
                                    //  a_{1} = \frac{a \frac{T}{2} - 0}{(0 - \frac{T}{2})^2}
                                    // The second one (smooth decel to v_1); TBD
                                    //  a_{2} = \frac{-a \frac{T}{2} - 0}{(0 - \frac{T}{2})^2}

                                    let a_1 = (a * t_j1) / (t_j1 * t_j1);

                                    // Finally, reformulating to meet the paper convention it is trivial to determine that:
                                    // In \dot{q}(t) = v_{0} + j_{max} \frac{t^2}{2}:
                                    // a_{v1} = j_max / 2, hence j_max = 2 * a_{v1}

                                    let j_max = TWO * a_1;

                                    let mut profile = SCurveMotionProfile {
                                        t_j1,
                                        t_a,
                                        t_v: math::ZERO,
                                        t_d: math::ZERO,
                                        t_j2: math::ZERO,
                                        v_0: v_0,
                                        v_1: v_1,
                                        j_max,
                                        a_lim_a: math::ZERO,
                                        a_lim_d: math::ZERO,
                                        v_lim: v_1.max(v_0),
                                        q1: q_1,
                                        cache: Cache::default(),
                                        constraints: *constraints,
                                    };
                                    profile.compute_cache();
                                    if error_correction {
                                        let final_pos = profile.s_i7(&profile.i7_end());
                                        let delta_e = profile.q1 - final_pos;
                                        profile
                                            .extend(delta_e)
                                            .map_err(|_| CodeExecutionFailure::ERR)?;
                                    }
                                    Ok(profile)
                                };
                            }
                            /*
                            let tv_excess = times.t_v.abs();
                            if tv_excess > times.t_a + times.t_d {
                                panic!("Unable to handle this")
                            }
                            let ta = times.t_a * Real::from_f32(0.1);
                            let td = times.t_d * Real::from_f32(0.1);
                            let tj1 = times.t_j1 * Real::from_f32(0.1);
                            let tj2 = times.t_j2 * Real::from_f32(0.1);
                            let a_lim_a = constraints.j_max * tj1;
                            let a_lim_d = constraints.j_max * tj2;
                            let v_lim0 = v_0 + (ta - tj1) * a_lim_a;
                            let v_lim1 = v_1 + (td - tj2) * a_lim_d;
                            v_max = v_lim0.max(v_lim1);
                            let vred = prev_vmax - v_max;
                            if vred < Real::from_lit(1,1) {

                                let mut profile = SCurveMotionProfile {
                                    t_j1: times.t_j1,
                                    t_a: times.t_a,
                                    t_v: times.t_v,
                                    t_d: times.t_d,
                                    t_j2: times.t_j2,
                                    v_0: v_0,
                                    v_1: v_1,
                                    j_max: constraints.j_max,
                                    a_lim_a,
                                    a_lim_d,
                                    v_lim: v_max,
                                    q1: q_1,
                                    cache: Cache::default(),
                                    constraints: *constraints,
                                };
                                cfg_if::cfg_if! {
                                    if #[cfg(feature="verbose-timings")] {
                                        hwa::debug!("Motion plan computed in {} us", _t0.elapsed().as_micros());
                                    }
                                }
                                profile.compute_cache();
                                if error_correction {
                                    let final_pos = profile.s_i7(&profile.i7_end());
                                    let delta_e = profile.q1 - final_pos;
                                    profile.extend(delta_e).map_err(|_| CodeExecutionFailure::ERR)?;

                                }
                                return Ok(profile);
                            }
                            else {
                                prev_vmax = v_max;
                            }
                            */

                            /*
                            let t_a = TWO * (q_1 / (v_0 + v_1));

                            // Compute the accel
                            let a = (v_1 - v_0) / t_a;
                            let t_j1 = HALF * t_a;


                            // Constructively define the two parabolas:
                            // The first one (smooth accel from v_0):
                            // v_{1}(t) := a_{v1} * (t - 0)^2 + v_{0}
                            // with a_{v_1} as:
                            //  a_{1} = \frac{a \frac{T}{2} - 0}{(0 - \frac{T}{2})^2}
                            // The second one (smooth decel to v_1); TBD
                            //  a_{2} = \frac{-a \frac{T}{2} - 0}{(0 - \frac{T}{2})^2}

                            let a_1 = (a * t_j1) / (t_j1 * t_j1);

                            // Finally, reformulating to meet the paper convention it is trivial to determine that:
                            // In \dot{q}(t) = v_{0} + j_{max} \frac{t^2}{2}:
                            // a_{v1} = j_max / 2, hence j_max = 2 * a_{v1}

                            let j_max = TWO * a_1;

                            let mut profile = SCurveMotionProfile {
                                t_j1,
                                t_a,
                                t_v: math::ZERO,
                                t_d: math::ZERO,
                                t_j2: math::ZERO,
                                v_0: v_0,
                                v_1: v_1,
                                j_max,
                                a_lim_a: math::ZERO,
                                a_lim_d: math::ZERO,
                                v_lim: v_1,
                                q1: q_1,
                                cache: Cache::default(),
                                constraints: *constraints,
                            };
                            cfg_if::cfg_if! {
                                if #[cfg(feature="verbose-timings")] {
                                    hwa::debug!("Motion plan computed in {} us", _t0.elapsed().as_micros());
                                }
                            }
                            profile.compute_cache();
                            if error_correction {
                                let final_pos = profile.s_i7(&profile.i7_end());
                                let delta_e = profile.q1 - final_pos;
                                profile.extend(delta_e).map_err(|_| CodeExecutionFailure::ERR)?;

                            }
                            return Ok(profile);

                             */
                        } else {
                            // H
                            hwa::debug!(
                                "OK at {:?} -> ta = {:?} td = {:?} tv = {:?}",
                                v_max,
                                times.t_a,
                                times.t_d,
                                times.t_v
                            );
                            let a_lim_a = constraints.j_max * times.t_j1;
                            let a_lim_d = constraints.j_max * times.t_j2;
                            let v_lim = v_0 + (times.t_a - times.t_j1) * a_lim_a;
                            let t = times.t_a + times.t_v + times.t_d;
                            let mut profile = SCurveMotionProfile {
                                t_j1: times.t_j1,
                                t_a: times.t_a,
                                t_v: times.t_v,
                                t_d: times.t_d,
                                t_j2: times.t_j2,
                                v_0: v_0,
                                v_1: v_1,
                                j_max: constraints.j_max,
                                a_lim_a,
                                a_lim_d,
                                v_lim,
                                q1: q_1,
                                cache: Cache::default(),
                                constraints: *constraints,
                            };
                            profile.compute_cache();
                            if error_correction {
                                let final_pos = profile.s_i7(&profile.i7_end());
                                let delta_e = profile.q1 - final_pos;
                                profile
                                    .extend(delta_e)
                                    .map_err(|_| CodeExecutionFailure::ERR)?;
                            }
                            return Ok(profile);
                        }
                    }
                    Err(_e) => return Err(_e),
                }
            }
        }
    }

    fn compute_case1(
        q_1: Real,
        v_0: Real,
        v_1: Real,
        v_max: Real,
        a_max: Real,
        j_max: Real,
        aj_ratio: Real,
        amax_squared: Real,
    ) -> Result<Times, CodeExecutionFailure> {
        let a_max_not_reached = (v_max - v_0) * j_max < amax_squared;
        let a_min_not_reached = (v_max - v_1) * j_max < amax_squared;

        let j_max_inv = if a_max_not_reached || a_min_not_reached {
            j_max.recip()
        } else {
            math::ZERO
        };

        let a_max_inv = if !a_max_not_reached || !a_min_not_reached {
            a_max.recip()
        } else {
            math::ZERO
        };

        let (t_j1, t_a) = if a_max_not_reached {
            let t_j = ((v_max - v_0).max(math::ZERO) * j_max_inv)
                .sqrt()
                .ok_or(CodeExecutionFailure::NumericalError)?;
            (t_j, t_j + t_j)
        } else {
            let t_j = aj_ratio;
            (t_j, t_j + (v_max - v_0).max(math::ZERO) * a_max_inv)
        };
        let (t_j2, t_d) = if a_min_not_reached {
            let t_j = ((v_max - v_1).max(math::ZERO) * j_max_inv)
                .sqrt()
                .ok_or(CodeExecutionFailure::NumericalError)?;
            (t_j, t_j + t_j)
        } else {
            let t_j = aj_ratio;
            (t_j, t_j + (v_max - v_1).max(math::ZERO) * a_max_inv)
        };

        // Efficently compute: (q_1 / v_max) - (t_a / 2) * (1 + (v_0 / v_max)) - (t_d / 2) * (1 + (v_1 / v_max))
        // ... with maximum possible numerical precision

        let v_max_inv = v_max.recip();
        let half_ta = t_a * HALF;
        let term1 = half_ta + half_ta * (v_0 * v_max_inv);
        let half_td = t_d * HALF;
        let term2 = half_td + half_td * (v_1 * v_max_inv);

        let t_v = (q_1 * v_max_inv) - term1 - term2;
        Ok(Times {
            t_j1,
            t_a,
            t_v,
            t_j2,
            t_d,
        })
    }

    pub fn params_dump(&self) {
        hwa::debug!("Params:\nq_{{1}} = {:?}\nv_{{0}} = {:?}\nv_{{1}} = {:?}\nv_{{max}} = {:?}\na_{{max}} = {:?}\nj_{{max}} = {:?}\nT_{{j1}} = {:?}\nT_{{a}} = {:?}\nT_{{v}} = {:?}\nT_{{j2}} = {:?}\nT_{{d}} = {:?}\na_{{lima}} = {:?}\na_{{limd}} = {:?}\nv_{{lim}} = {:?}",
            self.q1, self.v_0, self.v_1,
            self.constraints.v_max,
            self.constraints.a_max,
            self.constraints.j_max,
            self.t_j1, self.t_a,
            self.t_v,
            self.t_j2, self.t_d,
            self.a_lim_a, self.a_lim_d, self.v_lim,
        );

        hwa::debug!("V_{{1}}(t) = v_{{0}} + j_{{max}} * \\frac{{t^2}}{{2}}");
        hwa::debug!("V_{{2}}(t) = v_{{0}} + a_{{lima}} * (t - \\frac{{T_{{j1}}}}{{2}})");
        hwa::debug!("V_{{3}}(t) = v_{{lim}} - j_{{max}} * \\frac{{(T_{{a}} - t)^2}}{{2}}");
        hwa::debug!("V_{{4}}(t) = v_{{lim}}");
        hwa::debug!(
            "V_{{5}}(t) = v_{{lim}} - j_{{max}} * \\frac{{(t - T_{{a}} - T_{{v}})^2}}{{2}}"
        );
        hwa::debug!("V_{{6}}(t) = v_{{lim}} - a_{{limd}} * (t - T_{{a}} - T_{{v}} - \\frac{{T_{{j2}}}}{{2}})");
        hwa::debug!(
            "V_{{7}}(t) = v_{{1}} + j_{{max}} * \\frac{{(t - T_{{a}} - T_{{v}} - T_{{d}})^2}}{{2}}"
        );
        hwa::debug!("V(t) = if( t < 0, v_{{0}}, if( 0 <= t < T_{{j1}}, V_{{1}}(t), if( T_{{j1}} <= t < T_{{a}} - T_{{j1}}, V_{{2}}(t), if( T_{{a}} - T_{{j1}} <= t < T_{{a}}, V_{{3}}(t), if( T_{{a}} <= t < T_{{a}} + T_{{v}}, V_{{4}}(t), if( T_{{a}} + T_{{v}} <= t < T_{{a}} + T_{{v}} + T_{{j2}}, V_{{5}}(t), if( T_{{a}} + T_{{v}} + T_{{j2}} <= t < T_{{a}} + T_{{v}} + T_{{d}} - T_{{j2}}, V_{{6}}(t), if( T_{{a}} + T_{{v}} + T_{{d}} - T_{{j2}} <= t < T_{{a}} + T_{{v}} + T_{{d}}, V_{{7}}(t), v_{{1}} ) ) ) ) ) ) ) )");
        hwa::debug!("Points:\nP_{{j1a}}=(T_{{j1}}, 0)\nP_{{j1d}}=(T_{{a}} - T_{{j1}}, 0)\nP_{{a}}=(T_{{a}}, 0)\nP_{{v}}=(T_{{a}} + T_{{v}}, 0)\nP_{{v}}=(T_{{a}} + T_{{v}}, 0)\nP_{{j2a}}=(T_{{a}} + T_{{v}} + T_{{j2}}, 0)\nP_{{j2d}}=(T_{{a}} + T_{{v}} + T_{{d}} - T_{{j2}}, 0)\nP_{{j2a}}=(T_{{a}} + T_{{v}} + T_{{d}}, 0)");
        hwa::debug!("s_i7 = {:?}", self.s_i7(&self.i7_end()));
        hwa::debug!("--");
    }

    /// The time at the start of 1st interval:
    /// \[ 0, T_{j1}\]
    #[inline]
    pub fn i1_start(&self) -> Real {
        Real::zero()
    }

    /// The time at the end of 1st interval:
    /// \[ 0, T_{j1}\]
    #[inline]
    pub fn i1_end(&self) -> Real {
        self.i2_start()
    }

    /// The time at the start of 2nd interval:
    /// \[ T_{j1}, T_{a} - T_{j1}\]
    #[inline]
    pub fn i2_start(&self) -> Real {
        self.t_j1
    }

    /// The time at the end of 2nd interval:
    /// \[ T_{j1}, T_{a} - T_{j1}\]
    #[inline]
    pub fn i2_end(&self) -> Real {
        self.i3_start()
    }

    /// The time at the start of 3rd interval:
    /// \[ T_{a} - T_{j1}, T_{a}\]
    #[inline]
    pub fn i3_start(&self) -> Real {
        self.t_a - self.t_j1
    }

    /// The time at the end of 3rd interval:
    /// \[ T_{a} - T_{j1}, T_{a}\]
    #[inline]
    pub fn i3_end(&self) -> Real {
        self.i4_start()
    }

    /// The time at the start of 4th interval:
    /// \[T_a, T_a + T_v\]
    #[inline]
    pub fn i4_start(&self) -> Real {
        self.t_a
    }

    /// The time at the end of 4th interval:
    /// \[T_a, T_a + T_v\]
    #[inline]
    pub fn i4_end(&self) -> Real {
        self.i5_start()
    }

    /// The time at the start of 5th interval:
    /// \[T_a + T_v, T - T_d + T_j2\]
    ///
    /// \[T_a + T_v, T_a + T_v + T_j2\]
    #[inline]
    pub fn i5_start(&self) -> Real {
        self.t_a + self.t_v
    }

    /// The time at the end of 5th interval:
    /// \[T_a + T_v, T - T_d + T_j2\]
    ///
    /// \[T_a + T_v, T_a + T_v + T_j2\]
    #[inline]
    pub fn i5_end(&self) -> Real {
        self.i6_start()
    }

    /// The time at the start of 6th interval:
    /// \[T - T_{d} + T_{j2}, T - T_{j2}\]
    ///
    /// \[T_{a} + T_{v} + T_{j2}, T_{a} + T_{v} + T_{d} - T_{j2}\]
    #[inline]
    pub fn i6_start(&self) -> Real {
        self.t_a + self.t_v + self.t_j2
    }

    /// The time at the end of 7th interval:
    /// \[T - T_d + T_j2, T - T_j2\]
    ///
    /// \[T_a + T_v + T_j2, T_a + T_v + T_d - T_j2\]
    #[inline]
    pub fn i6_end(&self) -> Real {
        self.i7_start()
    }

    /// The time at the start of 7th interval:
    /// \[T - T_j2, T\]
    ///
    /// \[T_a + T_v + T_d - T_j2, T_a + T_v + T_d\]
    #[inline]
    pub fn i7_start(&self) -> Real {
        self.t_a + self.t_v + self.t_d - self.t_j2
    }

    /// The time at the end of 7th interval:
    /// \[T - T_j2, T\]
    ///
    /// \[T_a + T_v + T_d - T_j2, T_a + T_v + T_d\]
    #[inline]
    pub fn i7_end(&self) -> Real {
        self.t_a + self.t_v + self.t_d
    }

    /// Compute intermediate piecewise function points to speedup equations
    /// A max between previous segment max pos and next one is applied to guarantee consistency
    /// when the move is (pseudo)triangular, given that position can never decrease
    fn compute_cache(&mut self) -> Result<(), CodeExecutionFailure> {
        if self.v_lim.is_negligible() {
            return Err(CodeExecutionFailure::NumericalError);
        }
        self.cache.s1_pt = self.s_i1(&self.i1_end());
        self.cache.s2_pt = self.cache.s1_pt.max(self.s_i2(&self.i2_end()));
        self.cache.s3_pt = self.cache.s2_pt.max(self.s_i3(&self.i3_end()));
        self.cache.s4_pt = self.cache.s3_pt.max(self.s_i4(&self.i4_end()));
        self.cache.s5_pt = self.cache.s4_pt.max(self.s_i5(&self.i5_end()));
        self.cache.s6_pt = self.cache.s5_pt.max(self.s_i6(&self.i6_end()));
        self.cache.s7_pt = self.cache.s6_pt.max(self.s_i7(&self.i7_end()));
        Ok(())
    }

    pub fn extend(&mut self, delta_e: Real) -> Result<(), ()> {
        if self.v_lim > Real::epsilon() {
            let extra_time = delta_e / self.v_lim;
            if self.t_v + extra_time > math::ZERO {
                self.t_v += extra_time;
                self.cache.s4_pt += delta_e;
                self.cache.s5_pt += delta_e;
                self.cache.s6_pt += delta_e;
                self.cache.s7_pt += delta_e;
            }
        }

        Ok(())
    }

    /// Computes the position (steps) in given timestamp (uS)
    /// * p_{1}(t) = if( 0 <= t < T_{j1} , s_{i1}(t) ) | \[0, T_{j1} \]
    /// * p_{2}(t) = if( T_{j1} <= t < T_{a} - T_{j1}, s_{i2}(t) ) | \[ T_{j1}, T_{a} - T_{j1} \]
    /// * p_{3}(t) = if( T_{a} - T_{j1} <= t < T_{a}, s_{i3}(t) ) | \[ T_{a} - T_{j1}, T_{a} \]
    /// * p_{4}(t) = if( T_{a} <= t < T_{a} + T_{v}, s_{i4}(t) ) | \[ T_{a}, T_{a} + T_{v} \]
    /// * p_{5}(t) = if( T_{a} + T_{v} <= t < T - T_{d} + T_{j2}, s_{i5}(t) ) | \[ T_{a} + T_{v}, T - T_{d} + T_{j2} \]
    /// * p_{6}(t) = if( T_{a} + T_{v} + T_{j2} <= t < T_{a} + T_{v} + T_{d} - T_{j2}, s_{i6}(t) ) | \[ T_{a} + T_{v} + T_{j2}, T_{a} + T_{v} + T_{d} - T_{j2} \]
    /// * p_{7}(t) = if( T_{a} + T_{v} <= t < T - T_{d} + T_{j2}, s_{i5}(t) ) | \[ T_{a} + T_{v}, T - T_{d} + T_{j2} \]
    pub fn eval_position(&self, t: Real) -> (u8, Option<Real>) {
        if t < math::ZERO {
            (0, Some(math::ZERO))
        } else if t >= self.i1_start() && t < self.i1_end() {
            (1, Some(self.s_i1(&t)))
        } else if t >= self.i2_start() && t < self.i2_end() {
            (2, Some(self.s_i2(&t)))
        } else if t >= self.i3_start() && t < self.i3_end() {
            (3, Some(self.s_i3(&t)))
        } else if t >= self.i4_start() && t < self.i4_end() {
            (4, Some(self.s_i4(&t)))
        } else if t >= self.i5_start() && t < self.i5_end() {
            (5, Some(self.s_i5(&t)))
        } else if t >= self.i6_start() && t < self.i6_end() {
            (6, Some(self.s_i6(&t)))
        } else if t >= self.i7_start() && t <= self.i7_end() {
            (7, Some(self.s_i7(&t)))
        } else {
            (8, Some(self.s_i8(&t)))
        }
    }

    /// Acceleration phase, jerk limited acceleration
    ///
    /// v_{i1}(t) = \frac{j_{max}t^2}{2}+v_{0} \\
    ///
    /// s_{i1}(t) = \int{v_{i1}(t)dt} \\
    ///
    /// s_{i1}(t)_{|t>\delta} = \frac{j_{max} (t-\delta)^3}{6} + v_{0} (t-\delta) \\
    pub fn s_i1(&self, t: &Real) -> Real {
        let dt = (*t) - self.i1_start();
        (self.j_max * SIXTH * dt * dt * dt) + (self.v_0 * dt)
    }

    /// Acceleration phase, constant acceleration
    ///
    /// v_{i2}(t)_{|t>\delta} = j_{max} T_{j1} t + v_{i1}(\delta)\\
    ///
    /// s_{i2}(t) = \int{v_{i2}(t)dt}\\
    ///
    /// s_{i2}(t)_{|t>\delta} = \frac{j_{max} T_{j1} (t - \delta)^2}{2} + v_{i1}(\delta)(t-\delta) + s_{i1}(\delta)\\
    ///
    pub fn s_i2(&self, t: &Real) -> Real {
        let dt = (*t) - self.i2_start();
        (self.j_max * HALF * self.t_j1 * dt) * (dt + self.t_j1) + self.v_0 * dt + self.cache.s1_pt
    }

    /// Acceleration phase, jerk limited deceleration
    ///
    /// v_{i3}(t)(t)_{|t>\delta} = v_{i2}(t) + v_{0} - j_{max} T_{j1} (t-\delta)\\
    ///
    /// s_{i3}(t) = \int{v_{i2}(t) + v_0 dt} - \int{v_{i3}(t)dt}\\
    pub fn s_i3(&self, t: &Real) -> Real {
        let dt = (*t) - self.i3_start();
        self.s_i2(t) - ((self.j_max * SIXTH * dt) * (dt * dt))
    }

    /// Constant velocity
    ///
    /// v_{i4}(t)(t)_{|t>\delta} = v_{lim} \\
    ///
    /// s_{i4}(t) = s_{i3}(\delta) + \int{v_{i3}(t)dt}\\
    pub fn s_i4(&self, t: &Real) -> Real {
        let dt = (*t) - self.i4_start();
        self.cache.s3_pt + (self.v_lim * dt)
    }

    /// Deceleration phase, jerk limited deceleration
    ///
    /// Same as s_{í1}(t)
    ///
    /// v_{i5}(t) = \frac{j_{max}t^2}{2}+v_{0} \\
    ///
    /// s_{i5}(t) = s_i4(t) - \int{v_{i5}(t)dt} \\
    ///
    /// s_{i5}(t)_{|t>\delta} = s_{i4}(t) - \frac{j_{max} (t-\delta)^3}{6} + v_{0} (t-\delta) \\
    pub fn s_i5(&self, t: &Real) -> Real {
        let dt = (*t) - self.i5_start();
        let r = (self.j_max * SIXTH * dt * dt * dt);
        self.s_i4(t) - r
    }

    /// Deceleration phase, constant deceleration
    ///
    /// Same as s_{í2}(t)
    ///
    /// v_{i6}(t)_{|t>\delta} = j_{max} T_{j1} t + v_{i5}(\delta)\\
    ///
    /// s_{i2}(t) = \int{v_{i2}(t)dt} + s_{i5}(\delta)\\
    ///
    /// s_{i5}(t)_{|t>\delta} = -\frac{j_{max} T_{j1} (t - \delta)^2}{2} + v_{i5}(\delta)(t-\delta) + s_{i5}(\delta)\\
    ///
    pub fn s_i6(&self, t: &Real) -> Real {
        let dt = (*t) - self.i6_start();
        let mhj2 = -self.j_max * HALF * self.t_j2;
        (dt * (mhj2 * dt)) + (((mhj2 * self.t_j2) + self.v_lim) * dt) + self.cache.s5_pt
    }

    /// Deceleration phase, jerk limited acceleration
    ///
    /// v_{i3}(t)(t)_{|t>\delta} = v_{i2}(t) + v_{0} - j_{max} T_{j1} (t-\delta)\\
    ///
    /// s_{i3}(t) = \int{v_{i2}(t) + v_0 dt} - \int{v_{i3}(t)dt}\\
    pub fn s_i7(&self, t: &Real) -> Real {
        let dt = (*t) - self.i7_start();
        self.s_i6(t) + ((self.j_max * SIXTH * dt) * dt * dt)
    }

    /// Constant (exit) speed at the end
    pub fn s_i8(&self, t: &Real) -> Real {
        let dt = (*t) - self.i7_end();
        self.cache.s7_pt + (self.v_1 * dt)
    }
}

impl MotionProfile for SCurveMotionProfile {
    #[inline(always)]
    fn end_time(&self) -> Real {
        self.i7_end()
    }
    fn end_pos(&self) -> Real {
        self.q1
    }

    fn eval_position(&self, t: Real) -> Option<Real> {
        if t < math::ZERO {
            None
        } else if t >= self.i1_start() && t < self.i1_end() {
            Some(self.s_i1(&t))
        } else if t >= self.i2_start() && t < self.i2_end() {
            Some(self.s_i2(&t))
        } else if t >= self.i3_start() && t < self.i3_end() {
            Some(self.s_i3(&t))
        } else if t >= self.i4_start() && t < self.i4_end() {
            Some(self.s_i4(&t))
        } else if t >= self.i5_start() && t < self.i5_end() {
            Some(self.s_i5(&t))
        } else if t >= self.i6_start() && t < self.i6_end() {
            Some(self.s_i6(&t))
        } else if t >= self.i7_start() && t <= self.i7_end() {
            Some(self.s_i7(&t))
        } else {
            Some(self.q1)
        }
    }
}

#[derive(Copy, Clone, Default)]
pub struct Cache {
    /// Position at self.s_i1(&self.i1_end())
    pub s1_pt: Real,

    /// Position at self.s_i2(&self.i2_end())
    pub s2_pt: Real,

    /// Position at self.s_i3(&self.i3_end())
    pub s3_pt: Real,

    /// Position at self.s_i4(&self.i4_end())
    pub s4_pt: Real,

    /// Position at self.s_i5(&self.i5_end())
    pub s5_pt: Real,

    /// Position at self.s_i6(&self.i6_end())
    pub s6_pt: Real,

    /// Position at self.s_i7(&self.i7_end())
    pub s7_pt: Real,
}

#[cfg(feature = "native")]
impl core::fmt::Display for SCurveMotionProfile {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\n\tt_j1={:?}, t_a={:?}, t_v={:?}, t_d={:?}, t_j2={:?}",
            self.t_j1.rdp(4),
            self.t_a.rdp(4),
            self.t_v.rdp(4),
            self.t_d.rdp(4),
            self.t_j1.rdp(4),
        )?;
        write!(
            f,
            "\n\ta_lim_a={:?}, v_lim={:?}, a_lim_d={:?}",
            self.a_lim_a, self.v_lim, self.a_lim_d,
        )
    }
}

#[cfg(test)]
pub mod test {
    //Example 3.9

    use crate::control::motion::{Constraints, SCurveMotionProfile};
    use crate::control::CodeExecutionFailure;
    use crate::hwa::math::Real;
    use num_traits::ToPrimitive;
    use crate::hwa;

    pub fn do_compute(
        q1: f32,
        v_0: f32,
        v_1: f32,
        v_max: f32,
        a_max: f32,
        j_max: f32,
    ) -> Result<SCurveMotionProfile, CodeExecutionFailure> {
        let constraints = Constraints {
            v_max: Real::from_f32(v_max),
            a_max: Real::from_f32(a_max),
            j_max: Real::from_f32(j_max),
        };
        SCurveMotionProfile::compute(
            Real::from_f32(q1),
            Real::from_f32(v_0),
            Real::from_f32(v_1),
            &constraints,
            false,
        )
    }

    fn approx_equal(what: &str, v: Real, expected: f32, tolerance: f32) {
        let v1 = v.to_f64().to_f32().unwrap();
        let ok = (v1 - expected).abs() < tolerance;
        assert!(ok, "{} = {} but should be = {}", what, v1, expected);
    }

    #[test]
    fn ex_3_9() {
        // With: q_1 = 10, v_0 = 1, v_1 = 0
        // Given: v_max = 5, a_max = 10, j_max = 30
        // Exp: T_a = 0.7333, T_v = 1.1433, T_d = 0.8333, T_j1 = 0.3333, T_j2 = 0.3333
        let r = do_compute(10., 1., 0., 5., 10., 30.).unwrap();

        approx_equal("T_a", r.t_a, 0.7333, 0.001);
        approx_equal("T_d", r.t_d, 0.8333, 0.001);
        approx_equal("T_j1", r.t_j1, 0.3333, 0.001);
        approx_equal("T_j2", r.t_j2, 0.3333, 0.001);
        approx_equal("T_v", r.t_v, 1.1433, 0.001);
    }

    #[test]
    fn ex_3_10() {
        // With: q_1 = 10, v_0 = 1, v_1 = 0
        // Given: v_max = 10, a_max = 10, j_max = 30
        // Exp: Ta = 1.0747, T_v = 0.0, T_d = 1.1747, T_j1 = 0.3333, T_j2 = 0.3333, vlim = 8.4136
        let r = do_compute(10., 1., 0., 10., 10., 30.).unwrap();

        approx_equal("T_a", r.t_a, 1.0747, 0.001);
        approx_equal("T_v", r.t_v, 0.0, 0.001);
        approx_equal("T_d", r.t_d, 1.1747, 0.001);
        approx_equal("T_j1", r.t_j1, 0.3333, 0.001);
        approx_equal("T_j2", r.t_j2, 0.3333, 0.001);
        approx_equal("v_lim", r.v_lim, 8.4136, 0.001);
    }

    #[test]
    fn ex_3_11() {
        // With: q_1 = 10, v_0 = 7, v_1 = 0
        // Given: v_max = 10, a_max = 10, j_max = 30
        // According to the paper is:
        // Exp: Ta = 0.4666, T_v = 0.0, T_d = 1.4718, T_j1 = 0.2321, T_j2 = 0.2321, vlim = 8.6329
        // But with the less costly descend:
        // Exp: Ta = 0.4526, T_v = 0.0, T_d = 1.5195, T_j1 = 0.2186, T_j2 = 0.2186, vlim = 8.5347
        let r = do_compute(10., 7., 0., 10., 10., 30.).unwrap();

        approx_equal("T_a", r.t_a, 0.4526, 0.001);
        approx_equal("T_v", r.t_v, 0.0, 0.001);
        approx_equal("T_d", r.t_d, 1.5195, 0.001);
        approx_equal("T_j1", r.t_j1, 0.2186, 0.001);
        approx_equal("T_j2", r.t_j2, 0.2186, 0.001);
        approx_equal("vlim", r.v_lim, 8.5347, 0.001);
    }

    #[test]
    fn ex_3_12() {
        // With: q_1 = 10, v_0 = 7.5, v_1 = 0
        // Given: v_max = 10, a_max = 10, j_max = 30
        // Exp: Ta = 0.0, T_v = 0.0, T_d = 2.6667, T_j1 = 0.0, T_j2 = 0.0973, vlim = 7.5
        let r = do_compute(10., 7.5, 0., 10., 10., 30.).unwrap();

        approx_equal("T_a", r.t_a, 0.0, 0.01);
        approx_equal("T_v", r.t_v, 0.0, 0.001);
        approx_equal("T_d", r.t_d, 2.6667, 0.01);
        approx_equal("T_j1", r.t_j1, 0.0, 0.01);
        approx_equal("T_j2", r.t_j2, 0.0973, 0.01);
        approx_equal("v_lim", r.v_lim, 7.5, 0.01);
    }

    #[test]
    fn ex_3_13() {
        // With: q_1 = 10, v_0 = 0, v_1 = 0
        // Given: v_max = 10, a_max = 20, j_max = 30
        // Exp: Ta = 1.1006, T_v = 0.0, T_d = 1.1006, T_j1 = 0.5503, T_j2 = 0.5503, vlim = 9.0826
        let r = do_compute(10., 0.0, 0., 10., 20., 30.).unwrap();

        r.params_dump();
        hwa::info!("profile: {}", r);
        hwa::info!("constraints: {:?}", r.constraints);

        approx_equal("T_a", r.t_a, 1.1006, 0.01);
        approx_equal("T_v", r.t_v, 0.0, 0.001);
        approx_equal("T_d", r.t_d, 1.1006, 0.01);
        approx_equal("T_j1", r.t_j1, 0.5333, 0.01);
        approx_equal("T_j2", r.t_j2, 0.5333, 0.01);
        approx_equal("v_lim", r.v_lim, 9.0826, 0.01);
    }
}
