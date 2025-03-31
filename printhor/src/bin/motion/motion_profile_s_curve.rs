//! The Motion profile trait and current implementation: Trajectory with Double S Velocity Profile \[1\]
//!
//! \[1\] Biagiotti, L., Melchiorri, C.: Trajectory Planning for Automatic Machines and Robots. Springer, Heidelberg (2008). [DOI:10.1007/978-3-540-85629-0](https://doi.org/10.1007/978-3-540-85629-0)

use crate::hwa;
use crate::motion;
use crate::processing;
use hwa::math;

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
    ///
    /// # Units
    ///
    /// - `v_max`: [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s
    /// - `a_max`: [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²
    /// - `j_max`: [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s³
    pub constraints: Constraints,

    /// The time parameters of the motion profile.
    pub times: Times,

    /// Initial velocity for the motion profile.
    ///
    /// # Units
    ///
    /// - Initial velocity: [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s
    pub initial_velocity: math::Real,

    /// Final velocity for the motion profile.
    ///
    /// # Units
    ///
    /// - Final velocity: [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s
    pub final_velocity: math::Real,

    /// An optional error correction factor.
    ///
    /// # Units
    ///
    /// - Error correction factor: Implementation-specific units
    pub error_correction: Option<math::Real>,
}

/// The `Constraints` struct represents the limits of the motion profile in terms of velocity, acceleration, and jerk.
///
/// # Fields
///
/// * `v_max` - The maximum velocity (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s). This defines the upper limit of speed that can be achieved.
/// * `a_max` - The maximum acceleration (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²). This represents the highest rate of change of velocity that is permissible.
/// * `j_max` - The maximum jerk (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s³). This is the maximum rate of change of acceleration.
#[derive(Clone, Copy, PartialEq)]
pub struct Constraints {
    /// The maximum velocity (v_max) (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
    /// This defines the upper limit of speed that can be achieved.
    pub v_max: math::Real,

    /// The maximum acceleration (a_max) (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²).
    /// This represents the highest rate of change of velocity that is permissible.
    pub a_max: math::Real,

    /// The maximum jerk (j_max) (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s³).
    /// This is the maximum rate of change of acceleration.
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

/// Struct representing the time parameters for different phases of the S-curve motion profile.
///
/// This struct is used to define the specific time segments allocated to different phases of the S-curve.
///
/// # Fields
///
/// * `t_2s` - Time at which interval 2 starts.
/// * `t_a` - Time spent in the acceleration phase (in seconds).
/// * `t_v` - Time spent at constant velocity (in seconds).
/// * `t_d` - Time spent in the deceleration phase (in seconds).
/// * `t_j2` - Time spent in the second jerk phase (in seconds).
#[derive(PartialEq)]
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
        t_8s: math::Real) -> Self {
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
    
    #[inline]
    // Time spent in the first jerk phase (in seconds).
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
}

impl core::fmt::Debug for Times {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "t"
        )
    }
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
/// * `times` - The profile segment start times.
/// * `v_0` - Initial velocity (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
/// * `v_1` - Final velocity (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
/// * `a_0` - Initial acceleration (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²).
/// * `a_1` - Final velocity (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²).
/// * `j_max` - Maximum jerk (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s³).
/// * `a_lim_a` - Acceleration limit during the acceleration phase (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²).
/// * `a_lim_d` - Deceleration limit during the deceleration phase (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s²).
/// * `v_lim` - Velocity limit (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
/// * `q1` - Displacement or position to be achieved (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
/// * `constraints` - Constraints applied to the motion profile.
/// * `cache` - Cached values for optimization and repeated calculations.
#[derive(PartialEq)]
pub struct SCurveMotionProfile {
    pub times: Times,

    /// Initial velocity.
    pub v_i: math::Real,

    /// Final velocity.
    pub v_o: math::Real,

    /// Initial acceleration.
    pub a_i: math::Real,

    /// Final acceleration.
    pub a_o: math::Real,

    /// Maximum jerk.
    pub j_max: math::Real,

    /// Acceleration limit during the acceleration phase.
    pub a_lim_a: math::Real,

    /// Deceleration limit during the deceleration phase.
    pub a_lim_d: math::Real,

    /// Velocity limit.
    pub v_lim: math::Real,

    /// Displacement or position to be achieved.
    pub s_o: math::Real,

    /// Constraints applied to the motion profile.
    pub constraints: Constraints,

    /// Cached values for optimization and repeated calculations.
    pub cache: Cache,
}

impl SCurveMotionProfile {
    /// Compute the S-curve motion profile.
    ///
    /// # Arguments
    ///
    /// * `s_o` - Displacement or position to be achieved (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
    /// * `v_i` - Initial velocity (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
    /// * `v_o` - Output (final) velocity (in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/s).
    /// * `constraints` - [Constraints] applied to the motion profile.
    /// * `error_correction` - Flag to enable error correction.
    ///
    /// # Returns
    ///
    /// `Result<SCurveMotionProfile,  processing::CodeExecutionFailure>` - The computed S-curve motion profile or an error.
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
    /// \[ $$ v_1 = \frac{1}{0} $$ \]
    /// ```
    /// let constraints = Constraints {
    ///     v_max: math::Real::from_f32(10.0),
    ///     a_max: math::Real::from_f32(2.0),
    ///     j_max: math::Real::from_f32(1.0),
    /// };
    /// let profile = SCurveMotionProfile::compute(Real::from_f32(20.0), math::Real::from_f32(0.0), math::Real::from_f32(5.0), &constraints, false)?;
    /// println!("{}", profile);
    /// ```
    ///
    pub fn compute(
        s_o: math::Real,
        v_i: math::Real,
        v_o: math::Real,
        #[cfg(feature = "with-motion-chaining")]
        a_i: math::Real,
        #[cfg(feature = "with-motion-chaining")]
        a_o: math::Real,
        constraints: &Constraints,
        error_correction: bool,
    ) -> Result<SCurveMotionProfile, processing::CodeExecutionFailure> {
        if constraints.v_max.is_negligible()
            || constraints.a_max.is_negligible()
            || constraints.j_max.is_negligible()
        {
            hwa::warn!("[SCurveMotionProfile] Unable to perform movement: constraints are unset");
            return Err(processing::CodeExecutionFailure::NumericalError);
        }
        // Set v_max to arg_max { v_0, v_1, v_max}
        #[allow(unused, unused_mut)]
        let mut v_max = v_i.max(v_o).max(constraints.v_max);

        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[SCurveMotionProfile] v_{{0}} = {:?}", v_i);
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[SCurveMotionProfile] v_{{1}} = {:?}", v_o);
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[SCurveMotionProfile] q_{{1}} = {:?}", s_o);
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[SCurveMotionProfile] v_{{max}} = {:?}", constraints.v_max);
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[SCurveMotionProfile] a_{{max}} = {:?}", constraints.a_max);
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[SCurveMotionProfile] j_{{max}} = {:?}", constraints.j_max);

        // First, compute the displacement

        // [[1]] First necessary to verify whether a trajectory can be actually performed or not.
        // As a matter of fact, there are several cases in which a trajectory cannot be computed
        // with the given constraints. For example, if the desired displacement h is small with
        // respect to the difference between the initial and final velocities v0 and v1, it might be not possible
        // to change the velocity (with the given limits on jerk and acceleration), while accomplishing the displacement h := q_1 - q_0
        // In printhor, always h = q_1

        let j_max_inv = constraints.j_max.recip();
        let t_jmax = constraints.a_max * j_max_inv;

        #[cfg(feature = "debug-motion-planning")]
        hwa::info!("[SCurveMotionProfile] T_{{jmax}} = {:?}", t_jmax);

        // [eq 3.17] T_{j}^{*} = min\Bigg\{\sqrt{\frac{|v_{1} - v_{0}|}{j_{max}}}, \frac{a_{max}}{j_{max}}\Bigg\}
        let t_jstar =
            math::Real::vmin((((v_o - v_i).abs()) * j_max_inv).sqrt(), Some(t_jmax)).unwrap_or(math::ZERO);

        // [[1]] If t_jstar = amax/jmax, the acceleration reaches its maximum value and a segment with zero jerk may exist.
        let q_lim = if (t_jstar - t_jmax).is_negligible() {
            let lim = (math::HALF * (v_i + v_o)) * (t_jstar + ((v_o - v_i).abs() * j_max_inv));
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!(
                "[SCurveMotionProfile] T_{{j}}^{{*}} = \\sqrt{{\\frac{{|v_{{1}} - v_{{0}}|}}{{j_{{max}}}}}} = {:?}",
                lim
            );
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!("[SCurveMotionProfile] The acceleration does not reach its maximum value");
            lim
        } else {
            let lim = t_jstar * (v_i + v_o);
            // the acceleration reaches its maximum value and a segment with zero jerk may exist
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!(
                "[SCurveMotionProfile] T_{{j}}^{{*}} = \\frac{{a_{{max}}}}{{j_{{max}}}} = {:?}",
                lim
            );
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!("[SCurveMotionProfile] The acceleration reaches its maximum value");
            lim
        };

        let not_feasible = s_o <= q_lim;

        cfg_if::cfg_if! {
            if #[cfg(not(feature = "with-motion-chaining"))] {
                #[allow(non_upper_case_globals)]
                const a_i: math::Real = math::ZERO;
                #[allow(non_upper_case_globals)]
                const a_o: math::Real = math::ZERO;
            }
        }

        if not_feasible || !a_i.is_zero() || !a_o.is_zero() {
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!("[SCurveMotionProfile] Movement NOT feasible. Performing cubic blends");
            
            let t_1 = math::solve_cubic(
                constraints.j_max * math::SIXTH,
                a_i * math::HALF,
                v_i, -s_o).unwrap();
            
            let t_j = t_1;
            let mut profile = if v_i < v_o {
                SCurveMotionProfile {
                    times: Times::new(
                        t_j,
                        t_j,
                        t_j,
                        t_j,
                        t_j,
                        t_j,
                        t_j,
                    ),
                    // { t1: t_j, t2: math::ZERO, t3: math::ZERO, t4: math::ZERO, t5: math::ZERO, t6: math::ZERO, t7: math::ZERO, t: t_j},
                    v_i: v_i,
                    v_o: v_o,
                    a_i: a_i,
                    a_o: a_o,
                    j_max: constraints.j_max,
                    a_lim_a: math::ZERO,
                    a_lim_d: math::ZERO,
                    v_lim: v_o.max(v_i),
                    s_o: s_o,
                    cache: Cache::default(),
                    constraints: *constraints,
                }
            }
            else {
                SCurveMotionProfile {
                    times: Times::new(
                        math::ZERO,
                        math::ZERO,
                        math::ZERO,
                        math::ZERO,
                        math::ZERO,
                        math::ZERO,
                        t_j,
                    ),
                    v_i: v_i,
                    v_o: v_o,
                    a_i: a_i,
                    a_o: a_o,
                    j_max: constraints.j_max,
                    a_lim_a: math::ZERO,
                    a_lim_d: math::ZERO,
                    v_lim: v_o.max(v_i),
                    s_o: s_o,
                    cache: Cache::default(),
                    constraints: *constraints,
                }
            };
            profile.compute_cache()?;
            if error_correction {
                let final_pos = profile.s_7(&profile.i7_end());
                let delta_e = profile.s_o - final_pos;
                profile
                    .extend(delta_e)
                    .map_err(|_| processing::CodeExecutionFailure::ERR)?;
            }
            Ok(profile)
        } else {
            #[cfg(feature = "debug-motion-planning")]
            hwa::info!(
                "[SCurveMotionProfile] q_{{1}} > q_{{lim}} [ {:?} > {:?} ] ",
                s_o,
                q_lim
            );

            let mut a_max = constraints.a_max;
            let mut a_max_inv = a_max.recip();
            let mut a_max_squared = a_max * a_max;
            // aj_ratio = a_max / j_max
            let mut aj_ratio = a_max * j_max_inv;

            #[allow(unused, unused_mut)]
            let mut prev_vmax = v_max;
            let gamma = math::Real::from_f32(0.9);
            loop {
                let times = Self::compute_case_1(
                    s_o,
                    v_i,
                    v_o,
                    v_max,
                    a_max,
                    constraints.j_max,
                    aj_ratio,
                    a_max_squared,
                )?;
                // FIXME
                if times.t_v() <= math::ZERO {
                    // TODO: lagrange multipliers or something better to find a solution with less iterations

                    let t_j = aj_ratio;

                    let sqrt_delta = ((a_max * a_max * a_max * a_max * j_max_inv * j_max_inv)
                        + (math::TWO * ((v_i * v_i) + (v_o * v_o))
                            + (a_max * ((math::FOUR * s_o) - (math::TWO * (t_j) * (v_i + v_o))))))
                        .sqrt()
                        .unwrap();
                    let aj = (a_max * a_max) * j_max_inv;
                    let mut t_a_2 = (aj - (v_i + v_i) + sqrt_delta) * math::HALF * a_max_inv;
                    let mut t_d_2 = (aj - (v_o + v_o) + sqrt_delta) * math::HALF * a_max_inv;
                    let mut t_j1 = t_j;
                    let mut t_j2 = t_j;

                    if t_a_2 < math::ZERO || t_d_2 < math::ZERO {
                        // it may happen that Ta or Td becomes negative. In this case, only one of the acceleration or deceleration phase is necessary
                        let qd = s_o / (v_i + v_o);
                        #[allow(unused)]
                        let vsum_sq = (v_i + v_o) * (v_i + v_o);
                        if t_a_2 < math::ZERO {
                            t_a_2 = math::ZERO;
                            t_j1 = math::ZERO;

                            t_d_2 = qd * math::TWO;
                            t_j2 = ((constraints.j_max * s_o)
                                - (constraints.j_max
                                    * (constraints.j_max * s_o * s_o
                                        + (v_o + v_i) * (v_o + v_i) * (v_o - v_i)))
                                    .sqrt()
                                    .ok_or(processing::CodeExecutionFailure::NumericalError)?)
                                / (constraints.j_max * (v_o + v_i));
                        } else if t_d_2 < math::ZERO {
                            t_d_2 = math::ZERO;
                            t_j2 = math::ZERO;
                            t_a_2 = qd * math::TWO;
                            t_j1 = ((constraints.j_max * s_o)
                                - (constraints.j_max
                                    * (constraints.j_max * s_o * s_o
                                        + (v_o + v_i) * (v_o + v_i) * (v_i - v_o)))
                                    .sqrt()
                                    .ok_or(processing::CodeExecutionFailure::NumericalError)?)
                                / (constraints.j_max * (v_o + v_i));
                        }
                    } else {
                        t_j1 = t_j;
                        t_j2 = t_j;
                    }

                    if t_a_2 >= (t_j1 + t_j1) && t_d_2 >= (t_j2 + t_j2) {
                        let a_lim_a = constraints.j_max * t_j1;
                        let a_lim_d = -constraints.j_max * t_j2;

                        let v_lim = v_i + (t_a_2 - t_j1) * a_lim_a;
                        #[allow(unused)]
                        let t = t_a_2 + math::ZERO + t_d_2;
                        let mut profile = SCurveMotionProfile {
                            times: Times::new(
                                t_j1,
                                t_a_2 - t_j1,
                                t_a_2,
                                t_a_2,
                                t_a_2 + t_j2,
                                t_a_2 + t_d_2 - t_j2,
                                t_a_2 + t_d_2,
                            ),
                            v_i,
                            v_o,
                            a_i,
                            a_o,
                            j_max: constraints.j_max,
                            a_lim_a,
                            a_lim_d,
                            v_lim,
                            s_o,
                            cache: Cache::default(),
                            constraints: *constraints,
                        };
                        profile.compute_cache()?;
                        if error_correction {
                            let final_pos = profile.s_7(&profile.i7_end()) + math::Real::epsilon();
                            let delta_e = profile.s_o - final_pos;
                            profile
                                .extend(delta_e)
                                .map_err(|_| processing::CodeExecutionFailure::ERR)?;
                        }
                        return Ok(profile);
                    }
                    a_max *= gamma;
                    a_max_inv = a_max.recip();
                    a_max_squared = a_max * a_max;
                    aj_ratio = a_max * j_max_inv;
                    if a_max < math::Real::from_f32(0.1) {
                        return if (v_o - v_i).abs().is_negligible() {
                            let _t_v = math::TWO * (s_o / (v_i + v_o));
                            let mut profile = SCurveMotionProfile {
                                times: Times::new(
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                ),
                                v_i,
                                v_o,
                                a_i,
                                a_o,
                                j_max: constraints.j_max,
                                a_lim_a: math::ZERO,
                                a_lim_d: math::ZERO,
                                v_lim: v_o,
                                s_o: s_o,
                                cache: Cache::default(),
                                constraints: *constraints,
                            };
                            profile.compute_cache()?;
                            if error_correction {
                                let final_pos = profile.s_7(&profile.i7_end());
                                let delta_e = profile.s_o - final_pos;
                                profile
                                    .extend(delta_e)
                                    .map_err(|_| processing::CodeExecutionFailure::ERR)?;
                            }
                            Ok(profile)
                        } else {
                            // Compute the time need to accel
                            let t_a = math::TWO * (s_o / (v_i + v_o));

                            // Compute the accel
                            let a = (v_o - v_i) / t_a;
                            let t_j1 = math::HALF * t_a;

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

                            let j_max = math::TWO * a_1;

                            let mut profile = SCurveMotionProfile {
                                times: Times::new(
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                    math::ZERO,
                                ),
                                v_i: v_i,
                                v_o: v_o,
                                a_i: a_i,
                                a_o: a_1,
                                j_max,
                                a_lim_a: math::ZERO,
                                a_lim_d: math::ZERO,
                                v_lim: v_o.max(v_i),
                                s_o: s_o,
                                cache: Cache::default(),
                                constraints: *constraints,
                            };
                            profile.compute_cache()?;
                            if error_correction {
                                let final_pos = profile.s_7(&profile.i7_end());
                                let delta_e = profile.s_o - final_pos;
                                profile
                                    .extend(delta_e)
                                    .map_err(|_| processing::CodeExecutionFailure::ERR)?;
                            }
                            Ok(profile)
                        };
                    }
                } else {
                    let a_lim_a = constraints.j_max * times.t_j1();
                    let a_lim_d = constraints.j_max * times.t_j2();
                    let v_lim = v_i + (times.t_a() - times.t_j1()) * a_lim_a;
                    let mut profile = SCurveMotionProfile {
                        times,
                        v_i,
                        v_o,
                        a_i,
                        a_o,
                        j_max: constraints.j_max,
                        a_lim_a,
                        a_lim_d,
                        v_lim,
                        s_o,
                        cache: Cache::default(),
                        constraints: *constraints,
                    };
                    profile.compute_cache()?;
                    if error_correction {
                        let final_pos = profile.s_7(&profile.i7_end());
                        let delta_e = profile.s_o - final_pos;
                        profile
                            .extend(delta_e)
                            .map_err(|_| processing::CodeExecutionFailure::ERR)?;
                    }
                    return Ok(profile);
                }
            }
        }
    }

    /// Procedure:
    ///
    /// Assuming that `v_vmax` and `a_max` are reached (*case_1*) compute the time intervals:
    ///
    /// For T_j1, T_a:
    /// <p>
    /// \[
    /// \begin{align}
    ///   (v_{max} - v_{0}) j_{max} < a_{max}^2 & \implies a_{max} \text{ is not reached} \implies
    /// T_{j1} & = \sqrt{ \frac{v_{max} - v_{0}}{j_{max}} }\\
    /// & & T_{a} & = 2 T_{j1} \\
    /// \text{ otherwise} & \implies a_{max} \text{ is reached} \implies T_{j1} & = \frac{a_{max}}{j_{max}} \\
    /// & & T_{a} = T_{j1} + \frac{v_{max} - v_{0}}{a_{max}}
    /// \end{align}
    /// \]
    /// </p>
    ///
    /// For T_j2, T_d:
    ///
    /// $$
    ///   (v_{max} - v_{1}) j_{max} < a_{max}^2 \implies a_{min} \text{ is not reached} \implies T_{j2} = \sqrt{ \frac{v_{max} - v_{1}}{j_{max}} }, T_{d} = 2 T_{j2}
    /// $$
    ///
    /// $$
    ///   \text{ otherwise} \implies a_{min} \text{ is reached} \implies T_{j2} = \frac{a_{max}}{j_{max}}, T_{d} = T_{j2} + \frac{v_{max} - v_{1}}{a_{max}}
    /// $$
    fn compute_case_1(
        s_o: math::Real,
        v_i: math::Real,
        v_o: math::Real,
        v_max: math::Real,
        a_max: math::Real,
        j_max: math::Real,
        aj_ratio: math::Real,
        a_max_squared: math::Real,
    ) -> Result<Times, processing::CodeExecutionFailure> {
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[SCurveMotionProfile] [compute_case_1] with aj_ratio {:?}",
            aj_ratio
        );
        let a_max_not_reached = (v_max - v_i) * j_max < a_max_squared;
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[SCurveMotionProfile] [compute_case_1] a_max is {}",
            if a_max_not_reached {
                "not reached"
            } else {
                "reached"
            }
        );
        let a_min_not_reached = (v_max - v_o) * j_max < a_max_squared;
        #[cfg(feature = "debug-motion-planning")]
        hwa::info!(
            "[SCurveMotionProfile] [compute_case_1] a_min is {}",
            if a_min_not_reached {
                "not reached"
            } else {
                "reached"
            }
        );

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
            let t_j = ((v_max - v_i).max(math::ZERO) * j_max_inv)
                .sqrt()
                .ok_or(processing::CodeExecutionFailure::NumericalError)?;
            (t_j, t_j + t_j)
        } else {
            let t_j = aj_ratio;
            (t_j, t_j + (v_max - v_i).max(math::ZERO) * a_max_inv)
        };
        let (t_j2, t_d) = if a_min_not_reached {
            let t_j = ((v_max - v_o).max(math::ZERO) * j_max_inv)
                .sqrt()
                .ok_or(processing::CodeExecutionFailure::NumericalError)?;
            (t_j, t_j + t_j)
        } else {
            let t_j = aj_ratio;
            (t_j, t_j + (v_max - v_o).max(math::ZERO) * a_max_inv)
        };

        // Efficiently compute: (q_1 / v_max) - (t_a / 2) * (1 + (v_0 / v_max)) - (t_d / 2) * (1 + (v_1 / v_max))
        // ... with maximum possible numerical precision

        let v_max_inv = v_max.recip();
        let half_ta = t_a * math::HALF;
        let term1 = half_ta + half_ta * (v_i * v_max_inv);
        let half_td = t_d * math::HALF;
        let term2 = half_td + half_td * (v_o * v_max_inv);

        let t_v = (s_o * v_max_inv) - term1 - term2;
        Ok(Times::new(
            t_j1,
            t_a - t_j1,
            t_a,
            t_a + t_v,
            t_a + t_v + t_j2,
            t_a + t_v + t_d - t_j2,
            t_a + t_v + t_d,
        ))
    }

    /// The time at the start of 1st interval:
    /// \[ 0, T_{j1}\]
    #[inline]
    pub const fn i1_start(&self) -> math::Real {
        math::ZERO
    }

    /// The time at the end of 1st interval:
    /// \[ 0, T_{j1}\]
    #[inline]
    pub const fn i1_end(&self) -> math::Real {
        self.times.t_2s
    }

    /// The time at the start of 2nd interval:
    /// \[ T_{j1}, T_{a} - T_{j1}\]
    #[inline]
    pub const fn i2_start(&self) -> math::Real {
        self.times.t_2s
    }

    /// The time at the end of 2nd interval:
    /// \[ T_{j1}, T_{a} - T_{j1}\]
    #[inline]
    pub const fn i2_end(&self) -> math::Real {
        self.times.t_3s
    }

    /// The time at the start of 3rd interval:
    /// \[ T_{a} - T_{j1}, T_{a}\]
    #[inline]
    pub const fn i3_start(&self) -> math::Real {
        self.times.t_3s
    }

    /// The time at the end of 3rd interval:
    /// \[ T_{a} - T_{j1}, T_{a}\]
    #[inline]
    pub const fn i3_end(&self) -> math::Real {
        self.times.t_4s
    }

    /// The time at the start of 4th interval:
    /// \[T_a, T_a + T_v\]
    #[inline]
    pub const fn i4_start(&self) -> math::Real {
        self.times.t_4s
    }

    /// The time at the end of 4th interval:
    /// \[T_a, T_a + T_v\]
    #[inline]
    pub const fn i4_end(&self) -> math::Real {
        self.times.t_5s
    }

    /// The time at the start of 5th interval:
    /// \[T_a + T_v, T - T_d + T_j2\]
    ///
    /// \[T_a + T_v, T_a + T_v + T_j2\]
    #[inline]
    pub const fn i5_start(&self) -> math::Real {
        self.times.t_5s
    }

    /// The time at the end of 5th interval:
    /// \[T_a + T_v, T - T_d + T_j2\]
    ///
    /// \[T_a + T_v, T_a + T_v + T_j2\]
    #[inline]
    pub const fn i5_end(&self) -> math::Real {
        self.times.t_6s
    }

    /// The time at the start of 6th interval:
    /// \[T - T_{d} + T_{j2}, T - T_{j2}\]
    ///
    /// \[T_{a} + T_{v} + T_{j2}, T_{a} + T_{v} + T_{d} - T_{j2}\]
    #[inline]
    pub const fn i6_start(&self) -> math::Real {
        self.times.t_6s
    }

    /// The time at the end of 7th interval:
    /// \[T - T_d + T_j2, T - T_j2\]
    ///
    /// \[T_a + T_v + T_j2, T_a + T_v + T_d - T_j2\]
    #[inline]
    pub const fn i6_end(&self) -> math::Real {
        self.times.t_7s
    }

    /// The time at the start of 7th interval:
    /// \[T - T_j2, T\]
    ///
    /// \[T_a + T_v + T_d - T_j2, T_a + T_v + T_d\]
    #[inline]
    pub const fn i7_start(&self) -> math::Real {
        self.times.t_7s
    }

    /// The time at the end of 7th interval:
    /// \[T - T_j2, T\]
    ///
    /// \[T_a + T_v + T_d - T_j2, T_a + T_v + T_d\]
    #[inline]
    pub const fn i7_end(&self) -> math::Real {
        self.times.t_8s
    }

    /// Compute intermediate piecewise function points to speedup equations
    /// A max between previous segment max pos and next one is applied to guarantee consistency
    /// when the move is (pseudo)triangular, given that position can never decrease
    fn compute_cache(&mut self) -> Result<(), processing::CodeExecutionFailure> {
        if self.v_lim.is_negligible() {
            return Err(processing::CodeExecutionFailure::NumericalError);
        }
        let t_end = self.i1_end();
        self.cache.Da1 = self.a_1(&t_end);
        self.cache.Dv1 = self.v_1(&t_end);
        self.cache.Ds1 = self.s_1(&t_end);

        let t_end = self.i2_end();
        self.cache.Da2 = self.a_2(&t_end);
        self.cache.Dv2 = self.v_2(&t_end);
        self.cache.Ds2 = self.s_2(&t_end);

        let t_end = self.i3_end();
        self.cache.Da3 = self.a_3(&t_end);
        self.cache.Dv3 = self.v_3(&t_end);
        self.cache.Ds3 = self.s_3(&t_end);

        let t_end = self.i4_end();
        self.cache.Da4 = self.a_4(&t_end);
        self.cache.Dv4 = self.v_4(&t_end);
        self.cache.Ds4 = self.s_4(&t_end);

        let t_end = self.i5_end();
        self.cache.Da5 = self.a_5(&t_end);
        self.cache.Dv5 = self.v_5(&t_end);
        self.cache.Ds5 = self.s_5(&t_end);

        let t_end = self.i6_end();
        self.cache.Da6 = self.a_6(&t_end);
        self.cache.Dv6 = self.v_6(&t_end);
        self.cache.Ds6 = self.s_6(&t_end);

        let t_end = self.i7_end();
        self.cache.Da7 = self.a_7(&t_end);
        self.cache.Dv7 = self.v_7(&t_end);
        self.cache.Ds7 = self.s_7(&t_end);
        Ok(())
    }

    pub fn extend(&mut self, delta_e: math::Real) -> Result<(), ()> {
        if self.v_lim > math::Real::epsilon() {
            let extra_time = delta_e / self.v_lim;
            if self.times.t_v() + extra_time > math::ZERO {
                self.times.t_5s += extra_time;
                self.times.t_6s += extra_time;
                self.times.t_7s += extra_time;
                self.times.t_8s += extra_time;
                self.cache.Ds4 += delta_e;
                self.cache.Ds5 += delta_e;
                self.cache.Ds6 += delta_e;
                self.cache.Ds7 += delta_e;
            }
        }
        Ok(())
    }

    /// Acceleration phase, jerk limited acceleration
    pub fn a_1(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i1_start();
        (self.j_max * dt) + self.a_i
    }

    /// Acceleration phase, jerk limited acceleration
    pub fn v_1(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i1_start();
        let dt_sq = dt * dt;
        (self.j_max * math::HALF * dt_sq) + (self.a_i * dt) + self.v_i
    }

    /// Acceleration phase, jerk limited acceleration
    pub fn s_1(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i1_start();
        let dt_sq = dt * dt;
        (self.j_max * math::SIXTH * dt_sq * dt) + (self.a_i * math::HALF * dt_sq) +  (self.v_i * dt)
    }

    /// Acceleration phase, constant acceleration
    pub fn a_2(&self, _t: &math::Real) -> math::Real {
        self.cache.Da1
    }

    /// Acceleration phase, constant acceleration
    pub fn v_2(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i2_start();
        (self.cache.Da1 *  dt) + self.cache.Dv1
    }

    /// Acceleration phase, constant acceleration
    pub fn s_2(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i2_start();
        (self.j_max * math::HALF * self.times.t_j1() * dt) * (dt + self.times.t_j1()) + self.v_i * dt + self.cache.Ds1
    }

    /// Acceleration phase, jerk limited deceleration
    pub fn a_3(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i3_start();
        (-self.j_max * dt) +  self.cache.Da2
    }

    /// Acceleration phase, jerk limited deceleration
    pub fn v_3(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i3_start();
        let dt_sq = dt * dt;
        (-self.j_max * math::HALF * dt_sq) + (self.cache.Da2 * dt) + self.cache.Dv2
    }

    /// Acceleration phase, jerk limited deceleration
    pub fn s_3(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i3_start();
        let dt_sq = dt * dt;
        (-self.j_max * math::SIXTH * dt_sq * dt) + (self.cache.Da2 * math::HALF * dt_sq) + self.cache.Dv2 * dt + self.cache.Ds2
    }

    /// Constant velocity phase
    pub fn a_4(&self, _t: &math::Real) -> math::Real {
        self.cache.Da3
    }

    /// Constant velocity phase
    pub fn v_4(&self, _t: &math::Real) -> math::Real {
        self.cache.Dv3
    }

    /// Constant velocity phase
    pub fn s_4(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i4_start();
        (self.cache.Dv3 * dt) + self.cache.Ds3
    }

    /// Deceleration phase, jerk limited deceleration
    pub fn a_5(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i5_start();
        (-self.j_max * dt) + self.cache.Da4
    }

    /// Deceleration phase, jerk limited deceleration
    pub fn v_5(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i5_start();
        let dt_sq = dt * dt;
        (-self.j_max * math::HALF * dt_sq) + (self.cache.Da4 * dt) + self.cache.Dv4
    }

    /// Deceleration phase, jerk limited deceleration
    pub fn s_5(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i5_start();
        let dt_sq = dt * dt;
        (-self.j_max * math::SIXTH * dt_sq * dt) + (self.cache.Da4 * math::HALF * dt_sq) + self.cache.Dv4 * dt + self.cache.Ds4
    }

    /// Deceleration phase, constant deceleration
    pub fn a_6(&self, _t: &math::Real) -> math::Real {
        self.cache.Da5
    }

    /// Deceleration phase, constant deceleration
    pub fn v_6(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i6_start();
        (self.cache.Da5 * dt) + self.cache.Dv5
    }

    /// Deceleration phase, constant deceleration
    pub fn s_6(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i6_start();
        let dt_sq = dt * dt;
        (self.cache.Da5 * math::HALF * dt_sq) + (self.cache.Dv5 * dt) + self.cache.Ds5
    }
    
    /// Deceleration phase, jerk limited acceleration
    pub fn a_7(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i7_start();
        (self.j_max * dt) + self.cache.Da6
    }

    /// Deceleration phase, jerk limited acceleration
    pub fn v_7(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i7_start();
        let dt_sq = dt * dt;
        (self.j_max * math::HALF * dt_sq) + (self.cache.Da6 * dt) + self.cache.Dv6
    }

    /// Deceleration phase, jerk limited acceleration
    pub fn s_7(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i7_start();
        let dt_sq = dt * dt;
        ((self.j_max * math::SIXTH * dt_sq) + (self.cache.Da6 * math::HALF * dt) + self.cache.Dv6) * dt + self.cache.Ds6
    }

    /// Constant (exit) speed at the end
    pub fn s_8(&self, t: &math::Real) -> math::Real {
        let dt = (*t) - self.i7_end();
        self.cache.Ds7 + (self.v_o * dt)
    }
}

impl motion::MotionProfile for SCurveMotionProfile {
    #[inline(always)]
    fn end_time(&self) -> math::Real {
        self.i7_end()
    }
    #[inline(always)]
    fn end_pos(&self) -> math::Real {
        self.s_o
    }

    fn eval_interval(&self, t: math::Real) -> usize {
        if t <= math::ZERO {
            0
        } else if t < self.i1_end() {
            1
        } else if t < self.i2_end() {
            2
        } else if t < self.i3_end() {
            3
        } else if t < self.i4_end() {
            4
        } else if t < self.i5_end() {
            5
        } else if t < self.i6_end() {
            6
        } else if t <= self.i7_end() {
            7
        } else {
            8
        }
    }

    /// Computes the position in [hwa::HwiContract::SPACE_UNIT_MAGNITUDE] at given relative instant (secs)
    /// * p_{1}(t) = if( 0 <= t < T_{j1} , s_{i1}(t) ) | \[0, T_{j1} \]
    /// * p_{2}(t) = if( T_{j1} <= t < T_{a} - T_{j1}, s_{i2}(t) ) | \[ T_{j1}, T_{a} - T_{j1} \]
    /// * p_{3}(t) = if( T_{a} - T_{j1} <= t < T_{a}, s_{i3}(t) ) | \[ T_{a} - T_{j1}, T_{a} \]
    /// * p_{4}(t) = if( T_{a} <= t < T_{a} + T_{v}, s_{i4}(t) ) | \[ T_{a}, T_{a} + T_{v} \]
    /// * p_{5}(t) = if( T_{a} + T_{v} <= t < T - T_{d} + T_{j2}, s_{i5}(t) ) | \[ T_{a} + T_{v}, T - T_{d} + T_{j2} \]
    /// * p_{6}(t) = if( T_{a} + T_{v} + T_{j2} <= t < T_{a} + T_{v} + T_{d} - T_{j2}, s_{i6}(t) ) | \[ T_{a} + T_{v} + T_{j2}, T_{a} + T_{v} + T_{d} - T_{j2} \]
    /// * p_{7}(t) = if( T_{a} + T_{v} <= t < T - T_{d} + T_{j2}, s_{i5}(t) ) | \[ T_{a} + T_{v}, T - T_{d} + T_{j2} \]
    fn eval_position(&self, t: math::Real) -> math::Real {
        if t <= math::ZERO {
            math::ZERO
        } else if t < self.i1_end() {
            self.s_1(&t)
        } else if t < self.i2_end() {
            self.s_2(&t)
        } else if t < self.i3_end() {
            self.s_3(&t)
        } else if t < self.i4_end() {
            self.s_4(&t)
        } else if t < self.i5_end() {
            self.s_5(&t)
        } else if t < self.i6_end() {
            self.s_6(&t)
        } else if t <= self.i7_end() {
            self.s_7(&t)
        } else {
            self.s_o
        }
    }

    fn eval_velocity(&self, t: math::Real) -> math::Real {
        if t <= math::ZERO {
            self.v_i
        } else if t < self.i1_end() {
            self.v_1(&t)
        } else if t < self.i2_end() {
            self.v_2(&t)
        } else if t < self.i3_end() {
            self.v_3(&t)
        } else if t < self.i4_end() {
            self.v_4(&t)
        } else if t < self.i5_end() {
            self.v_5(&t)
        } else if t < self.i6_end() {
            self.v_6(&t)
        } else if t <= self.i7_end() {
            self.v_7(&t)
        } else {
            self.v_o
        }
    }

    fn eval_acceleration(&self, t: math::Real) -> math::Real {
        if t <= math::ZERO {
            self.a_i
        } else if t < self.i1_end() {
            self.a_1(&t)
        } else if t < self.i2_end() {
            self.a_2(&t)
        } else if t < self.i3_end() {
            self.a_3(&t)
        } else if t < self.i4_end() {
            self.a_4(&t)
        } else if t < self.i5_end() {
            self.a_5(&t)
        } else if t < self.i6_end() {
            self.a_6(&t)
        } else if t <= self.i7_end() {
            self.a_7(&t)
        } else {
            self.a_o
        }
    }
}

impl core::fmt::Debug for SCurveMotionProfile {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "SCurveMotionProfile {{times = {{{:?}}} j_max = {:?}, a_0 = {:?} v_0 = {:?}, v_lim = {:?}, v_1 = {:?}, a_1 = {:?}, q_1 = {:?} }}",
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
#[allow(non_snake_case)]
#[derive(Copy, Clone, PartialEq, Default)]
pub struct Cache {
    /// Delta acceleration at end of interval 1
    pub Da1: math::Real,
    /// Delta velocity at end of interval 1
    pub Dv1: math::Real,
    /// Delta position at end of interval 1
    pub Ds1: math::Real,

    /// Delta acceleration at end of interval 2
    pub Da2: math::Real,
    /// Delta velocity at end of interval 2
    pub Dv2: math::Real,
    /// Delta position at end of interval 2
    pub Ds2: math::Real,

    /// Delta acceleration at end of interval 3
    pub Da3: math::Real,
    /// Delta velocity at end of interval 3
    pub Dv3: math::Real,
    /// Delta position at end of interval 3
    pub Ds3: math::Real,

    /// Delta acceleration at end of interval 4
    pub Da4: math::Real,
    /// Delta velocity at end of interval 4
    pub Dv4: math::Real,
    /// Delta position at end of interval 4
    pub Ds4: math::Real,

    /// Delta acceleration at end of interval 5
    pub Da5: math::Real,
    /// Delta velocity at end of interval 5
    pub Dv5: math::Real,
    /// Delta position at end of interval 5
    pub Ds5: math::Real,

    /// Delta acceleration at end of interval 6
    pub Da6: math::Real,
    /// Delta velocity at end of interval 6
    pub Dv6: math::Real,
    /// Delta position at end of interval 6
    pub Ds6: math::Real,

    /// Delta acceleration at end of interval 7
    pub Da7: math::Real,
    /// Delta velocity at end of interval 7
    pub Dv7: math::Real,
    /// Delta position at end of interval 7
    pub Ds7: math::Real,
}

#[cfg(test)]
pub mod test {
    //Example 3.9

    use crate::{hwa, motion, processing};
    use hwa::math;
    use motion::{Constraints, SCurveMotionProfile};
    use num_traits::ToPrimitive;

    pub fn do_compute(
        q1: f32,
        v_0: f32,
        v_1: f32,
        v_max: f32,
        a_max: f32,
        j_max: f32,
        error_correction: bool,
    ) -> Result<SCurveMotionProfile, processing::CodeExecutionFailure> {
        let constraints = Constraints {
            v_max: math::Real::from_f32(v_max),
            a_max: math::Real::from_f32(a_max),
            j_max: math::Real::from_f32(j_max),
        };
        SCurveMotionProfile::compute(
            math::Real::from_f32(q1),
            math::Real::from_f32(v_0),
            math::Real::from_f32(v_1),
            &constraints,
            error_correction,
        )
    }

    fn approx_equal(what: &str, v: math::Real, expected: f32, tolerance: f32) {
        let v1 = v.to_f64().to_f32().unwrap();
        let ok = (v1 - expected).abs() < tolerance;
        assert!(ok, "{} = {} but should be = {}", what, v1, expected);
    }

    fn init_logger() {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-log")] {
                use std::io::Write;
                let env = env_logger::Env::new().default_filter_or("info");
                let _ = env_logger::builder()
                    .parse_env(env)
                    .format(|buf, record| {
                        writeln!(buf, "{}: {}", record.level(), record.args())
                    })
                    .try_init();
            }
        }
    }

    #[test]
    fn ex_3_9() {
        // With: q_1 = 10, v_0 = 1, v_1 = 0
        // Given: v_max = 5, a_max = 10, j_max = 30
        // Exp: T_a = 0.7333, T_v = 1.1433, T_d = 0.8333, T_j1 = 0.3333, T_j2 = 0.3333
        let r = do_compute(10., 1., 0., 5., 10., 30., false).unwrap();

        approx_equal("T_a", r.t_a, 0.7333, 0.001);
        approx_equal("T_d", r.t_d, 0.8333, 0.001);
        approx_equal("T_j1", r.t_j1, 0.3333, 0.001);
        approx_equal("T_j2", r.t_j2, 0.3333, 0.001);
        approx_equal("T_v", r.t_v, 1.1433, 0.001);

        let r = do_compute(10., 1., 0., 5., 10., 30., true).unwrap();

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
        let r = do_compute(10., 1., 0., 10., 10., 30., false).unwrap();

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
        let r = do_compute(10., 7., 0., 10., 10., 30., false).unwrap();

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
        let r = do_compute(10., 7.5, 0., 10., 10., 30., false).unwrap();

        approx_equal("T_a", r.t_a, 0.0, 0.01);
        approx_equal("T_v", r.t_v, 0.0, 0.001);
        approx_equal("T_d", r.t_d, 2.6667, 0.01);
        approx_equal("T_j1", r.t_j1, 0.0, 0.01);
        approx_equal("T_j2", r.t_j2, 0.0973, 0.01);
        approx_equal("v_lim", r.v_lim, 7.5, 0.01);
    }

    #[test]
    fn ex_3_13() {
        init_logger();
        // With: q_1 = 10, v_0 = 0, v_1 = 0
        // Given: v_max = 10, a_max = 20, j_max = 30
        // Exp: Ta = 1.1006, T_v = 0.0, T_d = 1.1006, T_j1 = 0.5503, T_j2 = 0.5503, vlim = 9.0826
        let r = do_compute(10., 0.0, 0., 10., 20., 30., false).unwrap();

        #[cfg(feature = "debug-motion-planning")]
        r.params_dump();
        hwa::info!("profile: {:?}", r);
        hwa::info!("constraints: {:?}", r.constraints);

        approx_equal("T_a", r.t_a, 1.1006, 0.01);
        approx_equal("T_v", r.t_v, 0.0, 0.001);
        approx_equal("T_d", r.t_d, 1.1006, 0.01);
        approx_equal("T_j1", r.t_j1, 0.5333, 0.01);
        approx_equal("T_j2", r.t_j2, 0.5333, 0.01);
        approx_equal("v_lim", r.v_lim, 9.0826, 0.01);
    }

    #[test]
    fn test_limits() {
        use crate::motion::MotionProfile;
        let r = do_compute(10., 1., 0., 10., 10., 30., false).unwrap();

        assert_eq!(
            MotionProfile::eval_position(&r, math::ZERO - math::ONE),
            math::ZERO,
        );

        let pos = MotionProfile::eval_position(&r, math::ONE_HUNDRED);
        approx_equal("pos", pos, 10.0, 0.0001);
    }

    #[test]
    fn test_bad_args() {
        let profile = do_compute(10., 0., 0., 10., 0., 0., false);
        assert_eq!(
            profile,
            Err(processing::CodeExecutionFailure::NumericalError)
        );

        // TODO: s_i8()

        // fn compute_cache(&mut self) -> Result<(),  processing::CodeExecutionFailure> {
        //         if self.v_lim.is_negligible() {
        //             return Err(CodeExecutionFailure::NumericalError);
        //         }
    }
}
