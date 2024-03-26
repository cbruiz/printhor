//! The Motion profile trait and current implementation: Trajectory with Double S Velocity Profile [[1]]
//!
//! [[1]] Biagiotti, L., Melchiorri, C.: Trajectory Planning for Automatic Machines and Robots. Springer, Heidelberg (2008). [DOI:10.1007/978-3-540-85629-0](https://doi.org/10.1007/978-3-540-85629-0)

use crate::{hwa, math};
#[cfg(feature = "native")]
use core::fmt::Display;
#[cfg(feature = "native")]
use core::fmt::Formatter;
use core::ops::{Div, Neg};
use core::ops::Mul;
#[cfg(all(feature = "native", feature = "plot-motion-plan"))]
use num_traits::float::FloatCore;
#[cfg(all(feature = "native", feature = "plot-motion-plan"))]
use crate::{math::Real, math::RealInclusiveRange};
#[cfg(all(feature = "native", feature = "plot-motion-plan"))]
use rust_decimal::prelude::ToPrimitive;
#[cfg(all(not(feature = "native"), feature = "plot-motion-plan"))]
use num_traits::ToPrimitive;
use crate::math::*;
use crate::control::CodeExecutionFailure;

pub trait MotionProfile {

    fn end(&self) -> Real;

    fn eval_position(&self, t: Real) -> Option<Real>;
}

#[derive(Clone, Default)]
pub struct Boundaries {
    /// Final position
    pub q_1: Real,
    /// Start speed
    pub v_0: Real,
    /// Final speed
    pub v_1: Real,
}

#[cfg(feature = "native")]
impl Display for Boundaries {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f,
               "q_{{1}}={} v_{{0}}={} v_{{1}}={}", self.q_1, self.v_0, self.v_1
        )
    }
}

#[derive(Clone, Copy, Default)]
pub struct Constraints {
    /// Max velocity
    pub v_max: Real,
    /// Maximum acceleration
    pub a_max: Real,
    /// Maxium jerk
    pub j_max: Real,
}

#[cfg(feature = "native")]
impl Display for Constraints {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f,
               "v_{{max}}={} a_{{max}}={} j_{{max}}={}", self.v_max, self.a_max, self.j_max
        )
    }
}

#[derive(Clone, Copy, Default)]
pub struct SCurveMotionProfile {
    pub t_j1: Real,
    pub t_a: Real,
    pub t_v: Real,
    pub t_d: Real,
    pub t_j2: Real,

    pub v_0: Real,
    pub v_1: Real,
    pub j_max: Real,
    pub j_min: Real,
    pub a_lim_a: Real,
    pub a_lim_d: Real,
    pub v_lim: Real,
    pub q1: Real,

    pub constraints: Constraints,

    pub cache: Cache,
}

#[allow(unused)]
impl SCurveMotionProfile {
    pub fn compute(q_1: Real, v_0: Real, v_1:Real, constraints: &Constraints, error_correction: bool) -> Result<SCurveMotionProfile, CodeExecutionFailure>  {

        hwa::debug!("compute q_{{1}} = {} v_{{0}} = {} v_{{1}} = {}", q_1, v_0, v_1);
        let _t0 = embassy_time::Instant::now();

        let t_jmax = constraints.a_max / constraints.j_max;
        let t_jstar = Real::min(
            Some((((v_1 - v_0).abs()) / constraints.j_max).sqrt().unwrap()),
            Some(t_jmax),
        ).unwrap();

        hwa::debug!("t_jstar = {} t_jmax = {}", t_jstar, t_jmax);

        let feasible = if t_jstar < t_jmax {
            q_1 > t_jstar * (v_0 + v_1)
        }
        else if t_jstar == t_jmax {
            q_1 > ((v_0 + v_1) / TWO)
                * (t_jstar + ((v_1 - v_0).abs() / constraints.a_max))
        }
        else {
            hwa::error!("unhandled situation");
            return Err(CodeExecutionFailure::ERR)
        };
        if !feasible {
            hwa::error!("Movement NOT FEASIBLE");
            return Err(CodeExecutionFailure::ERR)
        }

        let amax_squared = constraints.a_max * constraints.a_max;

        let (t_j1, t_a) = if (constraints.v_max - v_0) * constraints.j_max < amax_squared {
            let t_j1 = ( ( constraints.v_max - v_0 ).abs() / constraints.j_max ).sqrt().ok_or(CodeExecutionFailure::NumericalError)?;
            (t_j1, t_j1 + t_j1)
        } else {
            let t_j1 = constraints.a_max / constraints.j_max;
            (t_j1, t_j1 + (constraints.v_max - v_0).abs() / constraints.a_max)
        };

        let (t_j2, t_d) = if (constraints.v_max - v_1) * constraints.j_max < amax_squared {
            let t_j1 = ( ( constraints.v_max - v_1 ).abs() / constraints.j_max ).sqrt().ok_or(CodeExecutionFailure::NumericalError)?;
            (t_j1, t_j1 + t_j1)
        } else {
            let t_j1 = constraints.a_max / constraints.j_max;
            (t_j1, t_j1 + (constraints.v_max - v_1).abs() / constraints.a_max)
        };

        let t_v = (q_1 / constraints.v_max )
            - ((t_a * HALF) * ( ONE + (v_0 / constraints.v_max)))
            - ((t_d * HALF) * ( ONE + (v_1 / constraints.v_max)));

        let (mut intervals, _j_max) = if t_v >= ZERO {

            let a_lim_a = constraints.j_max * t_j1;
            let a_lim_d = constraints.j_max * t_j2;
            let v_lim = v_0 + (t_a - t_j1) * a_lim_a;
            let t = t_a + t_v + t_d;
            (SCurveMotionProfile {
                t_j1,
                t_a,
                t_v,
                t_d,
                t_j2,
                v_0: v_0,
                v_1: v_1,
                j_max: constraints.j_max,
                j_min: constraints.j_max.neg(),
                a_lim_a,
                a_lim_d,
                v_lim,
                q1: q_1,
                cache: Cache::default(),
                constraints: *constraints,
            }, constraints.j_max)
        }
        else {

            let mut gamma = ONE;
            let mut t_a_2 = ZERO;
            let mut t_d_2 = ZERO;
            let mut t_j = ZERO;
            #[allow(unused_assignments)]
                let mut t_j1_2 = ZERO;
            #[allow(unused_assignments)]
                let mut t_j2_2 = ZERO;
            #[allow(unused_variables)]
                let mut v_lim = ZERO;

            let mut a_max_2 = constraints.a_max;

            for _i in 0..10 {
                // TODO: lagrange multipliers
                //println!("Trying with gamma={}", gamma.rdp(4));

                t_j = a_max_2 / constraints.j_max;

                let sqrt_delta = (((a_max_2 * a_max_2 * a_max_2 * a_max_2) / (constraints.j_max * constraints.j_max))
                    + (TWO * ((v_0 * v_0) + (v_1 * v_1))
                    + (a_max_2 * ((FOUR * q_1)
                    - (TWO * (a_max_2 / constraints.j_max) * (v_0 + v_1)))))).sqrt().unwrap();
                let aj = (a_max_2 * a_max_2) / constraints.j_max;
                t_a_2 = (aj - (TWO * v_0) + sqrt_delta) / (TWO * a_max_2);
                t_d_2 = (aj - (TWO * v_1) + sqrt_delta) / (TWO * a_max_2);

                let a_lim_a = constraints.j_max * t_j;
                v_lim = v_0 + (t_a_2 - t_j) * a_lim_a;

                if t_a_2 > (TWO * t_j) && t_d_2 > (TWO * t_j) {
                    break;
                }
                gamma = gamma * HALF;
                a_max_2 = constraints.a_max * gamma;
            }
            t_j1_2 = t_j;
            t_j2_2 = t_j;
            if t_a_2 < Real::zero() {
                t_a_2 = Real::zero();
                t_j1_2 = Real::zero();
                t_d_2 = (q_1.mul(TWO)).div(v_1 + v_0);
                t_j2_2 = (constraints.j_max.mul(q_1) - (constraints.j_max.mul(
                    (constraints.j_max.mul(q_1.powi(2))) + ((v_1 + v_0).powi(2)).mul(v_1 - v_0)
                )
                ).sqrt().unwrap()).div(
                    constraints.j_max.mul(v_1 + v_0)
                )
            }
            else if t_d_2 < Real::zero() {
                t_d_2 = Real::zero();
                t_j2_2 = Real::zero();
                t_a_2 = (q_1.mul(TWO)).div(v_1 + v_0);
                t_j1_2 = (constraints.j_max.mul(q_1) - (constraints.j_max.mul(
                    (constraints.j_max.mul(q_1.powi(2))) + ((v_1 + v_0).powi(2)).mul(v_0 - v_1)
                )
                ).sqrt().unwrap()).div(
                    constraints.j_max.mul(v_1 + v_0)
                )
            }
            let a_lim_a = constraints.j_max * t_j1_2;
            let a_lim_d = constraints.j_max * t_j2_2;
            let t = t_a_2 + t_d_2;
            (SCurveMotionProfile {
                t_j1: t_j1_2,
                t_a: t_a_2,
                t_v: ZERO,
                t_d: t_d_2,
                t_j2: t_j2_2,
                v_0,
                v_1,
                j_max: constraints.j_max,
                j_min: constraints.j_max.neg(),
                a_lim_a,
                a_lim_d,
                v_lim,
                q1: q_1,
                cache: Cache::default(),
                constraints: *constraints,
            }, a_max_2)
        };

        cfg_if::cfg_if! {
            if #[cfg(feature="timing-stats")] {
                let _t = intervals.t_a + intervals.t_v + intervals.t_d;
                #[allow(unused)]
                let a_lim_a = constraints.j_max * intervals.t_j1;
                #[allow(unused)]
                let a_lim_d = constraints.j_max.neg() * intervals.t_j2;
                #[allow(unused)]
                let v_lim = v_0 + ((intervals.t_a - intervals.t_j1) * a_lim_a);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature="verbose-timings")] {
                hwa::info!("Motion plan computed in {} us", _t0.elapsed().as_micros());
            }
        }
        intervals.compute_cache();
        if error_correction {
            let final_pos = intervals.s_i7(&intervals.i7_end());
            let delta_e = intervals.q1 - final_pos;
            if delta_e > ZERO {
                intervals.extend(delta_e).map_err(|_| CodeExecutionFailure::ERR)?;
            }
        }
        Ok(intervals)
    }

    /// The time at the start of 1st interval
    /// Interval is:
    /// \[ 0, T_{j1}\]
    #[inline]
    pub fn i1_start(&self) -> Real {
        Real::zero()
    }

    /// The time at the end of 1st interval
    /// Interval is:
    /// \[ 0, T_{j1}\]
    #[inline]
    pub fn i1_end(&self) -> Real {
        self.i2_start()
    }

    /// The time at the start of 2nd interval
    /// Interval is:
    /// \[ T_{j1}, T_{a} - T_{j1}\]
    #[inline]
    pub fn i2_start(&self) -> Real {
        self.t_j1
    }

    /// The time at the end of 2nd interval
    /// Interval is:
    /// \[ T_{j1}, T_{a} - T_{j1}\]
    #[inline]
    pub fn i2_end(&self) -> Real {
        self.i3_start()
    }

    /// The time at the start of 3rd interval
    /// Interval is:
    /// \[ T_{a} - T_{j1}, T_{a}\]
    #[inline]
    pub fn i3_start(&self) -> Real {
        self.t_a - self.t_j1
    }

    /// The time at the end of 3rd interval
    /// Interval is:
    /// \[ T_{a} - T_{j1}, T_{a}\]
    #[inline]
    pub fn i3_end(&self) -> Real {
        self.i4_start()
    }

    /// The time at the start of 4th interval
    /// Interval is:
    /// \[T_a, T_a + T_v\]
    #[inline]
    pub fn i4_start(&self) -> Real {
        self.t_a
    }

    /// The time at the end of 4th interval
    /// Interval is:
    /// \[T_a, T_a + T_v\]
    #[inline]
    pub fn i4_end(&self) -> Real {
        self.i5_start()
    }

    /// The time at the start of 5th interval
    /// Interval is:
    /// \[T_a + T_v, T - T_d + T_j2\]
    ///
    /// \[T_a + T_v, T_a + T_v + T_j2\]
    #[inline]
    pub fn i5_start(&self) -> Real {
        self.t_a + self.t_v
    }

    /// The time at the end of 5th interval
    /// Interval is:
    /// \[T_a + T_v, T - T_d + T_j2\]
    ///
    /// \[T_a + T_v, T_a + T_v + T_j2\]
    #[inline]
    pub fn i5_end(&self) -> Real {
        self.i6_start()
    }

    /// The time at the start of 6th interval
    /// Interval is:
    /// \[T - T_{d} + T_{j2}, T - T_{j2}\]
    ///
    /// \[T_{a} + T_{v} + T_{j2}, T_{a} + T_{v} + T_{d} - T_{j2}\]
    #[inline]
    pub fn i6_start(&self) -> Real {
        self.t_a + self.t_v + self.t_j2
    }

    /// The time at the end of 7th interval
    /// Interval is:
    /// \[T - T_d + T_j2, T - T_j2\]
    ///
    /// \[T_a + T_v + T_j2, T_a + T_v + T_d - T_j2\]
    #[inline]
    pub fn i6_end(&self) -> Real {
        self.i7_start()
    }

    /// The time at the start of 7th interval
    /// Interval is:
    /// \[T - T_j2, T\]
    ///
    /// \[T_a + T_v + T_d - T_j2, T_a + T_v + T_d\]
    #[inline]
    pub fn i7_start(&self) -> Real {
        self.t_a + self.t_v + self.t_d - self.t_j2
    }

    /// The time at the end of 7th interval
    /// Interval is:
    /// \[T - T_j2, T\]
    ///
    /// \[T_a + T_v + T_d - T_j2, T_a + T_v + T_d\]
    #[inline]
    pub fn i7_end(&self) -> Real {
        self.t_a + self.t_v + self.t_d
    }

    /// Compute intermediate piecewise function points to speedup equations
    fn compute_cache(&mut self)  {
        self.cache.s1_pt = self.s_i1(&self.i1_end());
        self.cache.s2_pt = self.s_i2(&self.i2_end());
        self.cache.s3_pt = self.s_i3(&self.i3_end());
        self.cache.s4_pt = self.s_i4(&self.i4_end());
        self.cache.s5_pt = self.s_i5(&self.i5_end());
        self.cache.s6_pt = self.s_i6(&self.i6_end());
        self.cache.s7_pt = self.s_i7(&self.i7_end());
    }

    pub fn extend(&mut self, delta_e: Real)  -> Result<(), ()> {
        let extra_time =  delta_e / self.v_lim;
        self.t_v += extra_time;
        self.cache.s4_pt += delta_e;
        self.cache.s5_pt += delta_e;
        self.cache.s6_pt += delta_e;
        self.cache.s7_pt += delta_e;
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
        }
        else if t >= self.i1_start() && t < self.i1_end() {
            (1, Some(self.s_i1(&t)))
        }
        else if t >= self.i2_start() && t < self.i2_end() {
            (2, Some(self.s_i2(&t)))
        }
        else if t >= self.i3_start() && t < self.i3_end() {
            (3, Some(self.s_i3(&t)))
        }
        else if t >= self.i4_start() && t < self.i4_end() {
            (4, Some(self.s_i4(&t)))
        }
        else if t >= self.i5_start() && t < self.i5_end() {
            (5, Some(self.s_i5(&t)))
        }
        else if t >= self.i6_start() && t < self.i6_end() {
            (6, Some(self.s_i6(&t)))
        }
        else if t >= self.i7_start() && t <= self.i7_end() {
            (7, Some(self.s_i7(&t)))
        }
        else {
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
        (self.j_max * HALF * self.t_j1 * dt) * (dt + self.t_j1)
                + self.v_0 * dt + self.cache.s1_pt
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
    fn end(&self) -> Real {
        self.i7_end()
    }

    fn eval_position(&self, t: Real) -> Option<Real> {
        if t < math::ZERO {
            None
        }
        else if t >= self.i1_start() && t < self.i1_end() {
            Some(self.s_i1(&t))
        }
        else if t >= self.i2_start() && t < self.i2_end() {
            Some(self.s_i2(&t))
        }
        else if t >= self.i3_start() && t < self.i3_end() {
            Some(self.s_i3(&t))
        }
        else if t >= self.i4_start() && t < self.i4_end() {
            Some(self.s_i4(&t))
        }
        else if t >= self.i5_start() && t < self.i5_end() {
            Some(self.s_i5(&t))
        }
        else if t >= self.i6_start() && t < self.i6_end() {
            Some(self.s_i6(&t))
        }
        else if t >= self.i7_start() && t <= self.i7_end() {
            Some(self.s_i7(&t))
        }
        else {
            None
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
impl Display for SCurveMotionProfile {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f,
               "\n\tt_j1={}, t_a={}, t_v={}, t_d={}, t_j2={}",
               self.t_j1.rdp(4),
               self.t_a.rdp(4),
               self.t_v.rdp(4),
               self.t_d.rdp(4),
               self.t_j1.rdp(4),
        )?;
        write!(f,
               "\n\ta_lim_a={}, v_lim={}, a_lim_d={}",
               self.a_lim_a,
               self.v_lim,
               self.a_lim_d,
        )
    }
}

pub struct PlanProfile {
    plan: SCurveMotionProfile,
    lin_space: RealInclusiveRange,
    #[allow(unused)]
    advanced: Real,
}


#[allow(unused)]
impl PlanProfile
{
    pub fn new(intervals: SCurveMotionProfile, step_size: Real) -> Self {

        let lin_space = RealInclusiveRange::new(
            ZERO,
            intervals.i7_end(),
            step_size);
        Self{
            plan: intervals,
            lin_space,
            advanced: Real::zero(),
        }
    }

    #[allow(unused)]
    #[inline]
    pub fn reset(&mut self) {
        self.advanced = Real::zero();
        self.lin_space.reset();
    }

    #[allow(unused)]
    #[inline]
    pub fn total_duration(&self) -> Real {
        self.lin_space.width()

    }

    #[allow(unused)]
    #[allow(dead_code)]
    #[cfg(feature = "native")]
    pub fn plot(&mut self, plot_pos: bool, plot_vel: bool, plot_accel: bool, plot_jerk: bool) {

        use gnuplot::{AxesCommon, Figure};
        use gnuplot::{AutoOption, Tick, MultiplotFillOrder::RowsFirst, MultiplotFillDirection::{Downwards}};
        use gnuplot::{DashType, PlotOption};

        let p = &(self.plan);

        let steps_per_unit = Real::from_lit(2, 0);

        let mut time: Vec<f64> = Vec::with_capacity(1000);
        let mut time_step_by_pulse: Vec<f64> = Vec::with_capacity(1000);
        let mut step_by_pulse: Vec<f64> = Vec::with_capacity(1000);
        let mut pos: Vec<f64> = Vec::with_capacity(1000);

        let mut spd: Vec<f64> = Vec::with_capacity(1000);
        let mut acc: Vec<f64> = Vec::with_capacity(1000);
        let mut jerk: Vec<f64> = Vec::with_capacity(1000);

        let mut inter: Vec<f64> = Vec::with_capacity(1000);

        let mut time_ref = &mut time;
        let mut pos_ref = &mut pos;
        let mut spd_ref = &mut spd;
        let mut acc_ref = &mut acc;
        let mut jerk_ref = &mut jerk;
        let mut cnt = 0;
        let mut last_t = 0.;
        let mut last_p = 0.;
        let mut last_s = 0.;
        let mut last_a = 0.;
        let mut last_j = 0.;
        let mut real_advanced = Real::zero();
        let mut abs_advanced = 0u32;
        let mut abs_steps_advanced = 0u32;

        time_step_by_pulse.push(last_t);
        step_by_pulse.push(f64::nan());

        let pv0 = self.plan.v_0.rdp(4);
        let pv1 = self.plan.v_1.rdp(4);
        let pvm = self.plan.v_lim.rdp(4);
        let pdisp = self.plan.q1.rdp(4);
        let pres = self.lin_space.step_size().rdp(4);

        for (inte, ti, pos) in self.into_iter() {

            real_advanced = pos;

            let ti_v = ti.inner();
            let real_pos = pos.max(Real::zero()).inner();
            let c_ti = ti_v.to_f64().unwrap();
            let c_pos = real_pos.to_f64().unwrap();

            let dt = c_ti.clone() - last_t;

            let pos_abs = real_pos.to_i32().unwrap() as u32;

            let real_steps = (pos * steps_per_unit).round();

            let abs_steps = real_steps.to_i64().unwrap() as u32;
            let steps = core::cmp::max(abs_steps, abs_steps_advanced) - abs_steps_advanced;
            abs_steps_advanced = abs_steps;

            if steps > 0 {
                let dt_step = dt / (steps as f64);
                let mut dt_acc = ti_v.to_f64().unwrap();
                for _i in 0..steps {
                    time_step_by_pulse.push(dt_acc);
                    step_by_pulse.push(abs_steps_advanced as f64);
                    abs_steps_advanced += 1;
                    time_step_by_pulse.push(dt_acc);
                    step_by_pulse.push(abs_steps_advanced as f64);
                    dt_acc += dt_step;
                }
            }

            abs_advanced = pos_abs;

            let s = (c_pos - last_p) / dt.clone().abs();
            let a = ((s.clone() - last_s) / dt.clone());
            let j = ((a.clone() - last_a) / dt.clone());

            time_ref.push(c_ti.clone());
            pos_ref.push(c_pos.clone());

            inter.push(inte.to_f64().unwrap());

            if plot_vel {
                spd_ref.push(s.clone());
            }
            if plot_accel {
                acc_ref.push(a.clone());
            }
            if plot_jerk {
                jerk_ref.push(j.clone());
            }
            cnt += 1;
            last_t = c_ti;
            last_p = c_pos;
            last_s = if s.is_nan() {0.} else {s};
            last_a = if a.is_nan() {0.} else {a};
            last_j = if j.is_nan() {0.} else {j};
        }
        time_step_by_pulse.push(last_t);
        step_by_pulse.push(abs_steps_advanced as f64);
        println!("D; Advanced: est: {} mm, real: {:.03} mm ({} steps)", real_advanced.rdp(4), (abs_steps_advanced.to_f64().unwrap() / steps_per_unit.to_f64()), abs_steps_advanced);
        let mut fg = Figure::new();

        fg.set_multiplot_layout(6, 1)
            .set_title(
                format!(
                    "FixedPoint Double S-Curve velocity profile\n[vin={} mm/s, vmax={} mm/s, vout={} mm/s, displacement={} mm, sample\\_period={} s]",
                    pv0, pvm, pv1, pdisp, pres,
                ).as_str()
            )
            .set_scale(1.0, 1.0)
            .set_offset(0.0, 0.0)
            .set_multiplot_fill_order(RowsFirst, Downwards);

        fg.axes2d()
            .set_y_label("Position (mm)", &[])
            .set_cb_range(AutoOption::Fix(self.advanced.inner().to_f64().unwrap() * 0.5), AutoOption::Auto)
            .lines(time.clone(), pos.clone(), &[PlotOption::Color("blue")])
        ;
        fg.axes2d()
            .set_y_label("interv", &[])
            .set_cb_range(AutoOption::Fix(self.advanced.inner().to_f64().unwrap()), AutoOption::Auto)

            .lines(time.clone(), inter.clone(), &[PlotOption::Color("black")])
        ;
        fg.axes2d()
            .set_y_label("Velocity (mm/s)", &[])
            .set_cb_range(AutoOption::Fix(self.advanced.inner().to_f64().unwrap()), AutoOption::Auto)

            .lines(time.clone(), spd.clone(), &[PlotOption::Color("green")])
        ;
        fg.axes2d()
            .set_y_label("Acceleration (mm/s²)", &[])
            .set_cb_range(AutoOption::Fix(self.advanced.inner().to_f64().unwrap()), AutoOption::Auto)

            .lines(time.clone(), acc.clone(), &[PlotOption::Color("red")])
        ;
        fg.axes2d()
            .set_y_label("Jerk (m/s³)", &[])
            .set_cb_range(AutoOption::Fix(self.advanced.inner().to_f64().unwrap()), AutoOption::Auto)

            .lines(time.clone(), jerk.clone(), &[PlotOption::Color("orange")])
        ;
        fg.axes2d()
            .set_x_label("Time in seconds", &[])
            .set_y_label("Discrete\nsteps", &[])
            .set_x2_grid(true)
            .set_x2_minor_grid(true)
            .set_cb_range(AutoOption::Fix(self.advanced.inner().to_f64().unwrap()), AutoOption::Auto)

            .set_grid_options(true, &[PlotOption::Color("gray"),PlotOption::LineStyle(DashType::Dash)])
            .lines(time_step_by_pulse.clone(), step_by_pulse.clone(), &[PlotOption::Color("black")])
        ;
        fg.show_and_keep_running().unwrap();
    }

}

impl Iterator for PlanProfile
{
    type Item = (u8, Real, Real);

    fn next(&mut self) -> Option<Self::Item> {
        match self.lin_space.next() {
            None => {
                None
            },
            Some(tp) => {
                let (s, v) = self.plan.eval_position(tp);
                Some((s, tp, v?))
            }
        }
    }
}
