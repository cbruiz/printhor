use crate::hwa;
#[cfg(feature = "native")]
use core::fmt::Display;
#[cfg(feature = "native")]
use core::fmt::Formatter;
use core::ops::{Div, Neg};
use core::ops::Mul;
//use embassy_time::{Instant, Duration};
#[cfg(all(feature = "native", feature = "plot-motion-plan"))]
use num_traits::float::FloatCore;
use crate::{math::Real, math::RealInclusiveRange};
#[cfg(all(feature = "native", feature = "plot-motion-plan"))]
use rust_decimal::prelude::ToPrimitive;
use crate::math::{FOUR, HALF, ONE, SIX, THREE, TWO, ZERO};
use crate::control::planner::CodeExecutionFailure;

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

#[derive(Clone, Default)]
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

    pub t: Real,

    pub v_0: Real,
    pub v_1: Real,
    pub j_max: Real,
    pub j_min: Real,
    pub a_lim_a: Real,
    pub a_lim_d: Real,
    pub v_lim: Real,
    pub q1: Real,
}

#[allow(unused)]
impl SCurveMotionProfile {
    pub fn compute(q_1: Real, v_0: Real, v_1:Real, constraints: &Constraints) -> Result<SCurveMotionProfile, CodeExecutionFailure>  {

        let _t0 = embassy_time::Instant::now();

        let t_jmax = constraints.a_max / constraints.j_max;
        let t_jstar = Real::min(
            Some((((v_1 - v_0).abs()) / constraints.j_max).sqrt().unwrap()),
            Some(t_jmax),
        ).unwrap();

        hwa::trace!("t_jstar = {} t_jmax = {}", t_jstar, t_jmax);

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
            let t_j1 = ( ( constraints.v_max - v_0 ) / constraints.j_max ).sqrt().ok_or(CodeExecutionFailure::NumericalError)?;
            (t_j1, t_j1 * TWO)
        } else {
            let t_j1 = constraints.a_max / constraints.j_max;
            (t_j1, t_j1 + (constraints.v_max - v_0) / constraints.a_max)
        };

        let (t_j2, t_d) = if (constraints.v_max - v_1) * constraints.j_max < amax_squared {
            let t_j1 = ( ( constraints.v_max - v_1 ) / constraints.j_max ).sqrt().ok_or(CodeExecutionFailure::NumericalError)?;
            (t_j1, t_j1 * TWO)
        } else {
            let t_j1 = constraints.a_max / constraints.j_max;
            (t_j1, t_j1 + (constraints.v_max - v_1) / constraints.a_max)
        };

        let t_v = (q_1 / constraints.v_max )
            - ((t_a / TWO) * ( ONE + (v_0 / constraints.v_max)))
            - ((t_d / TWO) * ( ONE + (v_1 / constraints.v_max)));

        let (intervals, _j_max) = if t_v >= ZERO {
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
                t,
                v_0: v_0,
                v_1: v_1,
                j_max: constraints.j_max,
                j_min: constraints.j_max.neg(),
                a_lim_a,
                a_lim_d,
                v_lim,
                q1: q_1,
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
                t,
                v_0,
                v_1,
                j_max: constraints.j_max,
                j_min: constraints.j_max.neg(),
                a_lim_a,
                a_lim_d,
                v_lim,
                q1: q_1,
            }, a_max_2)
        };

        let _t = intervals.t_a + intervals.t_v + intervals.t_d;

        #[allow(unused)]
            let a_lim_a = constraints.j_max * intervals.t_j1;
        #[allow(unused)]
            let a_lim_d = constraints.j_max.neg() * intervals.t_j2;
        #[allow(unused)]
            let v_lim = v_0 + ((intervals.t_a - intervals.t_j1) * a_lim_a);

        hwa::debug!("Motion plan computed in {} us", _t0.elapsed().as_micros());
        Ok(intervals)
    }

    pub fn recalculate(&mut self) {

    }

    #[inline]
    fn t1(&self) -> Real {
        self.t_j1
    }
    #[inline]
    fn t2(&self) -> Real {
        self.t_a - self.t_j1
    }

    #[inline]
    fn t3(&self) -> Real {
        self.t_a
    }

    #[inline]
    fn t4(&self) -> Real {
        self.t_a + self.t_v
    }

    #[inline]
    fn t5(&self) -> Real {
        self.t - self.t_d + self.t_j2
    }

    #[inline]
    fn t6(&self) -> Real {
        self.t - self.t_j2
    }

    #[inline]
    fn t7(&self) -> Real {
        self.t
    }

    /// Computes the position (steps) in given timestamp (uS)
    pub fn eval_position(&self, t_i: Real) -> Option<Real> {
        let v0 =  self.v_0;
        let v1 = self.v_1;

        if t_i <= self.t1() {
            Some((v0 * t_i) + (self.j_max * t_i.powi(3) / SIX))
        } else if t_i <= self.t2() {
            Some((v0 * t_i) + ((self.a_lim_a * ((THREE * t_i.powi(2)) - (THREE * self.t_j1 * t_i) + self.t_j1.powi(2))) / SIX))
        } else if t_i <= self.t3() {
            Some((self.v_lim + v0) * self.t_a / TWO - self.v_lim * (self.t_a - t_i) - self.j_min * (self.t_a - t_i).powi(3) / SIX)
        } else if t_i <= self.t4() {
            Some((self.v_lim + v0) * self.t_a / TWO + self.v_lim * (t_i - self.t_a))
        } else if t_i <= self.t5() {
            Some(self.q1 - (self.v_lim + v1) * self.t_d / TWO
                + self.v_lim * (t_i - self.t + self.t_d)
                - self.j_max * (t_i - self.t + self.t_d).powi(3) / SIX)
        } else if t_i <= self.t6() {
            Some(self.q1 - (self.v_lim + v1) * self.t_d / TWO
                + self.v_lim * (t_i - self.t + self.t_d)
                - (self.a_lim_d / SIX) * (
                    (THREE * ((t_i - self.t + self.t_d).powi(2)))
                        - (THREE * self.t_j2 * (t_i - self.t + self.t_d))
                        + self.t_j2.powi(2)
                )
            )
        } else if t_i <= self.t7() {
            Some(self.q1 - v1 * (self.t - t_i) - self.j_max * (self.t - t_i).powi(3) / SIX)
        } else {
            None
        }
    }

    /*
    pub fn iterate(&self, ref_time: Instant, offset: Duration) -> SCurveRealTimeIterator {
        SCurveRealTimeIterator::new(self, ref_time, offset)
    }
     */
}

/*
pub struct SCurveRealTimeIterator<'a> {
    profile: &'a SCurveMotionProfile,
    ref_time: Instant,
    offset: Duration,
    exhausted: bool,
}
impl<'a> SCurveRealTimeIterator<'a> {
    pub const fn new(profile: &'a SCurveMotionProfile, ref_time: Instant, offset: Duration) -> Self {
        Self {
            profile,
            ref_time,
            offset,
            exhausted: false,
        }
    }
}

impl<'a> Iterator for SCurveRealTimeIterator<'a> {
    type Item = Real;

    fn next(&mut self) -> Option<Self::Item> {
        todo!("WIP")
    }
}
*/

#[cfg(feature = "native")]
impl Display for SCurveMotionProfile {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f,
               "\n\tt_j1={}",
               self.t_j1.rdp(4)
        )
    }
}

//#[test]
#[allow(unused)]
pub fn test() -> Result<(),()>{
    /// TODO
    Ok(())
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
            intervals.t,
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

        let t :Vec<Tick<f64, std::string::String>> = vec!(
            Tick::Major(p.t1().to_f64(), AutoOption::Fix(std::string::String::from("t1"))),
            Tick::Major(p.t2().to_f64(), AutoOption::Fix(std::string::String::from("t2"))),
            Tick::Major(p.t3().to_f64(), AutoOption::Fix(std::string::String::from("t3"))),
            Tick::Major(p.t4().to_f64(), AutoOption::Fix(std::string::String::from("t4"))),
            Tick::Major(p.t5().to_f64(), AutoOption::Fix(std::string::String::from("t5"))),
            Tick::Major(p.t6().to_f64(), AutoOption::Fix(std::string::String::from("t6"))),
            Tick::Major(p.t7().to_f64(), AutoOption::Fix(std::string::String::from("t7"))),

        );

        let steps_per_unit = Real::from_lit(5, 0);

        let mut time: Vec<f64> = Vec::with_capacity(1000);
        let mut time_step_by_pulse: Vec<f64> = Vec::with_capacity(1000);
        let mut step_by_pulse: Vec<f64> = Vec::with_capacity(1000);
        let mut pos: Vec<f64> = Vec::with_capacity(1000);

        let mut spd: Vec<f64> = Vec::with_capacity(1000);
        let mut acc: Vec<f64> = Vec::with_capacity(1000);
        let mut jerk: Vec<f64> = Vec::with_capacity(1000);

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

        for (_v, ti, pos) in self.into_iter() {
            //info!("T[{}] {} P {}", _v, ti, pos);

            real_advanced = pos;

            let ti_v = ti.inner();
            let real_pos = pos.inner();
            let c_ti = ti_v.to_f64().unwrap();
            let c_pos = real_pos.to_f64().unwrap();

            let dt = c_ti.clone() - last_t;

            let pos_abs = real_pos.to_i32().unwrap() as u32;

            let real_steps = (pos * steps_per_unit).ceil();

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

            let s = (c_pos - last_p) / dt.clone();
            let a = ((s.clone() - last_s) / dt.clone());
            let j = ((a.clone() - last_a) / dt.clone());

            time_ref.push(c_ti.clone());
            pos_ref.push(c_pos.clone());

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
        println!("D; Advanced: {} mm {} steps", real_advanced.rdp(4), abs_steps_advanced);
        let mut fg = Figure::new();

        fg.set_multiplot_layout(5, 1)
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
            .lines(time.clone(), pos.clone(), &[PlotOption::Color("blue")])
            .set_x2_ticks_custom(t.clone(), &[], &[]);
        ;
        fg.axes2d()
            .set_y_label("Velocity (mm/s)", &[])
            .lines(time.clone(), spd.clone(), &[PlotOption::Color("green")])
        ;
        fg.axes2d()
            .set_y_label("Acceleration (mm/s²)", &[])
            .lines(time.clone(), acc.clone(), &[PlotOption::Color("red")])
        ;
        fg.axes2d()
            .set_y_label("Jerk (m/s³)", &[])
            .lines(time.clone(), jerk.clone(), &[PlotOption::Color("orange")])
        ;
        fg.axes2d()
            .set_x_label("Time in seconds", &[])
            .set_y_label("Discrete\nsteps", &[])
            .set_x2_ticks_custom(t, &[], &[])
            .set_x2_grid(true)
            .set_x2_minor_grid(true)
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
                Some((1, tp, self.plan.eval_position(tp)?))
            }
        }
    }
}


#[test]
pub fn plan_test() -> Result<(),()>{
    /*

    println!("3.9: Constant velocity phase is present");
    // Boundaries
    let q_1 = Real::from_fixed(dec!(10.0));
    let v_0 = Real::from_fixed(dec!(1.0));
    let v_1 = Real::from_fixed(dec!(0.0));

    // Constraints
    let v_max = Real::from_fixed(dec!(5.0));
    let a_max = Real::from_fixed(dec!(10.0));
    let j_max = Real::from_fixed(dec!(30.0));

    let boundaries = Boundaries { q_1, v_0, v_1, };
    let constraints = Constraints { v_max, a_max, j_max, };
    println!("boundaries: {}, constraints: {}", boundaries, constraints);
    let intervals = SCurveMotionProfile::new(&boundaries, &constraints);
    println!("expected  = T_{{j1}}=0.3333 T_{{a}}=0.7333 T_{{v}}=1.1433 T_{{d}}=0.8333 T_{{j2}}=0.3333");
    println!("intervals = {}", intervals);
    println!("--");

    println!("3.10: Constant velocity phase is NOT present");
    // Boundaries
    let q_1 = Real::from_fixed(dec!(10.0));
    let v_0 = Real::from_fixed(dec!(1.0));
    let v_1 = Real::from_fixed(dec!(0.0));

    // Constraints
    let v_max = Real::from_fixed(dec!(10.0));
    let a_max = Real::from_fixed(dec!(10.0));
    let j_max = Real::from_fixed(dec!(30.0));

    let boundaries = Boundaries { q_1, v_0, v_1, };
    let constraints = Constraints { v_max, a_max, j_max, };
    println!("boundaries: {}, constraints: {}", boundaries, constraints);
    let intervals = SCurveMotionProfile::new(&boundaries, &constraints);
    println!("expected  = T_{{j1}}=0.3333 T_{{a}}=1.0747 T_{{v}}=0.0 T_{{d}}=1.1747 T_{{j2}}=0.3333");
    println!("intervals = {}", intervals);
    println!("--");


    println!("3.11: Constant velocity phase is NOT present and maximum acceleration is not reached");
    // Boundaries
    let q_1 = Real::from_fixed(dec!(10.0));
    let v_0 = Real::from_fixed(dec!(7.0));
    let v_1 = Real::from_fixed(dec!(0.0));

    // Constraints
    let v_max = Real::from_fixed(dec!(10.0));
    let a_max = Real::from_fixed(dec!(10.0));
    let j_max = Real::from_fixed(dec!(30.0));

    let boundaries = Boundaries { q_1, v_0, v_1, };
    let constraints = Constraints { v_max, a_max, j_max, };
    println!("boundaries: {}, constraints: {}", boundaries, constraints);
    let intervals = SCurveMotionProfile::new(&boundaries, &constraints);
    println!("expected  = T_{{j1}}=0.2321 T_{{a}}=0.4666 T_{{v}}=0.0 T_{{d}}=1.4718 T_{{j2}}=0.2321");
    println!("intervals = {}", intervals);
    println!("--");

    println!("3.12: Constant velocity phase is NOT present, maximum acceleration is not reached and T_{{a}} becomes negative");

    // Boundaries
    let q_1 = Real::from_fixed(dec!(10.0));
    let v_0 = Real::from_fixed(dec!(7.5));
    let v_1 = Real::from_fixed(dec!(0.0));

    // Constraints
    let v_max = Real::from_fixed(dec!(10.0));
    let a_max = Real::from_fixed(dec!(10.0));
    let j_max = Real::from_fixed(dec!(30.0));

    let boundaries = Boundaries { q_1, v_0, v_1, };
    let constraints = Constraints { v_max, a_max, j_max, };
    println!("boundaries: {}, constraints: {}", boundaries, constraints);
    let intervals = SCurveMotionProfile::new(&boundaries, &constraints);
    // TBD
    println!("expected  = T_{{j1}}=0.0 T_{{a}}=0.0 T_{{v}}=0.0 T_{{d}}=2.6667 T_{{j2}}=0.0973");
    println!("intervals = {}", intervals);
    println!("--");

    let mut profile = PlanProfile::new(intervals,Real::from_fixed(dec!(0.01)));
    profile.plot();

    //////////////

    println!("3.12 bis: Constant velocity phase is NOT present, maximum acceleration is not reached and T_{{a}} becomes negative");

    // Boundaries
    let q_1 = Real::from_fixed(dec!(10.0));
    let v_0 = Real::from_fixed(dec!(0.0));
    let v_1 = Real::from_fixed(dec!(7.5));

    // Constraints
    let v_max = Real::from_fixed(dec!(10.0));
    let a_max = Real::from_fixed(dec!(10.0));
    let j_max = Real::from_fixed(dec!(30.0));

    let boundaries = Boundaries { q_1, v_0, v_1, };
    let constraints = Constraints { v_max, a_max, j_max, };
    println!("boundaries: {}, constraints: {}", boundaries, constraints);
    let intervals = SCurveMotionProfile::new(&boundaries, &constraints);
    // TBD
    println!("expected  = T_{{j1}}=0.0 T_{{a}}=0.0 T_{{v}}=0.0 T_{{d}}=2.6667 T_{{j2}}=0.0973");
    println!("intervals = {}", intervals);
    println!("--");

    let mut profile = PlanProfile::new(intervals,
                                       Real::from_fixed(dec!(0.01)));
    profile.plot();
    */

    Ok(())
}




