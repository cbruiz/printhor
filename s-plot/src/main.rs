#![allow(unused)]

mod prelude;

use num_traits::float::FloatCore;
use num_traits::ToPrimitive;
use prelude::*;

use crate::tgeo::{TVector, CoordSel};
use crate::control::motion_planning::*;
use crate::hwa::controllers::motion::*;
use crate::prelude::math::{RealInclusiveRange, TWO, ZERO};

fn main() {

    env_logger::init();

    let sampling_period = Real::from_f32(0.001);

    let dts = Real::from_f32(300.0); // default_travel_speed
    let flow_rate = Real::one();
    let speed_rate = Real::one();
    let max_speed = Real::from_f32(3000.0);
    let max_accel = Real::from_f32(2000.0);
    let max_jerk = Real::from_f32(1000.0);

    let requested_motion_speed = Some(Real::from_f32(300.0));
    let v0 = Real::from_f32(126.0);
    let v1 = Real::from_f32(126.0);

    let p0: TVector<Real> = TVector::zero();
    let p1: TVector<Real> = TVector::from_coords(
        Some(Real::from_f32(0.5)),
        None,
        None,
        None,
    );

    hwa::info!("----");
    hwa::info!("P0 ({})", p0);
    hwa::info!("P1 ({})", p1);
    hwa::info!("--");

    // Compute distance and decompose as unit vector and module.
    // When dist is zero, value is map to None (NaN).
    // In case o E dimension, flow rate factor is applied
    let (vdir, module_target_distance) = (p1 - p0)
        .map_coord(CoordSel::all(), |coord_value, coord_idx| {
            match coord_idx {
                CoordSel::X | CoordSel::Y | CoordSel::Z => {
                    match coord_value.is_zero() {
                        true => None,
                        false => Some(coord_value),
                    }
                },
                _ => None,
            }
        }).decompose_normal();

    // Compute the speed module applying speed_rate factor
    let speed_module = requested_motion_speed.unwrap_or(dts) * speed_rate;
    // Compute per-axis target speed
    let speed_vector: TVector<Real> = vdir.with_coord(CoordSel::E,
        p1.e.and_then(|v| {
            match v.is_zero() {
                true => None,
                false => Some(v * flow_rate),
            }

        })
    ).abs() * speed_module;
    // Clamp per-axis target speed to the physical restrictions
    let clamped_speed = speed_vector.clamp(TVector::from_coords(Some(max_speed), Some(max_speed), Some(max_speed), Some(max_speed)));
    // Finally, per-axis relative speed

    let v = vdir * module_target_distance;

    hwa::info!("v: {}", v);
    hwa::info!("speed_vector: {}", speed_vector);
    hwa::info!("speed_vector / v: {}", speed_vector / v);
    hwa::info!("max_speed: {}", max_speed);
    hwa::info!("max_speed_v: {}", speed_vector * vdir);
    hwa::info!("speed_vector / max_speed: {}", speed_vector / max_speed);
    hwa::info!("clamped_speed: {}", clamped_speed);

    let module_target_speed = clamped_speed.vmin().unwrap();

    if !module_target_distance.is_defined_positive() {
        match !module_target_speed.is_zero() && !max_accel.is_zero() && !max_jerk.is_zero() {
            true => {
                execute(v0, v1, module_target_speed, vdir,
                        module_target_distance,
                        max_accel, max_jerk,
                        sampling_period
                );
            },
            false => {
                hwa::error!("p0: {}", p0.rdp(4));
                hwa::error!("p1: {}", p1.rdp(4));
                hwa::error!("dist: {}", module_target_distance.rdp(4));
                hwa::error!("vdir: {}", vdir.rdp(4));
                hwa::error!("speed_vector: {}", speed_vector.rdp(4));
                hwa::error!("clamped_speed: {}", clamped_speed.rdp(4));
                hwa::error!("clamped_speed: {}", clamped_speed.rdp(4));
            }
        };
    }
}


fn execute(v0: Real, v1: Real, module_target_speed: Real,
           vdir: TVector<Real>, module_target_distance: Real,
           max_accel: Real, max_jerk: Real, sampling_period: Real,
) -> Result<(), ()> {
    let _constraints = Constraints {
        v_max: module_target_speed,
        a_max: max_accel,
        j_max: max_jerk,
    };
    let mut segment = Segment::new(
        SegmentData {
            speed_enter_mms: v0,
            speed_exit_mms: v1,
            speed_target_mms: Default::default(),
            displacement_mm: module_target_distance,
            speed_enter_constrained_mms: Default::default(),
            speed_exit_constrained_mms: Default::default(),
            proj_prev: Default::default(),
            proj_next: Default::default(),
            vdir,
            dest_pos: Default::default(),
            tool_power: Real::zero(),
            constraints: _constraints,
        },
    );

    let mut motion_profile = SCurveMotionProfile::compute(
        module_target_distance, v0, v1,
        &_constraints, true).map_err(|_| ())?;

    hwa::info!("real dist: {}", module_target_distance.rdp(4));
    hwa::info!("speed: {}", module_target_speed.rdp(4));

    hwa::info!("--");
    hwa::info!("Profile: {}", motion_profile);

    let mut profile = PlanProfile::new(motion_profile, sampling_period);

    profile.plot(v0, true, true, true, true);
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

    pub fn plot(&mut self, initial_speed: Real, plot_pos: bool, plot_vel: bool, plot_accel: bool, plot_jerk: bool) {

        use gnuplot::{AxesCommon, Figure};
        use gnuplot::{AutoOption, Tick, MultiplotFillOrder::RowsFirst, MultiplotFillDirection::{Downwards}};
        use gnuplot::{DashType, PlotOption};

        let p = &(self.plan);

        let steps_per_unit = Real::from_lit(16, 0);

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
        let mut last_s = initial_speed.to_f64();
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

                    dt_acc += dt_step;
                    time_step_by_pulse.push(dt_acc);
                    step_by_pulse.push(abs_steps_advanced as f64);
                }
            }
            else {
                time_step_by_pulse.push(last_t);
                step_by_pulse.push(abs_steps_advanced as f64);
                time_step_by_pulse.push(ti_v.to_f64().unwrap());
                step_by_pulse.push(abs_steps_advanced as f64);
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
            last_s = if s.is_nan() {initial_speed.to_f64()} else {s};
            last_a = if a.is_nan() {0.} else {a};
            last_j = if j.is_nan() {0.} else {j};
        }
        time_step_by_pulse.push(last_t);
        step_by_pulse.push(abs_steps_advanced as f64);
        println!("D; Advanced: est: {} mm, real: {:.03} mm ({} steps)", real_advanced.rdp(4), (abs_steps_advanced.to_f64().unwrap() / steps_per_unit.to_f64()), abs_steps_advanced);
        let mut fg = Figure::new();

        cfg_if::cfg_if! {
            if #[cfg(feature="float-point-f32-impl")] {
                const MATH_PRECISION: &str = "[float point 32bits]";
            }
            else if #[cfg(feature="float-point-f64-impl")] {
                const MATH_PRECISION: &str = "[float point 64bits]";
            }
            else if #[cfg(feature="fixed-point-128-impl")] {
                const MATH_PRECISION: &str = "[fixed point 128bits]";
            }
            else {
                const MATH_PRECISION: &str = "[N/A]";
            }
        }

        fg.set_multiplot_layout(6, 1)
            .set_title(
                format!(
                    "{} Double S-Curve velocity profile\n[vin={} mm/s, vlim={} mm/s, vout={} mm/s, displacement={} mm, sample\\_period={} s]",
                    MATH_PRECISION,
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