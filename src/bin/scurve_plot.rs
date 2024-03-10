#![allow(unused)]
#[path = "printhor/math/mod.rs"]
mod math;
use math::Real;
#[path = "printhor/hwa/mod.rs"]
mod hwa;
#[path = "printhor/hwi/mod.rs"]
mod hwi;
#[path = "printhor/sync/mod.rs"]
mod sync;
#[path = "printhor/tgeo.rs"]
mod tgeo;
use tgeo::*;
extern crate alloc;
#[path = "printhor/control/mod.rs"]
mod control;
#[path = "printhor/helpers/mod.rs"]
mod helpers;
#[path = "printhor/machine.rs"]
mod machine;
use crate::hwa::controllers::motion::motion_segment::{Segment, SegmentData};
use crate::control::motion_planning::{Constraints, PlanProfile, SCurveMotionProfile};
#[allow(unused)]
use crate::math::{ONE, ZERO};
#[allow(unused)]
fn initialization_error() {

}

fn main() {

    hwa::init_logger();

    let dts = Real::from_f32(100.0); // default_travel_speed
    let flow_rate = Real::one();
    let speed_rate = Real::one();
    let max_speed = Real::from_f32(300.0);
    let max_accel = Real::from_f32(600.0);
    let max_jerk = Real::from_f32(1200.0);

    let requested_motion_speed = Some(Real::from_f32(100.0f32));

    let p0: TVector<Real> = TVector::zero();
    let p1: TVector<Real> = TVector::from_coords(
        Some(Real::from_f32(10.0)),
        None,
        None,
        None,
    );

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

    let module_target_speed = clamped_speed.min().unwrap();

    if !module_target_distance.is_zero() {
        match !module_target_speed.is_zero() && !max_accel.is_zero() && !max_jerk.is_zero() {
            true => {
                let t2 = embassy_time::Instant::now();
                let _constraints = Constraints {
                    v_max: module_target_speed,
                    a_max: max_accel,
                    j_max: max_jerk,
                };
                let segment = Segment::new(
                    SegmentData {
                        speed_enter_mms: 0,
                        speed_exit_mms: 0,
                        displacement_u: (module_target_distance / Real::from_lit(1000, 0)).to_i32().unwrap_or(0) as u32,
                        vdir,
                        dest_pos: Default::default(),
                        tool_power: 0,
                    },
                    SCurveMotionProfile::compute(
                        module_target_distance, ZERO, ZERO,
                        &_constraints).unwrap()
                );
                let t3 = embassy_time::Instant::now();

                hwa::info!("----");
                hwa::info!("P0 ({})", p0);
                hwa::info!("P1 ({})", p1);
                hwa::info!("--");
                hwa::info!("dist: {}", module_target_distance.rdp(4));
                hwa::info!("speed: {}", module_target_speed.rdp(4));
                hwa::info!("accel: {}", max_accel);
                hwa::info!("jerk: {}", max_jerk);
                hwa::info!("--");
                hwa::info!("plan computed in : {} us", (t3-t2).as_micros());
                hwa::info!("Profile: {}", segment.motion_profile);

                PlanProfile::new(segment.motion_profile, Real::from_f32(0.01))
                    .plot(true, true, true, true);
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
