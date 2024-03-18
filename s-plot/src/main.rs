#![allow(unused)]

mod prelude;
use prelude::*;

use crate::tgeo::{TVector, CoordSel};
use crate::control::motion_planning::*;
use crate::hwa::controllers::motion::*;
use crate::prelude::math::TWO;

fn main() {

    env_logger::init();

    let sampling_period = Real::from_f32(0.01);

    let dts = Real::from_f32(300.0); // default_travel_speed
    let flow_rate = Real::one();
    let speed_rate = Real::one();
    let max_speed = Real::from_f32(3000.0);
    let max_accel = Real::from_f32(9000.0);
    let max_jerk = Real::from_f32(12000.0);

    let requested_motion_speed = Some(Real::from_f32(300.0));
    let v0 = Real::from_f32(0.0);
    let v1 = Real::from_f32(0.0);

    let p0: TVector<Real> = TVector::zero();
    let p1: TVector<Real> = TVector::from_coords(
        Some(Real::from_f32(100.0)),
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

    let module_target_speed = clamped_speed.min().unwrap();

    if !module_target_distance.is_zero() {
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
) {
    let t2 = embassy_time::Instant::now();
    let _constraints = Constraints {
        v_max: module_target_speed,
        a_max: max_accel,
        j_max: max_jerk,
    };
    let mut segment = Segment::new(
        SegmentData {
            speed_enter_mms: v0,
            speed_exit_mms: v1,
            displacement_mm: module_target_distance,
            vdir,
            dest_pos: Default::default(),
            tool_power: Real::zero(),
        },
        SCurveMotionProfile::compute(
            module_target_distance, v0, v1,
            &_constraints, false).unwrap()
    );
    let t3 = embassy_time::Instant::now();

    hwa::info!("real dist: {}", module_target_distance.rdp(4));
    hwa::info!("speed: {}", module_target_speed.rdp(4));

    let final_pos = segment.motion_profile.s_i7(&segment.motion_profile.i7_end());
    let delta_e = module_target_distance - final_pos;
    hwa::info!("practical dist(2): {}", final_pos);
    hwa::info!("practical dist(2): {}", segment.motion_profile.q1);
    hwa::info!("\\delta_{{e}}: {}", delta_e);

    hwa::info!("--");
    hwa::info!("plan computed in : {} us", (t3-t2).as_micros());
    hwa::info!("Profile: {}", segment.motion_profile);
    let _ = segment.motion_profile.extend(delta_e);
    hwa::info!("Refined profile: {}", segment.motion_profile);

    let mut profile = PlanProfile::new(segment.motion_profile, sampling_period);

    profile.plot(true, true, true, true);
}
