//! TODO: This feature is still in incubation
use embassy_time::{Duration, Instant};
use crate::math::Real;
use crate::control::motion_planning::{MotionProfile, SCurveMotionProfile};
use crate::tgeo::TVector;

#[allow(unused)]
#[derive(Clone, Copy)]
pub struct SegmentData {
    /// Enter speed in mm/s
    pub speed_enter_mms: Real,
    /// Exit speed in mm/s
    pub speed_exit_mms: Real,
    /// Total displacement to complete the movement in millimeters
    pub displacement_mm: Real,

    pub vdir: TVector<Real>,
    pub dest_pos: TVector<Real>,

    pub tool_power: Real,
}

#[allow(unused)]
#[derive(Clone, Copy)]
pub struct Segment {
    pub segment_data: SegmentData,
    pub motion_profile: SCurveMotionProfile,
}

impl Segment {
    pub const fn new(segment_data: SegmentData, motion_profile: SCurveMotionProfile) -> Self {
        Self {segment_data, motion_profile}
    }

    pub fn recalculate(&self, v_0: Real, v_1: Real, error_correction: bool) -> Self {
        Self {
            segment_data: SegmentData {
                speed_enter_mms: v_0,
                speed_exit_mms: v_1,
                displacement_mm: self.segment_data.displacement_mm,
                vdir: self.segment_data.vdir,
                dest_pos: self.segment_data.dest_pos,
                tool_power: self.segment_data.tool_power,
            },
            motion_profile: SCurveMotionProfile::compute(self.motion_profile.q1, v_0, v_1, &self.motion_profile.constraints, error_correction).unwrap(),
        }
    }
}

pub struct SegmentIterator<'a, P> {
    profile: &'a P,
    ref_time: Instant,
    offset: u64,
    exhausted: bool,
}

impl<'a, P> SegmentIterator<'a, P>
where P: MotionProfile
{
    pub const fn new(profile: &'a P, ref_time: Instant, offset: Duration) -> Self {
        SegmentIterator {
            profile,
            ref_time,
            offset: offset.as_micros(),
            exhausted: false,
        }
    }

    pub fn next(&mut self, now: Instant) -> Option<Real> {
        if self.exhausted {
            None
        }
        else {
            let relative_instant = self.offset + now.duration_since(self.ref_time).as_micros();
            let relative_time = Real::from_lit(relative_instant as i64, 6);
            if relative_time > self.profile.end() {
                self.exhausted = true
            }
            //crate::hwa::trace!("t = {:0.4}", relative_time);
            self.profile.eval_position(relative_time)
        }
    }
}
