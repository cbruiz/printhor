//! TODO: This feature is still in incubation
use crate::math::Real;
use crate::control::motion_planning::{Constraints, MotionProfile};
use crate::tgeo::TVector;

#[derive(Clone, Copy)]
pub struct SegmentData {
    /// Enter speed in mm/s
    pub speed_enter_mms: Real,
    /// Exit speed in mm/s
    pub speed_exit_mms: Real,
    /// Target speed in mm/s
    pub speed_target_mms: Real,
    /// Total displacement to complete the movement in millimeters
    pub displacement_mm: Real,

    pub speed_enter_constrained_mms: Real,
    //pub speed_max_gain_mms: Real,
    pub speed_exit_constrained_mms: Real,
    pub proj_prev: Real,
    pub proj_next: Real,

    pub vdir: TVector<Real>,
    pub dest_pos: TVector<Real>,

    pub tool_power: Real,
    pub constraints: Constraints,
}

#[derive(Clone, Copy)]
pub struct Segment {
    pub segment_data: SegmentData,
    #[cfg(feature = "native")]
    pub id: u32,
}

impl Segment {
    pub fn new(segment_data: SegmentData) -> Self {
        cfg_if::cfg_if! {
            if #[cfg(feature="native")] {
                static mut COUNTER: u32 = 0;
                let id = unsafe {
                    COUNTER += 1;
                    COUNTER
                };
                Self {segment_data, id}
            }
            else {
                Self {segment_data}
            }
        }

    }
}

pub struct SegmentIterator<'a, P> {
    profile: &'a P,
    ref_time: Real,
    exhausted: bool,
}

impl<'a, P> SegmentIterator<'a, P>
where P: MotionProfile
{
    pub const fn new(profile: &'a P, ref_time: Real) -> Self {
        SegmentIterator {
            profile,
            ref_time,
            exhausted: false,
        }
    }

    pub fn next(&mut self, now: Real) -> Option<Real> {
        if self.exhausted {
            None
        }
        else {
            let relative_time = now - self.ref_time;
            if relative_time > self.profile.end() {
                self.exhausted = true;
            }
            self.profile.eval_position(relative_time)
        }
    }
}
