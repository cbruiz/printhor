//! TODO: This feature is still very experimental
use crate::math::Real;
use crate::planner::SCurveMotionProfile;
use crate::tgeo::TVector;

#[allow(unused)]
#[derive(Clone, Copy)]
pub struct SegmentData {
    /// Enter speed in steps per sec
    pub speed_enter_sps: u32,
    /// Exit speed in steps per sec
    pub speed_exit_sps: u32,
    /// Total steps to complete the movement
    pub total_steps: u32,

    pub vdir: TVector<Real>,
    pub dest_pos: TVector<Real>,
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
}