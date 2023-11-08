//! TODO: This feature is still very experimental
use crate::math::Real;
use crate::planner::SCurveMotionProfile;
use crate::tgeo::TVector;

#[allow(unused)]
#[derive(Clone, Copy)]
pub struct SegmentData {
    /// Enter speed in mm/s
    pub speed_enter_mms: u32,
    /// Exit speed in mm/s
    pub speed_exit_mms: u32,
    /// Total displacement to complete the movement in micrometers
    pub displacement_u: u32,

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