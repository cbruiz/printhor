//! TODO: This feature is still in incubation
use crate::control::motion::{Constraints, MotionProfile};
use crate::hwa;
use crate::math::Real;
use crate::tgeo::TVector;

/// Represents the data for a motion segment.
///
/// # Fields
/// - `speed_enter_mms`: Initial speed at the entry of the segment in millimeters per second (mm/s).
/// - `speed_exit_mms`: Speed at the exit of the segment in millimeters per second (mm/s).
/// - `speed_target_mms`: Target speed for the segment in millimeters per second (mm/s).
/// - `displacement_mm`: Total displacement to complete the movement in millimeters (mm).
/// - `speed_enter_constrained_mms`: Constrained initial speed at the entry of the segment in millimeters per second (mm/s).
/// - `speed_exit_constrained_mms`: Constrained speed at the exit of the segment in millimeters per second (mm/s).
/// - `proj_prev`: Projection of the previous segment.
/// - `proj_next`: Projection of the next segment.
/// - `unit_vector_dir`: Unit vector for the direction of movement.
/// - `dest_pos`: Destination position vector.
/// - `tool_power`: Tool power utilized in the segment.
/// - `constraints`: Motion constraints applicable to the segment.
#[derive(Clone, Copy)]
pub struct SegmentData {
    /// Initial speed at the entry of the segment in millimeters per second (mm/s).
    pub speed_enter_mms: Real,
    /// Speed at the exit of the segment in millimeters per second (mm/s).
    pub speed_exit_mms: Real,
    /// Target speed for the segment in millimeters per second (mm/s).
    pub speed_target_mms: Real,
    /// Total displacement to complete the movement in millimeters (mm).
    pub displacement_mm: Real,

    /// Constrained initial speed at the entry of the segment in millimeters per second (mm/s).
    pub speed_enter_constrained_mms: Real,
    /// Constrained speed at the exit of the segment in millimeters per second (mm/s).
    pub speed_exit_constrained_mms: Real,
    /// Projection of the previous segment.
    pub proj_prev: Real,
    /// Projection of the next segment.
    pub proj_next: Real,

    /// Unit vector for the direction of movement.
    pub unit_vector_dir: TVector<Real>,
    /// Destination position vector in millimeters.
    pub dest_pos: TVector<Real>,

    #[allow(unused)]
    /// Tool power utilized in the segment.
    pub tool_power: Real,
    /// Motion constraints applicable to the segment.
    pub constraints: Constraints,
}

/// Represents a motion segment.
///
/// # Fields
/// - `segment_data`: Data for the segment, encapsulated in [SegmentData].
/// - `id`: Segment ID, available only when the "native" feature is enabled.
#[derive(Clone, Copy)]
pub struct Segment {
    pub segment_data: SegmentData,
    #[cfg(feature = "native")]
    pub id: u32,
}

impl Segment {
    /// Creates a new Segment with the given data.
    ///
    /// # Parameters
    /// - `segment_data`: Data for the segment.
    ///
    /// # Returns
    /// A new instance of `Segment`.
    pub fn new(segment_data: SegmentData) -> Self {
        cfg_if::cfg_if! {
            if #[cfg(feature="native")] {
                static mut COUNTER: u32 = 0;
                // TODO: remove unsafe
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

/// Iterator over motion segments.
///
/// # Type Parameters
/// - `'a`: Lifetime of the motion profile reference.
/// - `P`: Type of the motion profile, which must implement the [MotionProfile] trait.
pub struct SegmentIterator<'a, P>
where
    P: MotionProfile,
{
    /// Reference to the motion profile.
    profile: &'a P,
    /// Reference time for the iterator.
    ref_time: Real,
    /// State flag indicating whether the iterator is exhausted.
    exhausted: bool,
}

impl<'a, P> SegmentIterator<'a, P>
where
    P: MotionProfile,
{
    /// Creates a new SegmentIterator.
    ///
    /// # Parameters
    /// - `profile`: Reference to the motion profile.
    /// - `ref_time`: Reference time for the iterator.
    ///
    /// # Returns
    /// A new instance of `SegmentIterator`.
    pub const fn new(profile: &'a P, ref_time: Real) -> Self {
        SegmentIterator {
            profile,
            ref_time,
            exhausted: false,
        }
    }

    /// Advances the iterator and returns the next segment.
    ///
    /// # Parameters
    /// - `now`: Current time.
    ///
    /// # Returns
    /// An `Option` containing the next segment's position and a u8 value, or `None` if exhausted.
    pub fn next(&mut self, now: Real) -> Option<(Real, u8)> {
        if self.exhausted {
            None
        } else {
            let relative_time = now - self.ref_time;
            if relative_time > self.profile.end_time() {
                self.exhausted = true;
            }
            match self.profile.eval_position(relative_time) {
                None => None,
                Some(p) => {
                    let end_pos = self.profile.end_pos();

                    // FIXME: Do it in a better way. Use half step length as margin
                    if p.0 + Real::from_f32(0.00001f32) >= end_pos {
                        self.exhausted = true;
                        hwa::trace!(
                            "pos exhausted at t={} / {}",
                            relative_time,
                            self.profile.end_time()
                        );
                        Some((end_pos, p.1))
                    } else {
                        Some((p.0, p.1))
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::control::motion::SCurveMotionProfile;
    use crate::math;
    use super::*;
    use crate::math::Real;
    use crate::tgeo::TVector;

    fn dummy_segment() -> SegmentData {
        SegmentData {
            speed_enter_mms: Real::from_f32(10.0),
            speed_exit_mms: Real::from_f32(15.0),
            speed_target_mms: Real::from_f32(20.0),
            displacement_mm: Real::from_f32(100.0),
            speed_enter_constrained_mms: Real::from_f32(10.0),
            speed_exit_constrained_mms: Real::from_f32(15.0),
            proj_prev: Real::from_f32(0.0),
            proj_next: Real::from_f32(100.0),
            unit_vector_dir: TVector::one(),
            dest_pos: TVector::one() * math::ONE_HUNDRED,
            tool_power: Real::from_f32(5.0),
            constraints: Constraints {
                v_max: math::ONE_HUNDRED,
                a_max: math::ONE_THOUSAND,
                j_max: math::ONE_THOUSAND,
            }
        }
    }

    #[test]
    fn test_segment_creation() {

        let segment = Segment::new(dummy_segment());
        assert_eq!(segment.segment_data.speed_enter_mms, Real::from_f32(10.0));
        assert_eq!(segment.segment_data.speed_exit_mms, Real::from_f32(15.0));
        assert_eq!(segment.segment_data.speed_target_mms, Real::from_f32(20.0));
    }

    #[test]
    #[cfg(feature = "native")]
    fn test_segment_creation_with_id() {

        let segment = Segment::new(dummy_segment());
        assert_eq!(segment.segment_data.speed_enter_mms, Real::from_f32(10.0));
        assert_eq!(segment.segment_data.speed_exit_mms, Real::from_f32(15.0));
        assert_eq!(segment.segment_data.speed_target_mms, Real::from_f32(20.0));
        // Check if the ID is incremented correctly
        assert!(segment.id > 0);
    }

    #[test]
    fn test_segment_iterator() {
        let ref_time = Real::from_f32(0.0);

        let constraints = Constraints {
            v_max: Real::from_f32(10.0),
            a_max: Real::from_f32(2.0),
            j_max: Real::from_f32(1.0)
        };
        let motion_profile = SCurveMotionProfile::compute(
            Real::from_f32(20.0),
            Real::from_f32(0.0),
            Real::from_f32(5.0),
            &constraints, true).unwrap();

        let mut segment_iter = SegmentIterator::new(&motion_profile, ref_time);

        // Test iterator before exhaustion
        let now = Real::from_f32(5.0);
        let segment = segment_iter.next(now);
        assert!(segment.is_some());

        // Exhaust the iterator
        segment_iter.exhausted = true;
        let segment = segment_iter.next(now);
        assert!(segment.is_none());
    }

    /// Ensure that the iterator reaches end position oly once if micro-segment advance
    /// reaches maximum time
    #[test]
    fn test_segment_iterator_exhaustion() {
        let constraints = Constraints {
            v_max: Real::from_f32(10.0),
            a_max: Real::from_f32(2.0),
            j_max: Real::from_f32(1.0)
        };
        let motion_profile = SCurveMotionProfile::compute(
            Real::from_f32(20.0),
            Real::from_f32(0.0),
            Real::from_f32(5.0),
            &constraints, true).unwrap();
        let ref_time = Real::from_f32(0.0);
        let mut segment_iter = SegmentIterator::new(&motion_profile, ref_time);

        // Set to a time past the end of the profile
        let now = Real::from_f32(100.0);
        let micro_segment = segment_iter.next(now);

        assert_eq!(micro_segment.unwrap().0, motion_profile.end_pos(), "At end");
        let micro_segment = segment_iter.next(now);
        assert!(micro_segment.is_none(), "Does not avance more");
        assert!(segment_iter.exhausted, "Is exhausted");
    }
}