//! Motion segment module
//! TODO: This feature is still in incubation
use crate::control::motion::{Constraints, MotionProfile};
use crate::hwa;
use hwa::math::Real;
use hwa::math::TVector;

/// Represents the data for a motion segment.
///
/// # Fields
/// - `speed_enter_sus`: Initial speed at the entry of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `speed_exit_sus`: Speed at the exit of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `speed_target_sus`: Target speed for the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `displacement_su`: Total displacement to complete the movement in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]).
/// - `speed_enter_constrained_sus`: Constrained initial speed at the entry of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `speed_exit_constrained_sus`: Constrained speed at the exit of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `proj_prev`: Projection of the previous segment.
/// - `proj_next`: Projection of the next segment.
/// - `unit_vector_dir`: Unit vector for the direction of movement in space coordinates.
/// - `dest_pos`: Destination position vector in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]).
/// - `dest_world_pos`: Destination position vector in world units ([hwa::Contract::WORLD_UNIT_MAGNITUDE]).
/// - `tool_power`: Tool power utilized in the segment.
/// - `constraints`: Motion constraints applicable to the segment.
#[derive(Clone, Copy)]
pub struct Segment {
    /// Initial speed at the entry of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_enter_su_s: Real,
    /// Speed at the exit of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_exit_su_s: Real,
    /// Target speed for the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_target_su_s: Real,
    /// Total displacement to complete the movement in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]).
    pub displacement_su: Real,

    /// Constrained initial speed at the entry of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_enter_constrained_su_s: Real,
    /// Constrained speed at the exit of the segment in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_exit_constrained_su_s: Real,
    /// Projection of the previous segment.
    pub proj_prev: Real,
    /// Projection of the next segment.
    pub proj_next: Real,

    /// Unit vector for the direction of movement.
    pub unit_vector_dir: TVector<Real>,

    /// Source position vector in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]).
    pub src_pos: TVector<Real>,

    /// Destination position vector in space units ([hwa::Contract::SPACE_UNIT_MAGNITUDE]).
    pub dest_pos: TVector<Real>,

    /// Destination position vector in world units ([hwa::Contract::WORLD_UNIT_MAGNITUDE]).
    pub dest_world_pos: TVector<Real>,

    #[allow(unused)]
    /// Tool power utilized in the segment.
    pub tool_power: Real,
    /// Motion constraints applicable to the segment.
    pub constraints: Constraints, // Should remove this?
}

impl Segment {
    pub fn fix_deviation(&mut self, deviation: &TVector<Real>, _flow_rate: Real) {
        self.src_pos -= *deviation;
        (self.unit_vector_dir, self.displacement_su) = (self.dest_pos - self.src_pos)
            .map_values(|coord, coord_value| match coord {
                #[cfg(feature = "with-e-axis")]
                hwa::CoordSel::E => match coord_value.is_zero() {
                    true => None,
                    false => Some(coord_value * _flow_rate),
                },
                _ => match coord_value.is_zero() {
                    true => None,
                    false => Some(coord_value),
                },
            })
            .decompose_normal();
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
    /// The time step in secs to increment in each iteration.
    sampling_period_s: Real,
    /// Last evaluation position in world units.
    last_evaluated_position_wu: Real,
    /// Last evaluation time instant.
    last_evaluated_time_s: Real,
    ds: Real,
    dt: Real,
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
    /// - `sampling_period`: The sampling period in seconds.
    ///
    /// # Returns
    /// A new instance of `SegmentIterator`.
    pub const fn new(profile: &'a P, sampling_period_s: Real) -> Self {
        SegmentIterator {
            profile,
            sampling_period_s,
            last_evaluated_position_wu: Real::zero(),
            last_evaluated_time_s: Real::zero(),
            ds: Real::zero(),
            dt: Real::zero(),
            exhausted: false,
        }
    }

    pub fn current_position(&self) -> Real {
        self.last_evaluated_position_wu
    }

    pub fn current_time(&self) -> Real {
        self.last_evaluated_time_s
    }

    pub fn ds(&self) -> Real {
        self.ds
    }

    pub fn dt(&self) -> Real {
        self.dt
    }

    pub fn speed(&self) -> Real {
        if self.dt.is_negligible() {
            Real::zero()
        } else {
            self.ds / self.dt
        }
    }

    /// Advances the iterator and returns the next micro-segment position.
    ///
    /// # Returns
    /// An `Option` containing:
    /// - The position as real
    /// - `None` if exhausted.
    pub fn next(&mut self) -> Option<Real> {
        if self.exhausted {
            None
        } else {
            let now = self.last_evaluated_time_s + self.sampling_period_s;
            if now >= self.profile.end_time() {
                self.exhausted = true;
                self.dt = self.profile.end_time() - self.last_evaluated_time_s;
                self.last_evaluated_time_s = self.profile.end_time();
            } else {
                self.dt = now - self.last_evaluated_time_s;
                self.last_evaluated_time_s = now;
            }
            match self.profile.eval_position(self.last_evaluated_time_s) {
                None => None,
                Some(p) => {
                    let end_pos = self.profile.end_pos();

                    if p >= end_pos {
                        self.exhausted = true;
                        self.ds = end_pos - self.last_evaluated_position_wu;
                        self.last_evaluated_position_wu = end_pos;
                    } else {
                        self.ds = p - self.last_evaluated_position_wu;
                        self.last_evaluated_position_wu = p;
                    }
                    Some(self.last_evaluated_position_wu)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::motion::SCurveMotionProfile;
    use crate::math;
    use crate::math::Real;
    use crate::math::TVector;

    fn dummy_segment() -> Segment {
        Segment {
            speed_enter_su_s: Real::from_f32(10.0),
            speed_exit_su_s: Real::from_f32(15.0),
            speed_target_su_s: Real::from_f32(20.0),
            displacement_su: Real::from_f32(100.0),
            speed_enter_constrained_su_s: Real::from_f32(10.0),
            speed_exit_constrained_su_s: Real::from_f32(15.0),
            proj_prev: Real::from_f32(0.0),
            proj_next: Real::from_f32(100.0),
            unit_vector_dir: TVector::one(),
            src_pos: TVector::zero(),
            dest_pos: TVector::one() * math::ONE_HUNDRED,
            dest_world_pos: TVector::one() * math::ONE_HUNDRED,
            tool_power: Real::from_f32(5.0),
            constraints: Constraints {
                v_max: math::ONE_HUNDRED,
                a_max: math::ONE_THOUSAND,
                j_max: math::ONE_THOUSAND,
            },
        }
    }

    #[test]
    fn test_segment_creation() {
        let segment = dummy_segment();
        assert_eq!(segment.speed_enter_su_s, Real::from_f32(10.0));
        assert_eq!(segment.speed_exit_su_s, Real::from_f32(15.0));
        assert_eq!(segment.speed_target_su_s, Real::from_f32(20.0));
    }

    #[test]
    fn test_segment_iterator() {
        let constraints = Constraints {
            v_max: Real::from_f32(10.0),
            a_max: Real::from_f32(2.0),
            j_max: Real::from_f32(1.0),
        };
        let motion_profile = SCurveMotionProfile::compute(
            Real::from_f32(20.0),
            Real::from_f32(0.0),
            Real::from_f32(5.0),
            &constraints,
            true,
        )
        .unwrap();
        // Set sampling time to 60% of profile time
        let sampling_period = motion_profile.end_time() * Real::from_f32(0.6);
        let mut segment_iter = SegmentIterator::new(&motion_profile, sampling_period);

        // Test iterator before exhaustion (expected to be at 60%)
        let segment = segment_iter.next();
        assert!(segment.is_some());

        // Test iterator at exhaustion (expected to be at 120%), truncated to motion_profile.end_time() and exhausted
        let segment = segment_iter.next();
        assert!(segment.is_some());

        // Test iterator after exhaustion, expected to return None
        let segment = segment_iter.next();
        assert!(segment.is_none());
    }

    /// Ensure that the iterator reaches end position oly once if micro-segment advance
    /// reaches maximum time
    #[test]
    fn test_segment_iterator_exhaustion() {
        let constraints = Constraints {
            v_max: Real::from_f32(10.0),
            a_max: Real::from_f32(2.0),
            j_max: Real::from_f32(1.0),
        };
        let motion_profile = SCurveMotionProfile::compute(
            Real::from_f32(20.0),
            Real::from_f32(0.0),
            Real::from_f32(5.0),
            &constraints,
            true,
        )
        .unwrap();
        let sampling_period = Real::from_f32(100.0);
        let mut segment_iter = SegmentIterator::new(&motion_profile, sampling_period);

        // Set to a time past the end of the profile
        let micro_segment = segment_iter.next();

        assert_eq!(micro_segment.unwrap(), motion_profile.end_pos(), "At end");
        let micro_segment = segment_iter.next();
        assert!(micro_segment.is_none(), "Does not avance more");
        assert!(segment_iter.exhausted, "Is exhausted");
    }
}
