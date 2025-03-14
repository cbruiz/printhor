//! Motion segment module
use crate::hwa;
use hwa::math::Real;
use hwa::math::TVector;

/// Represents the data for a motion segment.
///
/// # Fields
/// - `speed_enter_sus`: Initial speed at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `speed_exit_sus`: Speed at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `speed_target_sus`: Target speed for the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `displacement_su`: Total displacement to complete the movement in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
/// - `speed_enter_constrained_sus`: Constrained initial speed at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `speed_exit_constrained_sus`: Constrained speed at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `proj_prev`: Projection of the previous segment.
/// - `proj_next`: Projection of the next segment.
/// - `unit_vector_dir`: Unit vector for the direction of movement in space coordinates.
/// - `dest_pos`: Destination position vector in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
/// - `dest_world_pos`: Destination position vector in world units ([hwa::HwiContract::WORLD_UNIT_MAGNITUDE]).
/// - `tool_power`: Tool power utilized in the segment.
/// - `constraints`: Motion constraints applicable to the segment.
#[derive(Clone, Copy)]
pub struct Segment {
    /// Initial speed at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_enter_su_s: Real,
    /// Speed at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_exit_su_s: Real,
    /// Target speed for the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_target_su_s: Real,
    /// Total displacement to complete the movement in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
    pub displacement_su: Real,

    /// Constrained initial speed at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_enter_constrained_su_s: Real,
    /// Constrained speed at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_exit_constrained_su_s: Real,
    /// Projection of the previous segment.
    pub proj_prev: Real,
    /// Projection of the next segment.
    pub proj_next: Real,

    /// Unit vector for the direction of movement.
    pub unit_vector_dir: TVector<Real>,

    /// Source position vector in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
    pub src_pos: TVector<Real>,

    /// Destination position vector in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
    pub dest_pos: TVector<Real>,

    /// Destination position vector in world units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
    pub dest_world_pos: TVector<Real>,

    #[allow(unused)]
    /// Tool power utilized in the segment.
    pub tool_power: Real,
    /// Motion constraints applicable to the segment.
    pub constraints: crate::motion::Constraints, // Should remove this?
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
                _ => match coord_value.is_negligible() {
                    true => None,
                    false => Some(coord_value),
                },
            })
            .decompose_normal();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hwa::math;
    use crate::hwa::math::TVector;
    use crate::motion::{Constraints, SCurveMotionProfile, SegmentSampler};

    fn dummy_segment() -> Segment {
        Segment {
            speed_enter_su_s: hwa::make_real!(10.0),
            speed_exit_su_s: hwa::make_real!(15.0),
            speed_target_su_s: hwa::make_real!(20.0),
            displacement_su: (TVector::one() * math::ONE_HUNDRED).norm2().unwrap(),
            speed_enter_constrained_su_s: hwa::make_real!(10.0),
            speed_exit_constrained_su_s: hwa::make_real!(15.0),
            proj_prev: hwa::make_real!(0.0),
            proj_next: hwa::make_real!(100.0),
            unit_vector_dir: TVector::one().unit(),
            src_pos: TVector::zero(),
            dest_pos: TVector::one() * math::ONE_HUNDRED,
            dest_world_pos: TVector::one() * math::ONE_HUNDRED,
            tool_power: hwa::make_real!(5.0),
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
        assert_eq!(segment.speed_enter_su_s, hwa::make_real!(10.0));
        assert_eq!(segment.speed_exit_su_s, hwa::make_real!(15.0));
        assert_eq!(segment.speed_target_su_s, hwa::make_real!(20.0));
    }

    #[cfg(feature = "with-x-axis")]
    #[test]
    fn test_segment_deviation_correction() {
        let mut segment = dummy_segment();

        // Simulate a deviation of 0.1 in the 'x' axis
        let deviation = hwa::make_vector_real!(x = 0.1);
        let expected_dist = (TVector::one() * math::ONE_HUNDRED).norm2().unwrap();
        segment.src_pos += deviation;

        segment.fix_deviation(&deviation, hwa::make_real!(1.0));

        assert_eq!(segment.displacement_su, expected_dist);
        assert!(
            (segment.src_pos - TVector::zero())
                .norm2()
                .unwrap()
                .is_negligible()
        );
    }

    #[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))]
    #[test]
    fn test_segment_deviation_correction_in_boundaries() {
        let mut segment = Segment {
            speed_enter_su_s: hwa::make_real!(1.0),
            speed_exit_su_s: hwa::make_real!(1.0),
            speed_target_su_s: hwa::make_real!(10.0),
            displacement_su: hwa::make_real!(1.0),
            speed_enter_constrained_su_s: hwa::make_real!(1.0),
            speed_exit_constrained_su_s: hwa::make_real!(1.0),
            proj_prev: hwa::make_real!(0.0),
            proj_next: hwa::make_real!(0.0),
            unit_vector_dir: hwa::make_vector_real!(x = 0.0, y = 1.0),
            src_pos: hwa::make_vector_real!(x = 0.0, y = 0.0),
            dest_pos: hwa::make_vector_real!(x = 0.0, y = 1.0),
            dest_world_pos: hwa::make_vector_real!(x = 0.0, y = 1.0),
            tool_power: hwa::make_real!(0.0),
            constraints: Constraints {
                v_max: math::ONE_HUNDRED,
                a_max: math::ONE_THOUSAND,
                j_max: math::ONE_THOUSAND,
            },
        };

        // Simulate a deviation of 0.1 in the 'x' axis
        let deviation = hwa::make_vector_real!(x = 0.1);
        let expected_dist = hwa::make_real!(1.0);
        segment.src_pos += deviation;

        segment.fix_deviation(&deviation, hwa::make_real!(1.0));

        assert_eq!(segment.displacement_su.rdp(6), expected_dist.rdp(6));
    }

    #[test]
    fn test_segment_iterator() {
        use crate::motion::MotionProfile;
        let constraints = Constraints {
            v_max: hwa::make_real!(10.0),
            a_max: hwa::make_real!(2.0),
            j_max: hwa::make_real!(1.0),
        };
        let motion_profile = SCurveMotionProfile::compute(
            hwa::make_real!(20.0),
            hwa::make_real!(0.0),
            hwa::make_real!(5.0),
            &constraints,
            true,
        )
        .unwrap();
        // Set sampling time to 60% of profile time
        let sampling_period = motion_profile.end_time() * hwa::make_real!(0.6);
        let mut sampler = SegmentSampler::new(&motion_profile, sampling_period);

        // Test iterator before exhaustion (expected to be at 60%)
        let micro_segment = sampler.next();
        assert!(micro_segment.is_some());

        // Test iterator at exhaustion (expected to be at 120%), truncated to motion_profile.end_time() and exhausted
        let micro_segment = sampler.next();
        assert!(micro_segment.is_some());

        // Test iterator after exhaustion, expected to return None
        let micro_segment = sampler.next();
        assert!(micro_segment.is_none());
    }

    /// Ensure that the iterator reaches end position oly once if micro-segment advance
    /// reaches maximum time
    #[test]
    fn test_segment_iterator_exhaustion() {
        use crate::motion::MotionProfile;
        let constraints = Constraints {
            v_max: hwa::make_real!(10.0),
            a_max: hwa::make_real!(2.0),
            j_max: hwa::make_real!(1.0),
        };
        let motion_profile = SCurveMotionProfile::compute(
            hwa::make_real!(20.0),
            hwa::make_real!(0.0),
            hwa::make_real!(5.0),
            &constraints,
            true,
        )
        .unwrap();

        let sampling_period = hwa::make_real!(100.0);
        let mut sampler = SegmentSampler::new(&motion_profile, sampling_period);

        assert!(sampler.instant_speed().is_negligible());

        // Set to a time past the end of the profile
        let micro_segment = sampler.next();

        assert!(!sampler.instant_speed().is_negligible());

        assert_eq!(micro_segment.unwrap(), motion_profile.end_pos(), "At end");
        let micro_segment = sampler.next();
        assert!(micro_segment.is_none(), "Does not advance more");
        assert!(sampler.is_exhausted(), "Is exhausted");
    }
}
