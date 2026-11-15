//! Motion segment module
use crate::hwa;
use hwa::math::Real;
use hwa::math::TVector;

/// Represents the data for a motion segment.
///
/// # Fields
/// - `speed_enter_sus`: Initial speed at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `speed_exit_sus`: Speed at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `accel_enter_sus`: Initial acceleration at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
/// - `accel_exit_sus`: Acceleration at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
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
#[cfg_attr(feature = "with-log", derive(Debug))]
#[cfg_attr(feature = "with-defmt", derive(defmt::Format))]
pub struct Segment {
    /// Initial speed at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_enter_su_s: Real,
    /// Speed at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_exit_su_s: Real,
    /// Target speed for the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    #[cfg(feature = "with-motion-chaining")]
    /// Initial acceleration at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub accel_enter_su_s: Real,
    #[cfg(feature = "with-motion-chaining")]
    /// Acceleration at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub accel_exit_su_s: Real,

    #[cfg(feature = "with-motion-chaining")]
    /// Rest speed of the segment (the speed it can start and stop at instantly: the minimum
    /// speeds of its moving axes) in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    /// No corner needs a lower junction speed.
    pub speed_rest_su_s: Real,
    
    pub speed_target_su_s: Real,
    /// Total displacement to complete the movement in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]).
    pub displacement_su: Real,

    /// Constrained initial speed at the entry of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_enter_constrained_su_s: Real,
    /// Constrained speed at the exit of the segment in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec).
    pub speed_exit_constrained_su_s: Real,
    
    #[cfg(feature = "with-motion-chaining")]
    /// Projection of the previous segment.
    pub proj_prev: Real,
    #[cfg(feature = "with-motion-chaining")]
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
    /// The speed programmed for the move (its `F`, before the M220 speed rate, the speed limits
    /// and the throughput slowdown) in space units ([hwa::HwiContract::SPACE_UNIT_MAGNITUDE]/sec):
    /// the reference of the laser dynamic power (`M4`), which scales the power by the speed
    /// relative to it. Zero for the moves without a programmed cutting speed (`G0` rapids), where
    /// the laser is off (`M3` and `M4`).
    pub speed_programmed_su_s: Real,
    /// Motion constraints applicable to the segment.
    pub constraints: crate::motion::Constraints, // Should remove this?
}

impl Segment {
    /// Lowers the target speed of a segment still at rest (just created, not chained) to `v`, with
    /// every speed bound derived from it (see `motion::throughput_speed_cap`)
    #[cfg(feature = "with-motion-chaining")]
    pub fn cap_speed(&mut self, v: Real) {
        self.speed_target_su_s = self.speed_target_su_s.min(v);
        self.constraints.v_max = self.constraints.v_max.min(v);
        self.speed_enter_constrained_su_s = self.speed_enter_constrained_su_s.min(v);
        self.speed_exit_constrained_su_s = self.speed_exit_constrained_su_s.min(v);
        self.speed_enter_su_s = self.speed_enter_su_s.min(v);
        self.speed_exit_su_s = self.speed_exit_su_s.min(v);
        self.speed_rest_su_s = self.speed_rest_su_s.min(v);
    }

    pub fn fix_deviation(&mut self, deviation: &TVector<Real>) {
        // The extruder factor the segment was planned with (the M221 flow rate, or 1 for the
        // firmware retraction): its planned E displacement over its E position delta. Taken from
        // the segment, not from the current config: M221 may have changed since it was planned
        #[cfg(feature = "with-e-axis")]
        let _flow_rate = {
            let planned_e = self
                .unit_vector_dir
                .get_coord(hwa::CoordSel::E)
                .map(|u| u * self.displacement_su);
            let raw_e = (self.dest_pos - self.src_pos).get_coord(hwa::CoordSel::E);
            match (planned_e, raw_e) {
                (Some(p), Some(r)) if !r.is_zero() => p / r,
                _ => hwa::math::ONE,
            }
        };
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
    use crate::motion::{Constraints, HybridMotionProfile, SegmentSampler};

    fn dummy_segment() -> Segment {
        Segment {
            speed_enter_su_s: hwa::make_real!(10.0),
            speed_exit_su_s: hwa::make_real!(15.0),
            #[cfg(feature = "with-motion-chaining")]
            accel_enter_su_s: math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            accel_exit_su_s: math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            speed_rest_su_s: math::ZERO,
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
            speed_programmed_su_s: hwa::make_real!(0.0),
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

        segment.fix_deviation(&deviation);

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
            #[cfg(feature = "with-motion-chaining")]
            accel_enter_su_s: math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            accel_exit_su_s: math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            speed_rest_su_s: math::ZERO,
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
            speed_programmed_su_s: hwa::make_real!(0.0),
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

        segment.fix_deviation(&deviation);

        assert_eq!(segment.displacement_su.rdp(6), expected_dist.rdp(6));
    }

    /// The extruder factor a segment was planned with (M221 flow rate, or 1 for the firmware
    /// retraction) is kept by the deviation correction, whatever the current M221 setting is
    #[cfg(all(feature = "with-x-axis", feature = "with-e-axis"))]
    #[test]
    fn test_segment_deviation_correction_keeps_flow_factor() {
        for factor in [0.5f32, 1.0, 1.5] {
            let raw = hwa::make_vector_real!(x = 3.0, e = 4.0);
            let mut planned = hwa::make_vector_real!(x = 3.0, e = 4.0);
            planned.set_coord(hwa::CoordSel::E, Some(Real::from_f32(4.0 * factor)));
            let (unit_vector_dir, displacement_su) = planned.decompose_normal();
            let mut segment = Segment {
                #[cfg(feature = "with-motion-chaining")]
                accel_enter_su_s: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                accel_exit_su_s: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                speed_rest_su_s: math::ZERO,
                speed_enter_su_s: math::ZERO,
                speed_exit_su_s: math::ZERO,
                speed_target_su_s: hwa::make_real!(10.0),
                displacement_su,
                speed_enter_constrained_su_s: math::ZERO,
                speed_exit_constrained_su_s: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                proj_prev: math::ZERO,
                #[cfg(feature = "with-motion-chaining")]
                proj_next: math::ZERO,
                unit_vector_dir,
                src_pos: TVector::zero(),
                dest_pos: raw,
                dest_world_pos: raw,
                tool_power: math::ZERO,
                speed_programmed_su_s: math::ZERO,
                constraints: Constraints {
                    v_max: math::ONE_HUNDRED,
                    a_max: math::ONE_THOUSAND,
                    j_max: math::ONE_THOUSAND,
                },
            };
            // A deviation of 0.1 in X: the E displacement must keep its planned factor
            let deviation = hwa::make_vector_real!(x = 0.1);
            segment.fix_deviation(&deviation);
            let e = segment.unit_vector_dir.get_coord(hwa::CoordSel::E).unwrap() * segment.displacement_su;
            let x = segment.unit_vector_dir.get_coord(hwa::CoordSel::X).unwrap() * segment.displacement_su;
            assert_eq!(e.rdp(4), Real::from_f32(4.0 * factor).rdp(4), "factor {}", factor);
            assert_eq!(x.rdp(4), hwa::make_real!(3.1).rdp(4), "factor {}", factor);
        }
    }

    #[test]
    fn test_segment_iterator() {
        use crate::motion::MotionProfile;
        let constraints = Constraints {
            v_max: hwa::make_real!(10.0),
            a_max: hwa::make_real!(2.0),
            j_max: hwa::make_real!(1.0),
        };
        let motion_profile = HybridMotionProfile::compute(
            hwa::make_real!(20.0),
            hwa::make_real!(0.0),
            hwa::make_real!(5.0),
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
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
        let motion_profile = HybridMotionProfile::compute(
            hwa::make_real!(20.0),
            hwa::make_real!(0.0),
            hwa::make_real!(5.0),
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
            &constraints,
            true,
        )
        .unwrap();

        let sampling_period = hwa::make_real!(100.0);
        let mut sampler = SegmentSampler::new(&motion_profile, sampling_period);

        assert!(sampler.current_speed().is_negligible());

        // Set to a time past the end of the profile
        let micro_segment = sampler.next();

        assert!(!sampler.current_speed().is_negligible());

        assert_eq!(micro_segment.unwrap(), motion_profile.end_pos(), "At end");
        let micro_segment = sampler.next();
        assert!(micro_segment.is_none(), "Does not advance more");
        assert!(sampler.is_exhausted(), "Is exhausted");
    }
}
