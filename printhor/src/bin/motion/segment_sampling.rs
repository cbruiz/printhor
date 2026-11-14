use crate::{hwa, motion};
use hwa::math::Real;
use printhor_hwa_common::math;

/// Iterator over the micro-segments of a motion profile.
///
/// Time is tracked as an **integer number of microseconds**: every sample instant is
/// `k · period_us` (and the last one is exactly the profile end), so there is no accumulation of
/// rounding error over long segments and the window widths handed to the step generator add up
/// exactly to the segment duration.
///
/// Guarantees:
/// * The returned positions are monotone and bounded by `[0, end_pos]`.
/// * The last returned position is **exactly** `end_pos` (no step is lost at the segment end).
/// * No window is shorter than half a period: a short tail is merged into the previous window
///   (formerly a sub-microsecond tail rounded to a zero-width window and was dropped).
///
/// # Type Parameters
/// - `'a`: Lifetime of the motion profile reference.
/// - `P`: Type of the motion profile, which must implement the [motion::MotionProfile] trait.
pub struct SegmentSampler<'a, P>
where
    P: motion::MotionProfile,
{
    /// Reference to the motion profile.
    profile: &'a P,
    /// Sampling period in µs (≥ 1)
    period_us: u64,
    /// Profile duration in µs (≥ 1)
    end_us: u64,
    /// Instant of the previous sample in µs
    prev_us: u64,
    /// Instant of the last sample in µs
    now_us: u64,
    /// Last evaluation position in (work) space units.
    last_evaluated_position_su: Real,
    ds: Real,
    /// State flag indicating whether the iterator is exhausted.
    exhausted: bool,
}

/// `Real` seconds → integer microseconds (rounded, saturating at 0)
fn secs_to_us(t: Real) -> u64 {
    (t * math::ONE_MILLION)
        .round()
        .to_i64()
        .map(|v| v.max(0) as u64)
        .unwrap_or(0)
}

/// Integer microseconds → `Real` seconds (a multiplication instead of `from_lit(us, 6)`, which
/// costs a power and a division per call)
#[inline]
fn us_to_secs(us: u64) -> Real {
    Real::from_lit(us as i64, 0) * Real::from_f32(1.0e-6)
}

impl<'a, P> SegmentSampler<'a, P>
where
    P: motion::MotionProfile,
{
    /// Creates a new sampler.
    ///
    /// # Parameters
    /// - `profile`: Reference to the motion profile.
    /// - `sampling_period_s`: The sampling period in seconds (rounded to whole microseconds).
    pub fn new(profile: &'a P, sampling_period_s: Real) -> Self {
        Self::new_us(profile, secs_to_us(sampling_period_s))
    }

    /// Creates a new sampler with the sampling period given in microseconds.
    pub fn new_us(profile: &'a P, sampling_period_us: u64) -> Self {
        SegmentSampler {
            profile,
            period_us: sampling_period_us.max(1),
            end_us: secs_to_us(profile.end_time()).max(1),
            prev_us: 0,
            now_us: 0,
            last_evaluated_position_su: Real::zero(),
            ds: Real::zero(),
            exhausted: false,
        }
    }

    pub fn current_position(&self) -> Real {
        self.last_evaluated_position_su
    }

    pub fn current_interval(&self) -> usize {
        self.profile.eval_interval(self.current_time())
    }

    pub fn current_speed(&self) -> Real {
        self.profile.eval_velocity(self.current_time())
    }

    pub fn current_acceleration(&self) -> Real {
        self.profile.eval_acceleration(self.current_time())
    }

    pub fn current_time(&self) -> Real {
        if self.exhausted {
            // Exact end instant (not the µs-rounded one)
            self.profile.end_time()
        } else {
            us_to_secs(self.now_us)
        }
    }

    /// Gets the instantaneous \[ds\] of the last advance
    pub fn ds(&self) -> Real {
        self.ds
    }

    /// Gets the width in seconds of the last advance
    pub fn dt(&self) -> Real {
        us_to_secs(self.dt_us())
    }

    /// Gets the width in microseconds of the last advance (≥ 1 after the first sample)
    pub fn dt_us(&self) -> u64 {
        self.now_us - self.prev_us
    }

    /// Gets the mean speed \[\frac{ds}{dt}\] of the last advance
    pub fn instant_velocity(&self) -> Real {
        let dt = self.dt();
        if dt.is_negligible() {
            Real::zero()
        } else {
            self.ds / dt
        }
    }

    /// Gets the instantaneous acceleration
    pub fn instant_acceleration(&self) -> Real {
        self.current_acceleration()
    }

    /// Advances the iterator and returns the next micro-segment position.
    ///
    /// # Returns
    /// An `Option` containing:
    /// - The position as real
    /// - `None` if exhausted.
    pub fn next(&mut self) -> Option<Real> {
        if self.exhausted {
            return None;
        }
        let mut t_next = self.now_us + self.period_us;
        // Merge a short tail (< half a period) into this window
        if t_next + (self.period_us >> 1) >= self.end_us {
            t_next = self.end_us;
        }
        self.prev_us = self.now_us;
        self.now_us = t_next;

        let end_pos = self.profile.end_pos();
        let p = if t_next >= self.end_us {
            self.exhausted = true;
            end_pos
        } else {
            self.profile
                .eval_position(us_to_secs(t_next))
                .max(self.last_evaluated_position_su)
                .min(end_pos)
        };
        self.ds = p - self.last_evaluated_position_su;
        self.last_evaluated_position_su = p;
        Some(p)
    }

    /// Checks if the segment is exhausted
    pub fn is_exhausted(&self) -> bool {
        self.exhausted
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{Constraints, HybridMotionProfile, MotionProfile};

    fn profile(s: f32, v0: f32, v1: f32) -> HybridMotionProfile {
        HybridMotionProfile::compute(
            Real::from_f32(s),
            Real::from_f32(v0),
            Real::from_f32(v1),
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
            #[cfg(feature = "with-motion-chaining")]
            math::ZERO,
            &Constraints {
                v_max: Real::from_f32(100.0),
                a_max: Real::from_f32(3000.0),
                j_max: Real::from_f32(150000.0),
            },
            false,
        )
        .unwrap()
    }

    /// The windows add up exactly to the (µs-rounded) duration, none is degenerate, positions are
    /// monotone and the last one is exactly the end position.
    #[test]
    fn windows_cover_the_whole_segment_exactly() {
        for &(s, v0, v1) in &[(10.0f32, 0.0f32, 0.0f32), (123.4, 5.0, 0.0), (0.05, 0.0, 0.0), (3.0, 40.0, 40.0)] {
            let p = profile(s, v0, v1);
            for period_us in [100u64, 2500, 10000, 333] {
                let mut sampler = SegmentSampler::new_us(&p, period_us);
                let mut total_us = 0u64;
                let mut last = Real::zero();
                let mut n = 0;
                while let Some(pos) = sampler.next() {
                    let w = sampler.dt_us();
                    assert!(w >= 1, "zero-width window");
                    let end_us = secs_to_us(p.end_time()).max(1);
                    if sampler.is_exhausted() {
                        assert!(w > period_us / 2 || w == end_us, "short tail must be merged: w={} period={}", w, period_us);
                    } else {
                        assert_eq!(w, period_us);
                    }
                    assert!(pos >= last, "monotone");
                    total_us += w;
                    last = pos;
                    n += 1;
                    assert!(n < 10_000_000);
                }
                assert_eq!(last, p.end_pos(), "last sample must be exactly the end position");
                assert_eq!(total_us, secs_to_us(p.end_time()).max(1));
            }
        }
    }
}
