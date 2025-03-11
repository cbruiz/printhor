use crate::{hwa, motion};
use hwa::math::Real;

/// Iterator over motion segments.
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
    /// The time step in secs to increment in each iteration.
    sampling_period_s: Real,
    /// Last evaluation position in (work) space units.
    last_evaluated_position_su: Real,
    /// Last evaluation time instant.
    last_evaluated_time_s: Real,
    ds: Real,
    dt: Real,
    /// State flag indicating whether the iterator is exhausted.
    exhausted: bool,
}

impl<'a, P> SegmentSampler<'a, P>
where
    P: motion::MotionProfile,
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
        SegmentSampler {
            profile,
            sampling_period_s,
            last_evaluated_position_su: Real::zero(),
            last_evaluated_time_s: Real::zero(),
            ds: Real::zero(),
            dt: Real::zero(),
            exhausted: false,
        }
    }

    pub fn current_position(&self) -> Real {
        self.last_evaluated_position_su
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
                        self.ds = end_pos - self.last_evaluated_position_su;
                        self.last_evaluated_position_su = end_pos;
                    } else {
                        self.ds = p - self.last_evaluated_position_su;
                        self.last_evaluated_position_su = p;
                    }
                    Some(self.last_evaluated_position_su)
                }
            }
        }
    }
    
    /// Checks if the segment is exhausted
    pub fn is_exhausted(&self) -> bool {
        self.exhausted
    }
}