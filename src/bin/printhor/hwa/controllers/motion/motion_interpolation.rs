use printhor_hwa_common::StepperChannel;
use crate::hwa::controllers::MultiTimer;
use crate::math;
use crate::math::Real;
use crate::tgeo::TVector;

/// A struct for interpolating microsteps along linear trajectories.
pub struct LinearMicrosegmentStepInterpolator {
    vdir_abs: TVector<Real>,
    usteps_per_mm: TVector<Real>,

    /// Number of discrete steps along vector already advanced.
    usteps_advanced: TVector<u32>,

    /// The number of discrete steps by axis already advanced.
    axis_steps_advanced_precise: TVector<Real>,

    /// The number of discrete steps by axis to advance.
    axis_steps_to_advance_precise: TVector<Real>,

    multi_timer: MultiTimer,
    #[cfg(any(test, feature = "assert-motion"))]
    pub(crate) delta: TVector<u32>,
}

impl LinearMicrosegmentStepInterpolator {
    /// Constructs a new `LinearMicrosegmentStepInterpolator`.
    ///
    /// # Arguments
    ///
    /// * `vdir_abs` - The direction (unitary) vector in positive coordinates.
    /// * `distance` - The distance to move in millimeters.
    /// * `usteps_per_mm` - Microsteps per millimeter.
    ///
    /// # Returns
    ///
    /// A new `LinearMicrosegmentStepInterpolator`.
    pub fn new(vdir_abs: TVector<Real>, distance: Real, usteps_per_mm: TVector<Real>) -> Self {
        Self {
            vdir_abs,
            usteps_per_mm,
            usteps_advanced: TVector::zero(),
            axis_steps_advanced_precise: TVector::zero(),
            axis_steps_to_advance_precise: (vdir_abs * distance * usteps_per_mm).floor(),
            multi_timer: MultiTimer::new(),
            #[cfg(any(test, feature = "assert-motion"))]
            delta: TVector::zero(),
        }
    }

    /// Advances the position to the given estimated position.
    ///
    /// # Arguments
    ///
    /// * `estimated_position` - The estimated position to advance to.
    /// * `width` - The width for centering calculation.
    ///
    /// # Returns
    ///
    /// A boolean indicating whether advancement is successful.
    pub fn advance_to(&mut self, estimated_position: Real, width: Real) -> bool {
        let axial_pos: TVector<Real> = self.vdir_abs * estimated_position;
        let step_pos: TVector<Real> = axial_pos * self.usteps_per_mm;
        let steps_to_advance = (step_pos - self.axis_steps_advanced_precise)
            .clamp_min(TVector::zero())
            .clamp(self.axis_steps_to_advance_precise);
        let steps_to_advance_precise_1: TVector<Real> = steps_to_advance;
        let steps_to_advance_precise = steps_to_advance_precise_1.floor();
        self.axis_steps_advanced_precise += steps_to_advance_precise;

        #[cfg(any(test, feature = "assert-motion"))]
        {
            self.delta = steps_to_advance_precise.map_all(|c| {
                c.unwrap_or(math::ZERO)
                    .to_i32()
                    .and_then(|i| Some(i as u32))
            });
        }

        let _can_advance_more = self
            .axis_steps_advanced_precise
            .bounded_by(&self.axis_steps_to_advance_precise);

        // Centering formula:
        // x = width + num_steps / num_steps

        let numerator = width.to_i32().unwrap_or(0) as u32;
        if numerator == 0 {
            panic!("bad advance");
        }
        let tick_period_by_axis = steps_to_advance_precise.map_all(|c| {
            c.unwrap_or(math::ZERO).to_i32().and_then(|i| {
                if i > 0 {
                    Some(numerator / (i as u32))
                } else {
                    None
                }
            })
        });

        let step_increment =
            steps_to_advance_precise.map_coords(|cv| cv.to_i32().and_then(|c| Some(c as u32)));

        self.usteps_advanced += step_increment;
        self.multi_timer.set_width(numerator);
        self.multi_timer.set_max_count(&step_increment);

        #[cfg(test)]
        {
            std::println!("...");
            std::println!(
                "step_pos: {} steps_to_advance : {}",
                step_pos,
                steps_to_advance
            );
            std::println!(
                "delta: {} steps_to_advance_precise : {}",
                self.delta,
                steps_to_advance_precise
            );
            std::println!(
                "width: {} tick period by axis: {}",
                numerator,
                tick_period_by_axis
            );
        }

        #[cfg(feature = "with-x-axis")]
        self.multi_timer
            .set_channel_ticks(StepperChannel::X, tick_period_by_axis.x);
        #[cfg(feature = "with-y-axis")]
        self.multi_timer
            .set_channel_ticks(StepperChannel::Y, tick_period_by_axis.y);
        #[cfg(feature = "with-z-axis")]
        self.multi_timer
            .set_channel_ticks(StepperChannel::Z, tick_period_by_axis.z);
        #[cfg(feature = "with-e-axis")]
        self.multi_timer
            .set_channel_ticks(StepperChannel::E, tick_period_by_axis.e);
        //can_advance_more
        true
    }

    /// Returns the internal state of the `MultiTimer`.
    ///
    /// # Returns
    ///
    /// A reference to the `MultiTimer`.
    pub fn state(&self) -> &MultiTimer {
        &self.multi_timer
    }

    /// Returns the number of advanced steps.
    ///
    /// # Returns
    ///
    /// A `TVector` containing the number of advanced steps.
    #[inline]
    pub fn advanced_steps(&self) -> TVector<u32> {
        self.usteps_advanced
    }

    /// Returns the number of advanced millimeters.
    ///
    /// # Returns
    ///
    /// A `TVector` containing the number of advanced millimeters.
    #[allow(unused)]
    pub fn advanced_mm(&self) -> TVector<Real> {
        self.advanced_steps()
            .map_coords(|c| Some(Real::from_lit(c.into(), 0)))
            / self.usteps_per_mm
    }

    /// Returns the width of the multi timer.
    ///
    /// # Returns
    ///
    /// The width of the multi timer as a `u32`.
    #[allow(unused)]
    pub fn width(&self) -> u32 {
        self.multi_timer.width()
    }
}

#[cfg(test)]
pub mod interpolation_test {

    #[cfg(feature = "wip-tests")]
    #[test]
    fn interpolation_test_1() {
        use crate::hwa::controllers::LinearMicrosegmentStepInterpolator;
        use crate::math;
        use crate::math::Real;
        use crate::tgeo::TVector;

        let mut lin = LinearMicrosegmentStepInterpolator::new(
            TVector::from_coords(Some(math::ONE), None, None, None),
            Real::from_f32(100.0),
            TVector::from_coords(Some(Real::from_f32(160.0)), None, None, None),
        );

        lin.advance_to(Real::from_f32(100.0), Real::from_f32(100.0));

        std::println!("xx");
    }
}