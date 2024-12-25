#[cfg(all(feature = "native", feature = "plot-timings"))]
use super::timing_monitor::*;
use crate::hwa;
use crate::math;
use crate::math::Real;
use crate::math::{ArithmeticOps, CoordSel, TVector};
#[allow(unused)]
use core::ops::Neg;
use embassy_time::Duration;
#[allow(unused)]
#[cfg(feature = "with-probe")]
use hwa::controllers::ProbeTrait;

pub struct MotionDriver {
    #[cfg(feature = "with-motion")]
    pub pins: hwa::controllers::MotionPins,
    #[cfg(feature = "with-trinamic")]
    pub trinamic_controller: hwa::controllers::TrinamicController,
    #[cfg(feature = "with-probe")]
    pub probe_controller: hwa::types::ProbeController,
    #[cfg(feature = "with-fan-layer")]
    pub fan_layer_controller: hwa::types::FanLayerController,
    #[cfg(feature = "with-fan-extra-1")]
    pub fan_extra_1_controller: hwa::types::FanExtra1Controller,
    #[cfg(feature = "with-laser")]
    pub _laser_controller: hwa::types::LaserController,
    #[cfg(all(feature = "native", feature = "plot-timings"))]
    tmon: TimingsMonitor,
}

#[cfg(feature = "with-motion")]
impl MotionDriver {
    pub fn new(
        pins: hwa::controllers::MotionPins,
        //#[cfg(feature = "with-trinamic")]
        //trinamic_controller: hwa::controllers::TrinamicController::new(params.motion_device.trinamic_uart, params.motion_config, ),
        #[cfg(feature = "with-probe")] probe_controller: hwa::types::ProbeController,
        #[cfg(feature = "with-fan-layer")] fan_layer_controller: hwa::types::FanLayerController,
        #[cfg(feature = "with-fan-extra-1")]
        fan_extra_1_controller: hwa::types::FanExtra1Controller,
        #[cfg(feature = "with-laser")] _laser_controller: hwa::types::LaserController,
        #[cfg(all(feature = "native", feature = "plot-timings"))] tmon: TimingsMonitor::new(),
    ) -> Self {
        Self {
            pins,
            #[cfg(feature = "with-trinamic")]
            trinamic_controller: hwa::controllers::TrinamicController::new(
                params.motion_device.trinamic_uart,
                params.motion_config,
            ),
            #[cfg(feature = "with-probe")]
            probe_controller,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_controller,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra_1_controller,
            #[cfg(feature = "with-laser")]
            _laser_controller,
            #[cfg(all(feature = "native", feature = "plot-timings"))]
            tmon: TimingsMonitor::new(),
        }
    }

    pub fn pins(&self) -> &hwa::controllers::MotionPins {
        &self.pins
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn update_clock(&mut self, real_time: embassy_time::Instant) {
        self.tmon.set_clock(real_time)
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn start_segment(
        &mut self,
        ref_time: embassy_time::Instant,
        real_time: embassy_time::Instant,
    ) {
        self.tmon.reset(ref_time, real_time)
    }
    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn end_segment(&mut self) {
        self.tmon.commit()
    }

    #[cfg(all(feature = "native", feature = "plot-timings"))]
    #[inline]
    pub fn mark_microsegment(&mut self) {
        self.tmon.swap(PinState::USCLK)
    }

    pub fn enable_and_set_dir(&mut self, vdir: &TVector<Real>) {
        self.pins().enable_steppers(hwa::StepperChannel::all());

        let mut dir_fwd = hwa::StepperChannel::empty();

        #[cfg(feature = "with-x-axis")]
        if vdir.x.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(hwa::StepperChannel::X, true)
        }
        #[cfg(feature = "with-y-axis")]
        if vdir.y.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(hwa::StepperChannel::Y, true)
        }
        #[cfg(feature = "with-z-axis")]
        if vdir.z.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(hwa::StepperChannel::Z, true)
        }
        #[cfg(feature = "with-e-axis")]
        if vdir.e.and_then(|v| Some(v.is_positive())).unwrap_or(false) {
            dir_fwd.set(hwa::StepperChannel::E, true)
        }
        self.pins()
            .set_forward_direction(dir_fwd, hwa::StepperChannel::all());
    }

    /// This method performs the homing action for the stepper motors.
    ///
    /// The homing action involves moving the stepper motors to a known reference
    /// position. This is typically done by moving the motors towards the end-stop
    /// switches and then setting the current position to zero, which serves as a
    /// reference point for future movements.
    ///
    /// # Arguments
    ///
    /// * `motion_config_ref` - A reference to the motion configuration which contains
    ///                         parameters such as units per millimeter, micro-steps
    ///                         per axis, and machine bounds.
    ///
    /// # Returns
    ///
    /// A `Result` with the new position as `TVector<Real>`, or an error with the
    /// homing position if the homing action fails.
    ///
    /// # Examples
    ///
    /// ```
    /// let result = controller.homing_action(&motion_config_ref).await;
    /// match result {
    ///     Ok(position) => println!("New homing position: {:?}", position),
    ///     Err(err_position) => eprintln!("Homing failed at position: {:?}", err_position),
    /// }
    /// ```
    ///
    /// # Errors
    ///
    /// This function will return an error if it is unable to complete the homing
    /// process, such as if an end-stop switch is not triggered within the expected
    /// range of motion.
    // TODO: Carefully review
    pub async fn homing_action(
        &mut self,
        motion_config: &hwa::controllers::MotionConfig,
    ) -> Result<TVector<Real>, TVector<Real>> {
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing]");

        let mut homming_position = TVector::zero();

        let steps_per_mm = motion_config
            .get_steps_per_unit_as_vector()
            .with_coord(CoordSel::E, None);

        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Steps per mm: {:?}", steps_per_mm);
        let machine_bounds = motion_config.get_machine_bounds();

        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] Machine bounds: {:?}",
            machine_bounds
        );

        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] - Assuming at: {:?}",
            homming_position
        );

        // Raise Z axis 10mm to avoid obstacles during X and Y homing
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Raise Z +10mm");
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE), None),
            Real::from_lit(10, 0),
            steps_per_mm,
            2000,
            false,
            Some(&mut homming_position),
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        // Home the X axis quickly
        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] Quick Homing X axis with a max of -{:?} mm",
            machine_bounds.x.unwrap_or(math::ZERO)
        );
        self.shabbily_move_to(
            TVector::from_coords(Some(math::ONE.neg()), None, None, None),
            machine_bounds.x.unwrap_or(Real::zero()),
            steps_per_mm,
            2000,
            true,
            None,
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Quick Separate X axis 5 mm");
        // Home the X axis precisely
        self.shabbily_move_to(
            TVector::from_coords(Some(math::ONE), None, None, None),
            Real::from_lit(5, 0),
            steps_per_mm,
            1000,
            false,
            Some(&mut homming_position),
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [trace-commands] [Homing] Slow approximate X axis up to 6 mm");

        // Home the X axis precisely
        self.shabbily_move_to(
            TVector::from_coords(Some(math::ONE.neg()), None, None, None),
            Real::from_lit(6, 0),
            steps_per_mm,
            1000,
            true,
            Some(&mut homming_position),
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        // Home the Y axis quickly
        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] Quick Homing Y axis with a max of -{:?} mm",
            machine_bounds.y.unwrap_or(math::ZERO)
        );
        self.shabbily_move_to(
            TVector::from_coords(None, Some(math::ONE.neg()), None, None),
            machine_bounds.y.unwrap_or(Real::zero()),
            steps_per_mm,
            2000,
            true,
            None,
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Quick Separate Y axis 5 mm");
        // Home the Y axis precisely
        self.shabbily_move_to(
            TVector::from_coords(None, Some(math::ONE), None, None),
            Real::from_lit(5, 0),
            steps_per_mm,
            2000,
            false,
            Some(&mut homming_position),
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Slow approximate Y axis up to 6 mm");
        // Home the X axis precisely
        self.shabbily_move_to(
            TVector::from_coords(None, Some(math::ONE.neg()), None, None),
            Real::from_lit(6, 0),
            steps_per_mm,
            1000,
            true,
            Some(&mut homming_position),
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        // TODO:
        // Go to center
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Upper Z + 10mm");
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE.neg()), None),
            Real::from_lit(10, 0),
            steps_per_mm,
            1000,
            false,
            Some(&mut homming_position),
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                self.probe_controller.lock().await.probe_pin_down(100).await;
            }
        }

        // FIXME: As of now, until tested in real hardware, for safety
        let zbounds = homming_position.z.unwrap_or(math::ZERO)
            + machine_bounds.z.unwrap_or(math::ONE) / Real::from_lit(16, 0);
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Lower Z up to {:?} mm", zbounds); // reduced for safety
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE.neg()), None),
            zbounds,
            steps_per_mm,
            1000,
            true,
            None,
        )
        .await;
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homming_position);

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-probe")] {
                self.probe_controller.lock().await.probe_pin_up(100).await;
            }
        }

        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] Upper Z + 10mm again por safety");
        self.shabbily_move_to(
            TVector::from_coords(None, None, Some(math::ONE), None),
            Real::from_lit(10, 0),
            steps_per_mm,
            2000,
            false,
            Some(&mut homming_position),
        )
        .await;

        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] Done. Finally at: {:?}",
            homming_position
        );

        Ok(homming_position)
    }

    async fn shabbily_move_to(
        &mut self,
        vdir: TVector<Real>,
        module: Real,
        steps_per_mm: TVector<Real>,
        step_frequency: u64,
        check_endstops: bool,
        position: Option<&mut TVector<Real>>,
    ) -> TVector<u32> {
        let steps_to_advance: TVector<u32> = (vdir * module * steps_per_mm)
            .abs()
            .round()
            .map_coords(|c| c.to_i32().and_then(|c| Some(c as u32)));
        let mut steps_advanced: TVector<u32> = TVector::zero();

        self.enable_and_set_dir(&vdir);

        let mut channel = hwa::StepperChannel::empty();
        let mut coord_sel = CoordSel::empty();
        steps_to_advance.apply_coords(|(coord, v)| {
            let applied = v.is_defined_positive();
            if coord.contains(CoordSel::X) {
                channel.set(hwa::StepperChannel::X, applied);
                coord_sel.set(CoordSel::X, applied);
            }
            if coord.contains(CoordSel::Y) {
                channel.set(hwa::StepperChannel::Y, applied);
                coord_sel.set(CoordSel::Y, applied);
            }
            if coord.contains(CoordSel::Z) {
                channel.set(hwa::StepperChannel::Z, applied);
                coord_sel.set(CoordSel::Z, applied);
            }
        });

        let mut ticker = embassy_time::Ticker::every(Duration::from_hz(step_frequency));

        loop {
            let mut completed = false;
            if check_endstops && self.pins().end_stop_triggered(channel) {
                hwa::debug!("ENDSTOP TRIGGERED");
                completed = true;
            }
            if !steps_advanced.bounded_by(&steps_to_advance) {
                hwa::trace!("FULL ADV");
                completed = true;
            }
            if completed {
                match position {
                    Some(mut _p) => {
                        *_p += vdir
                            * steps_advanced.map_coords(|c| Some(Real::from_lit(c.into(), 0)))
                            / steps_per_mm;
                    }
                    None => {}
                }

                return steps_advanced;
            }
            cfg_if::cfg_if! {
                if #[cfg(feature="pulsed")] {
                    compile_error!("Not yet implemented");
                    self.ping().step_high(channel);
                    // TODO
                    //crate::control::motion_timing::s_block_for(Duration::from_micros(1));
                    self.pins().step_low(channel);

                }
                else {
                    self.pins().step_toggle(channel);
                }
            }
            steps_advanced.increment(coord_sel, 1);
            ticker.next().await;
        }
    }
}
