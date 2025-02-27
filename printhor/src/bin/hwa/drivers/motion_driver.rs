use crate::hwa;
#[allow(unused)]
use core::ops::Neg;
use embassy_time::Duration;
use hwa::Contract;
#[allow(unused)]
use hwa::HwiContract;
#[allow(unused)]
#[cfg(feature = "with-probe")]
use hwa::controllers::ProbeTrait;
use hwa::math;
use math::Real;
use math::{ArithmeticOps, CoordSel, TVector};

pub struct MotionDriver {
    #[cfg(feature = "with-motion")]
    pub step_actuator: hwa::controllers::StepActuatorController,
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
}

#[cfg(feature = "with-motion")]
impl MotionDriver {
    pub fn new(
        step_actuator: hwa::controllers::StepActuatorController,
        #[cfg(feature = "with-trinamic")] trinamic_controller: hwa::controllers::TrinamicController,
        #[cfg(feature = "with-probe")] probe_controller: hwa::types::ProbeController,
        #[cfg(feature = "with-fan-layer")] fan_layer_controller: hwa::types::FanLayerController,
        #[cfg(feature = "with-fan-extra-1")]
        fan_extra_1_controller: hwa::types::FanExtra1Controller,
        #[cfg(feature = "with-laser")] _laser_controller: hwa::types::LaserController,
    ) -> Self {
        Self {
            step_actuator,
            #[cfg(feature = "with-trinamic")]
            trinamic_controller,
            #[cfg(feature = "with-probe")]
            probe_controller,
            #[cfg(feature = "with-fan-layer")]
            fan_layer_controller,
            #[cfg(feature = "with-fan-extra-1")]
            fan_extra_1_controller,
            #[cfg(feature = "with-laser")]
            _laser_controller,
        }
    }

    pub fn step_actuator(&self) -> &hwa::controllers::StepActuatorController {
        &self.step_actuator
    }

    pub fn enable_and_set_dir(&mut self, vdir: &TVector<Real>) {
        self.step_actuator()
            .enable_steppers(hwa::CoordSel::all_axis());
        let mut dir_fwd = hwa::CoordSel::empty();

        vdir.foreach_values(|coord, v| dir_fwd.set(coord, v.is_defined_positive()));
        self.step_actuator()
            .set_forward_direction(dir_fwd, CoordSel::all_axis());
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

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion-broadcast")] {
                if self.step_actuator.broadcast_channel.try_send(hwa::MotionBroadcastEvent::Reset).is_err() {
                    todo!("...");
                }
            }
        }

        #[allow(unused_mut)]
        let mut homing_position = TVector::zero();

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-e-axis")] {
                let _steps_per_world_unit = motion_config
                    .get_steps_per_space_unit_as_vector()
                    .with_coord(CoordSel::E, None);
            }
            else {
                let _steps_per_world_unit = motion_config.get_steps_per_space_unit_as_vector();
            }
        }

        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] Steps per {}: {:?}",
            hwa::Contract::WORLD_UNIT_MAGNITUDE,
            _steps_per_world_unit
        );
        let _machine_bounds = motion_config.get_machine_bounds();

        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] Machine bounds: {:?}",
            _machine_bounds
        );

        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] - Assuming at: {:?}",
            homing_position
        );

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-z-axis")] {
                // Raise Z axis 10mm to avoid obstacles during X and Y homing
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Raise Z +10mm");
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Z, Some(math::ONE)),
                    Real::from_lit(10, 0),
                    _steps_per_world_unit,
                    2000,
                    false,
                    Some(&mut homing_position),
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-x-axis")] {
                // Home the X axis quickly
                #[cfg(feature = "trace-commands")]
                hwa::info!(
                    "[trace-commands] [Homing] Quick Homing X axis with a max of -{:?} mm",
                    _machine_bounds.x.unwrap_or(math::ZERO)
                );
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::X, Some(math::ONE.neg())),
                    _machine_bounds.x.unwrap_or(Real::zero()),
                    _steps_per_world_unit,
                    2000,
                    true,
                    None,
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Quick Separate X axis 5 mm");
                // Home the X axis precisely
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::X, Some(math::ONE)),
                    Real::from_lit(5, 0),
                    _steps_per_world_unit,
                    1000,
                    false,
                    Some(&mut homing_position),
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);

                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [trace-commands] [Homing] Slow approximate X axis up to 6 mm");

                // Home the X axis precisely
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::X, Some(math::ONE.neg())),
                    Real::from_lit(6, 0),
                    _steps_per_world_unit,
                    1000,
                    true,
                    Some(&mut homing_position),
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-y-axis")] {
                // Home the Y axis quickly
                #[cfg(feature = "trace-commands")]
                hwa::info!(
                    "[trace-commands] [Homing] Quick Homing Y axis with a max of -{:?} mm",
                    _machine_bounds.y.unwrap_or(math::ZERO)
                );
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Y, Some(math::ONE.neg())),
                    _machine_bounds.y.unwrap_or(Real::zero()),
                    _steps_per_world_unit,
                    2000,
                    true,
                    None,
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);

                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Quick Separate Y axis 5 mm");
                // Home the Y axis precisely
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Y, Some(math::ONE)),
                    Real::from_lit(5, 0),
                    _steps_per_world_unit,
                    2000,
                    false,
                    Some(&mut homing_position),
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);

                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Slow approximate Y axis up to 6 mm");
                // Home the X axis precisely
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::X, Some(math::ONE.neg())),
                    Real::from_lit(6, 0),
                    _steps_per_world_unit,
                    1000,
                    true,
                    Some(&mut homing_position),
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))] {
                // TODO:
                // Go to center
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-z-axis")] {
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Upper Z + 10mm");
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Z, Some(math::ONE.neg())),
                    Real::from_lit(10, 0),
                    _steps_per_world_unit,
                    1000,
                    false,
                    Some(&mut homing_position),
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);

                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-probe")] {
                        self.probe_controller.lock().await.probe_pin_down(100).await;
                    }
                }

                // FIXME: As of now, until tested in real hardware, for safety
                let zbounds = homing_position.z.unwrap_or(math::ZERO)
                    + _machine_bounds.z.unwrap_or(math::ONE) / Real::from_lit(16, 0);
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Lower Z up to {:?} mm", zbounds); // reduced for safety
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Z, Some(math::ONE.neg())),
                    zbounds,
                    _steps_per_world_unit,
                    1000,
                    true,
                    None,
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at: {:?}", homing_position);

                cfg_if::cfg_if! {
                    if #[cfg(feature = "with-probe")] {
                        self.probe_controller.lock().await.probe_pin_up(100).await;
                    }
                }

                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Upper Z + 10mm again por safety");
                self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Z, Some(math::ONE)),
                    Real::from_lit(10, 0),
                    _steps_per_world_unit,
                    2000,
                    false,
                    Some(&mut homing_position),
                )
                .await;
            }
        }

        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] [Homing] Done. Finally at: {:?}",
            homing_position
        );
        #[cfg(feature = "trace-commands")]
        hwa::info!(
            "[trace-commands] Current position expecting to be {:?}",
            Contract::DEFAULT_WORLD_HOMING_POINT_WU
        );

        Ok(Contract::DEFAULT_WORLD_HOMING_POINT_WU)
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
            .map_values(|_, v| v.to_i32().and_then(|vi| Some(vi as u32)));
        let mut steps_advanced: TVector<u32> = TVector::zero();

        self.enable_and_set_dir(&vdir);

        let mut channel = hwa::CoordSel::empty();
        let mut coord_sel = CoordSel::empty();
        steps_to_advance.foreach_values(|coord, v| {
            let applied = v.is_defined_positive();
            channel.set(coord.into(), applied);
            coord_sel.set(coord, applied);
        });

        let mut ticker = embassy_time::Ticker::every(Duration::from_hz(step_frequency));

        loop {
            let mut completed = false;
            if check_endstops && self.step_actuator().end_stop_triggered(channel) {
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
                            * steps_advanced.map_values(|_, c| Some(Real::from_lit(c.into(), 0)))
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
                    self.step_actuator().step_low(channel);

                }
                else {
                    self.step_actuator().step_toggle(channel);
                }
            }
            steps_advanced.increment(coord_sel, 1);
            ticker.next().await;
        }
    }
}
