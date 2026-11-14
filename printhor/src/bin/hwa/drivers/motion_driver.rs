//! A composition of controllers to 'drive' motion logic
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
use math::{ArithmeticOps, CoordSel, Real, TVector};

/// Time for the probe pin to deploy or stow before moving (a BLTouch needs a few hundred ms)
#[cfg(feature = "with-probe")]
pub const PROBE_SETTLE_US: u64 = 500_000;

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
    /// * `motion_config` - A reference to the motion configuration which contains
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
        #[allow(unused_mut)]
        let mut homing_ok = true;

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
                homing_ok &= self.shabbily_move_to(
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
                homing_ok &= self.shabbily_move_to(
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
                homing_ok &= self.shabbily_move_to(
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
                homing_ok &= self.shabbily_move_to(
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
                homing_ok &= self.shabbily_move_to(
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
                homing_ok &= self.shabbily_move_to(
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
                // Home the Y axis precisely
                homing_ok &= self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Y, Some(math::ONE.neg())),
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
                let x_center = _machine_bounds.x.unwrap_or(math::ZERO) / Real::from_lit(2, 0);
                let y_center = _machine_bounds.y.unwrap_or(math::ZERO) / Real::from_lit(2, 0);
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Move to center X={:?} Y={:?}", x_center, y_center);
                homing_ok &= self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::X, Some(math::ONE)),
                    x_center,
                    _steps_per_world_unit,
                    2000,
                    false,
                    Some(&mut homing_position),
                )
                .await;
                homing_ok &= self.shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Y, Some(math::ONE)),
                    y_center,
                    _steps_per_world_unit,
                    2000,
                    false,
                    Some(&mut homing_position),
                )
                .await;
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] - Now at center: {:?}", homing_position);
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-z-axis")] {
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Upper Z + 10mm");
                homing_ok &= self.shabbily_move_to(
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
                        self.probe_controller.lock().await.probe_pin_down(PROBE_SETTLE_US).await;
                    }
                }

                let zbounds = homing_position.z.unwrap_or(math::ZERO)
                    + _machine_bounds.z.unwrap_or(math::ONE);
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Lower Z up to {:?} mm", zbounds);
                homing_ok &= self.shabbily_move_to(
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
                        self.probe_controller.lock().await.probe_pin_up(PROBE_SETTLE_US).await;
                    }
                }

                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] Upper Z + 10mm again por safety");
                homing_ok &= self.shabbily_move_to(
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

        if homing_ok {
            Ok(Contract::DEFAULT_WORLD_HOMING_POINT_WU)
        } else {
            hwa::error!("[Homing] End-stop not triggered within the axis travel");
            Err(homing_position)
        }
    }
    /// Translates a direction in world coordinates to motor (space) coordinates.
    ///
    /// Identity for cartesian machines. For Core-XY, a move along X (or Y) requires both motors.
    fn world_to_motor_direction(vdir: &TVector<Real>) -> TVector<Real> {
        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion-core-xy-kinematics")] {
                use hwa::HwiContract;
                let x = vdir.get_coord(CoordSel::X);
                let y = vdir.get_coord(CoordSel::Y);
                if x.is_none() && y.is_none() {
                    return *vdir;
                }
                let filled = vdir
                    .with_coord(CoordSel::X, Some(x.unwrap_or(math::ZERO)))
                    .with_coord(CoordSel::Y, Some(y.unwrap_or(math::ZERO)));
                match hwa::Contract.project_to_space(&filled) {
                    Ok(space) => space.map_values(|_, v| (!v.is_zero()).then_some(v)),
                    Err(_) => *vdir,
                }
            }
            else {
                *vdir
            }
        }
    }

    async fn shabbily_move_to(
        &mut self,
        vdir: TVector<Real>,
        module: Real,
        steps_per_mm: TVector<Real>,
        step_frequency: u64,
        check_endstops: bool,
        position: Option<&mut TVector<Real>>,
    ) -> bool {
        // The end-stops to check are the ones of the world axis of the move
        let mut endstop_channel = hwa::CoordSel::empty();
        vdir.foreach_values(|coord, v| endstop_channel.set(coord, !v.is_zero()));
        // The direction is given in world coordinates: translate it to motors
        let vdir = Self::world_to_motor_direction(&vdir);

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

        // Simulators may speed the homing moves up (the same steps and end-stop checks, faster)
        let step_frequency = step_frequency * Contract::HOMING_STEP_FREQUENCY_SCALE;
        // Not a Ticker: after a late wake-up (other tasks hogging the CPU) a Ticker fires the
        // missed ticks back to back, a burst of steps the motor can not follow. Here, a late step
        // just restarts the schedule from now.
        let step_period = Duration::from_hz(step_frequency);
        let mut next_step = embassy_time::Instant::now() + step_period;

        let mut endstop_triggered = false;
        #[cfg(feature = "debug-homing-latency")]
        let (mut max_late_us, mut late_count, t_start) =
            (0u64, 0u32, embassy_time::Instant::now());
        loop {
            let mut completed = false;
            if check_endstops && self.step_actuator().end_stop_triggered(endstop_channel) {
                hwa::debug!("ENDSTOP TRIGGERED");
                endstop_triggered = true;
                completed = true;
            }
            if !steps_advanced.bounded_by(&steps_to_advance) {
                hwa::trace!("FULL ADV");
                completed = true;
            }
            // M410/M112: interrupt the move. Treated as a failure, so the position stays unknown
            let aborted = crate::tasks::task_stepper::motion_aborted();
            if aborted {
                hwa::warn!("[motion_driver] Move interrupted (motion aborted)");
                completed = true;
            }
            if completed {
                #[cfg(feature = "debug-homing-latency")]
                hwa::info!(
                    "[homing-latency] move done: max_late = {} us, late(>{}us) = {}, steps = {:?}",
                    max_late_us,
                    step_period.as_micros() / 2,
                    late_count,
                    steps_advanced
                );
                match position {
                    Some(mut _p) => {
                        *_p += vdir
                            * steps_advanced.map_values(|_, c| Some(Real::from_lit(c.into(), 0)))
                            / steps_per_mm;
                    }
                    None => {}
                }

                // When checking end-stops, reaching the full travel means the end-stop was not found
                return !aborted && (!check_endstops || endstop_triggered);
            }
            cfg_if::cfg_if! {
                if #[cfg(feature="pulsed")] {
                    compile_error!("Not yet implemented");
                    self.ping().step_high(channel);
                    // TODO
                    //crate::tasks::motion_timing::s_block_for(Duration::from_micros(1));
                    self.step_actuator().step_low(channel);

                }
                else {
                    self.step_actuator().step_toggle(channel);
                }
            }
            steps_advanced.increment(coord_sel, 1);
            let now = embassy_time::Instant::now();
            #[cfg(feature = "debug-homing-latency")]
            {
                // `next_step - step_period` is when this step was due
                let due = next_step - step_period;
                if now > due {
                    let late = (now - due).as_micros();
                    max_late_us = max_late_us.max(late);
                    if late > step_period.as_micros() / 2 {
                        late_count += 1;
                        hwa::warn!(
                            "[homing-latency] step late by {} us at t = {} ms",
                            late,
                            (now - t_start).as_millis()
                        );
                    }
                }
            }
            if now >= next_step {
                next_step = now + step_period;
            }
            embassy_time::Timer::at(next_step).await;
            next_step += step_period;
        }
    }
}
