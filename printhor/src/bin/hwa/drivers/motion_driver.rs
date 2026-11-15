//! A composition of controllers to 'drive' motion logic
use crate::hwa;
#[allow(unused)]
use core::ops::Neg;
use hwa::Contract;
#[allow(unused)]
use hwa::HwiContract;
#[allow(unused)]
#[cfg(feature = "with-probe")]
use hwa::controllers::ProbeTrait;
use hwa::math;
#[allow(unused_imports)]
use math::ArithmeticOps;
use math::{CoordSel, Real, TVector};

/// Time for the probe pin to deploy or stow before moving (a BLTouch needs a few hundred ms)
#[cfg(feature = "with-probe")]
pub const PROBE_SETTLE_US: u64 = 500_000;

/// Time for a BLTouch to decode the SW mode (latched output) command after the deploy
#[cfg(feature = "with-probe")]
pub const PROBE_SW_MODE_SETTLE_US: u64 = 200_000;

// ── Homing configuration (world units, mm), as Marlin's. Overridable at build time ─────────

/// Z raise before homing X and Y (Marlin `Z_HOMING_HEIGHT`)
#[const_env::env_item("HOMING_Z_RAISE_MM")]
const HOMING_Z_RAISE_MM: f32 = 10.0;

/// Z raise before deploying the probe to take the Z sample (Marlin `Z_CLEARANCE_DEPLOY_PROBE`)
#[const_env::env_item("PROBE_DEPLOY_CLEARANCE_MM")]
const PROBE_DEPLOY_CLEARANCE_MM: f32 = 10.0;

/// Minimum Z travel searching the bed during the homing (the machine Z size if larger)
#[const_env::env_item("HOMING_Z_MAX_TRAVEL_MM")]
const HOMING_Z_MAX_TRAVEL_MM: f32 = 100.0;

/// Back-off before the slow approach of X and Y (Marlin `HOMING_BUMP_MM`)
#[const_env::env_item("HOMING_XY_BUMP_MM")]
const HOMING_XY_BUMP_MM: f32 = 5.0;

/// Back-off before the slow (fine) Z approach (Marlin `Z_HOME_BUMP_MM`)
#[const_env::env_item("HOMING_Z_BUMP_MM")]
const HOMING_Z_BUMP_MM: f32 = 2.0;

/// Height the homing leaves Z above the plane where the Z end-stop (or the probe) triggered
/// (Marlin `Z_AFTER_HOMING`)
#[const_env::env_item("Z_AFTER_HOMING_MM")]
const Z_AFTER_HOMING_MM: f32 = 10.0;

/// Speed divisor of the slow (fine) approach of X and Y, from their homing feed rate (Marlin
/// `HOMING_BUMP_DIVISOR`). Only with a homing feed rate ([HwiContract::HOMING_FEED_RATE_PS])
#[cfg(any(feature = "with-x-axis", feature = "with-y-axis"))]
const HOMING_BUMP_DIVISOR_XY: f32 = 2.0;

/// Speed divisor of the slow (fine) approach of Z, from its homing feed rate (Marlin
/// `HOMING_BUMP_DIVISOR`). Only with a homing feed rate ([HwiContract::HOMING_FEED_RATE_PS])
#[cfg(feature = "with-z-axis")]
const HOMING_BUMP_DIVISOR_Z: f32 = 4.0;

/// Step rates (steps/s) of the homing moves when the axes have no homing feed rate
/// ([HwiContract::HOMING_FEED_RATE_PS]): fast approach and travel, and slow approaches
const HOMING_FAST_STEP_FREQUENCY: u64 = 2000;
const HOMING_SLOW_XY_STEP_FREQUENCY: u64 = 1000;
#[cfg(feature = "with-z-axis")]
const HOMING_FAST_Z_STEP_FREQUENCY: u64 = 1000;
/// Marlin's `HOMING_BUMP_DIVISOR` of Z (4)
#[cfg(feature = "with-z-axis")]
const HOMING_SLOW_Z_STEP_FREQUENCY: u64 = 250;

/// Height (world units) the homing leaves Z above the plane where the Z end-stop (or the probe)
/// triggered (see `Z_AFTER_HOMING_MM`)
pub fn homing_z_clearance() -> Real {
    Real::from_f32(Z_AFTER_HOMING_MM)
}

/// Step rate of the raise after probing
#[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
const PROBE_RAISE_STEP_FREQUENCY: u64 = 2000;

/// Why a single probe (`G30`) failed
#[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProbeError {
    /// The probe did not trigger within the allowed descent (a safeguard: probe failure, or no
    /// bed below)
    NotTriggered,
    /// Interrupted on purpose (`M410`/`M112`)
    Aborted,
}

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

    /// Homing (`G28`), as Marlin with `Z_SAFE_HOMING` (and the probe as Z end-stop):
    ///
    /// 1. Z is raised `HOMING_Z_RAISE_MM` (the position is still unknown: relative, no end-stop).
    /// 2. X, then Y: fast approach to the end-stop, back-off `HOMING_XY_BUMP_MM`, slow approach.
    /// 3. X and Y move together (straight line) to the safe homing point: the bed center minus the
    ///    probe XY offset, so that the *probe* is over the center (see
    ///    [MotionConfig::get_native_post_homing_position]).
    /// 4. Z sample: raise `PROBE_DEPLOY_CLEARANCE_MM`, deploy the probe, fast descent (at most the
    ///    larger of `HOMING_Z_MAX_TRAVEL_MM` and the machine Z size) until it triggers, back-off
    ///    `HOMING_Z_BUMP_MM` and slow (fine) approach. The probe is stowed. There, the nozzle is
    ///    at `-probe_offset.z` above the bed plane.
    /// 5. Z is raised `Z_AFTER_HOMING_MM`.
    ///
    /// Returns the native (machine) position reached, or the displacement done when it failed.
    pub async fn homing_action(
        &mut self,
        motion_config: &hwa::controllers::MotionConfig,
    ) -> Result<TVector<Real>, TVector<Real>> {
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing]");

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion-broadcast")] {
                // Best effort: the broadcast consumers only lose the reset marker
                if self.step_actuator.broadcast_channel.try_send(hwa::MotionBroadcastEvent::Reset).is_err() {
                    hwa::warn!("[Homing] Motion broadcast full: Reset event dropped");
                }
            }
        }

        #[allow(unused_mut)]
        let mut displacement = TVector::zero();
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
        let _machine_bounds = motion_config.get_machine_bounds();
        #[allow(unused)]
        let target = motion_config.get_native_post_homing_position();

        // 1. Raise Z before homing X and Y
        #[cfg(feature = "with-z-axis")]
        {
            #[cfg(feature = "trace-commands")]
            hwa::info!("[trace-commands] [Homing] Raise Z");
            let up = TVector::new().with_coord(CoordSel::Z, Some(math::ONE));
            homing_ok &= self
                .shabbily_move_to(
                    up,
                    Real::from_f32(HOMING_Z_RAISE_MM),
                    _steps_per_world_unit,
                    Self::homing_rate(&up, &_steps_per_world_unit, 1.0, HOMING_FAST_STEP_FREQUENCY),
                    false,
                    Some(&mut displacement),
                )
                .await;
        }

        // 2. X, then Y
        #[cfg(feature = "with-x-axis")]
        {
            homing_ok &= self
                .home_axis(CoordSel::X, _machine_bounds.x.unwrap_or(math::ZERO), _steps_per_world_unit)
                .await;
        }
        #[cfg(feature = "with-y-axis")]
        {
            homing_ok &= self
                .home_axis(CoordSel::Y, _machine_bounds.y.unwrap_or(math::ZERO), _steps_per_world_unit)
                .await;
        }

        // 3. X and Y together to the safe homing point
        #[cfg(any(feature = "with-x-axis", feature = "with-y-axis"))]
        if homing_ok {
            let homing_point = Contract::DEFAULT_WORLD_HOMING_POINT_WU;
            let mut delta: TVector<Real> = TVector::new();
            for coord in [CoordSel::X, CoordSel::Y] {
                if let (Some(t), Some(h)) = (target.get_coord(coord), homing_point.get_coord(coord)) {
                    delta.set_coord(coord, Some(t - h));
                }
            }
            if let Some(module) = delta.norm2().filter(|m| m.is_defined_positive()) {
                #[cfg(feature = "trace-commands")]
                hwa::info!("[trace-commands] [Homing] To the safe homing point: {:?}", delta);
                let vdir = delta / module;
                homing_ok &= self
                    .shabbily_move_to(
                        vdir,
                        module,
                        _steps_per_world_unit,
                        Self::homing_rate(&vdir, &_steps_per_world_unit, 1.0, HOMING_FAST_STEP_FREQUENCY),
                        false,
                        None,
                    )
                    .await;
            }
        }

        // 4. Z sample (probe or end-stop), with the fine approach
        #[cfg(feature = "with-z-axis")]
        if homing_ok {
            #[cfg(feature = "with-probe")]
            {
                let up = TVector::new().with_coord(CoordSel::Z, Some(math::ONE));
                homing_ok &= self
                    .shabbily_move_to(
                        up,
                        Real::from_f32(PROBE_DEPLOY_CLEARANCE_MM),
                        _steps_per_world_unit,
                        Self::homing_rate(&up, &_steps_per_world_unit, 1.0, HOMING_FAST_STEP_FREQUENCY),
                        false,
                        None,
                    )
                    .await;
                self.probe_controller
                    .lock()
                    .await
                    .probe_deploy_latched(PROBE_SETTLE_US, PROBE_SW_MODE_SETTLE_US)
                    .await;
            }
            let max_travel = Real::from_f32(HOMING_Z_MAX_TRAVEL_MM)
                .max(_machine_bounds.z.unwrap_or(math::ZERO)
                    + Real::from_f32(HOMING_Z_RAISE_MM + PROBE_DEPLOY_CLEARANCE_MM));
            #[cfg(feature = "trace-commands")]
            hwa::info!("[trace-commands] [Homing] Z sample (max travel {:?})", max_travel);
            homing_ok &= self
                .touch_z(max_travel, _steps_per_world_unit, None)
                .await;
            #[cfg(feature = "with-probe")]
            self.probe_controller
                .lock()
                .await
                .probe_pin_up(PROBE_SETTLE_US)
                .await;
            // 5. Raise Z
            if homing_ok {
                let up = TVector::new().with_coord(CoordSel::Z, Some(math::ONE));
                homing_ok &= self
                    .shabbily_move_to(
                        up,
                        homing_z_clearance(),
                        _steps_per_world_unit,
                        Self::homing_rate(&up, &_steps_per_world_unit, 1.0, HOMING_FAST_STEP_FREQUENCY),
                        false,
                        None,
                    )
                    .await;
            }
        }

        if homing_ok {
            #[cfg(feature = "trace-commands")]
            hwa::info!("[trace-commands] [Homing] Done at native {:?}", target);
            Ok(target)
        } else {
            hwa::error!("[Homing] End-stop not triggered within the axis travel");
            Err(displacement)
        }
    }

    /// Homes one axis towards its minimum (as Marlin): fast approach to the end-stop within the
    /// axis travel, back-off `HOMING_XY_BUMP_MM` and slow approach. `false` when it did not trigger
    #[cfg(any(feature = "with-x-axis", feature = "with-y-axis"))]
    async fn home_axis(&mut self, coord: CoordSel, travel: Real, steps_per_world_unit: TVector<Real>) -> bool {
        let bump = Real::from_f32(HOMING_XY_BUMP_MM);
        #[cfg(feature = "trace-commands")]
        hwa::info!("[trace-commands] [Homing] {:?} (travel {:?})", coord, travel);
        let toward = TVector::new().with_coord(coord, Some(math::ONE.neg()));
        let away = TVector::new().with_coord(coord, Some(math::ONE));
        let fast = Self::homing_rate(&toward, &steps_per_world_unit, 1.0, HOMING_FAST_STEP_FREQUENCY);
        let slow = Self::homing_rate(&toward, &steps_per_world_unit, HOMING_BUMP_DIVISOR_XY, HOMING_SLOW_XY_STEP_FREQUENCY);
        self.shabbily_move_to(toward, travel + bump, steps_per_world_unit, fast, true, None)
            .await
            && self
                .shabbily_move_to(away, bump, steps_per_world_unit, fast, false, None)
                .await
            && self
                .shabbily_move_to(
                    toward,
                    bump + bump,
                    steps_per_world_unit,
                    slow,
                    true,
                    None,
                )
                .await
    }

    /// Z sample with the fine approach (as Marlin's double touch): fast descent of at most
    /// `max_travel` until the Z end-stop (or the probe) triggers, back-off `HOMING_Z_BUMP_MM` and
    /// slow descent (at most twice the back-off). `displacement` accumulates the Z moved (world
    /// units). `false` when it did not trigger.
    #[cfg(feature = "with-z-axis")]
    async fn touch_z(
        &mut self,
        max_travel: Real,
        steps_per_world_unit: TVector<Real>,
        mut displacement: Option<&mut TVector<Real>>,
    ) -> bool {
        let down = TVector::new().with_coord(CoordSel::Z, Some(math::ONE.neg()));
        let up = TVector::new().with_coord(CoordSel::Z, Some(math::ONE));
        let bump = Real::from_f32(HOMING_Z_BUMP_MM);
        let descent = Self::homing_rate(&down, &steps_per_world_unit, 1.0, HOMING_FAST_Z_STEP_FREQUENCY);
        let back_off = Self::homing_rate(&up, &steps_per_world_unit, 1.0, HOMING_FAST_STEP_FREQUENCY);
        let slow = Self::homing_rate(&down, &steps_per_world_unit, HOMING_BUMP_DIVISOR_Z, HOMING_SLOW_Z_STEP_FREQUENCY);
        self.shabbily_move_to(down, max_travel, steps_per_world_unit, descent, true, displacement.as_deref_mut())
            .await
            && self
                .shabbily_move_to(up, bump, steps_per_world_unit, back_off, false, displacement.as_deref_mut())
                .await
            && self
                .shabbily_move_to(
                    down,
                    bump + bump,
                    steps_per_world_unit,
                    slow,
                    true,
                    displacement.as_deref_mut(),
                )
                .await
    }

    /// Single probe (`G30`) at the current XY: deploys the probe, lowers Z until it triggers
    /// (fast, then the fine approach, as the homing), raises Z back to the start height and stows
    /// the probe. So the position is unchanged afterwards (also on failure).
    ///
    /// `max_descent` (world units) bounds the travel. Returns the descent until the trigger.
    ///
    /// The motion queue must be idle: the steps are generated here, as in the homing.
    #[cfg(all(feature = "with-probe", feature = "with-z-axis"))]
    pub async fn probe_action(
        &mut self,
        motion_config: &hwa::controllers::MotionConfig,
        max_descent: Real,
    ) -> Result<Real, ProbeError> {
        let steps_per_world_unit = motion_config.get_steps_per_space_unit_as_vector();
        self.probe_controller
            .lock()
            .await
            .probe_deploy_latched(PROBE_SETTLE_US, PROBE_SW_MODE_SETTLE_US)
            .await;
        let mut displacement = TVector::zero();
        let triggered = self
            .touch_z(max_descent, steps_per_world_unit, Some(&mut displacement))
            .await;
        let aborted = crate::tasks::task_stepper::motion_aborted();
        let descent = displacement.z.unwrap_or(math::ZERO).neg();
        // Back to the start height, also after a failure (the position stays known). Not when
        // aborted: the abort stops every motion
        if !aborted && descent > math::ZERO {
            let _ = self
                .shabbily_move_to(
                    TVector::new().with_coord(CoordSel::Z, Some(math::ONE)),
                    descent,
                    steps_per_world_unit,
                    PROBE_RAISE_STEP_FREQUENCY,
                    false,
                    None,
                )
                .await;
        }
        self.probe_controller
            .lock()
            .await
            .probe_pin_up(PROBE_SETTLE_US)
            .await;
        match (triggered, aborted) {
            (_, true) => Err(ProbeError::Aborted),
            (true, false) => Ok(descent),
            (false, false) => Err(ProbeError::NotTriggered),
        }
    }

    /// Translates a direction in world coordinates to motor (space) coordinates.
    ///
    /// Identity for cartesian machines. For Core-XY, a move along X (or Y) requires both motors.
    #[cfg_attr(not(any(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis")), allow(dead_code))]
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

    /// The step rate (steps per second of the motor with the most steps, as
    /// [Self::shabbily_move_to] takes it) of a homing move along `vdir` (a world direction): the
    /// fastest at which no world axis of the move exceeds its homing feed rate
    /// ([HwiContract::HOMING_FEED_RATE_PS], world units/s) divided by `divisor` (1 for the fast
    /// moves, the bump divisor for the slow approach). `fallback` (steps/s) when an axis of the
    /// move has no feed rate.
    #[cfg_attr(not(any(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis")), allow(dead_code))]
    fn homing_rate(vdir: &TVector<Real>, steps_per_mm: &TVector<Real>, divisor: f32, fallback: u64) -> u64 {
        Self::homing_step_frequency(&Contract::HOMING_FEED_RATE_PS, vdir, steps_per_mm, Real::from_f32(divisor), fallback)
    }

    /// [Self::homing_rate] with the given homing feed rates (world units/s)
    #[cfg_attr(not(any(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis")), allow(dead_code))]
    fn homing_step_frequency(
        feed_rates: &TVector<Real>,
        vdir: &TVector<Real>,
        steps_per_mm: &TVector<Real>,
        divisor: Real,
        fallback: u64,
    ) -> u64 {
        // Speed along the move (world units/s), set by its most limiting axis
        let mut speed: Option<Real> = None;
        let mut every_axis_has_rate = true;
        vdir.foreach_values(|coord, v| {
            if v.is_zero() {
                return;
            }
            match feed_rates.get_coord(coord).filter(|f| f.is_defined_positive()) {
                Some(feed) => {
                    let s = feed / v.abs();
                    speed = Some(speed.map_or(s, |m| m.min(s)));
                }
                None => every_axis_has_rate = false,
            }
        });
        let speed = match speed {
            Some(s) if every_axis_has_rate && divisor.is_defined_positive() => s / divisor,
            _ => return fallback,
        };
        // Steps per world unit along the move of the motor with the most steps
        let mut major = math::ZERO;
        Self::world_to_motor_direction(vdir).foreach_values(|coord, v| {
            if let Some(spm) = steps_per_mm.get_coord(coord) {
                major = major.max((*v * spm).abs());
            }
        });
        match (speed * major).round().to_i64() {
            Some(f) if f > 0 => f as u64,
            _ => fallback,
        }
    }

    /// A move of `module` world units along `vdir` (a world direction) at `step_frequency` steps
    /// per second of the axis with the most steps, executed by the step executor (ISR) as the
    /// regular motion: a sequence of step windows (see [hwa::controllers::StepPlan::watched]).
    ///
    /// When `check_endstops`, the end-stops of the world axes of the move are checked by the ISR
    /// on every tick before stepping: the move stops at the first tick an end-stop is found
    /// triggered (latency: one tick, e.g. 20 µs at 50 kHz), whatever the load of the async tasks.
    /// `position` accumulates the displacement done (world units). `false` when the move was
    /// aborted (M410/M112) or, checking the end-stops, none triggered within the travel.
    #[cfg_attr(not(any(feature = "with-x-axis", feature = "with-y-axis", feature = "with-z-axis")), allow(dead_code))]
    async fn shabbily_move_to(
        &mut self,
        vdir: TVector<Real>,
        module: Real,
        steps_per_mm: TVector<Real>,
        step_frequency: u64,
        check_endstops: bool,
        position: Option<&mut TVector<Real>>,
    ) -> bool {
        use crate::tasks::task_stepper::{
            STEPPER_PLANNER_CLOCK_PERIOD_US, STEPPER_PLANNER_MICROSEGMENT_PERIOD_US, motion_aborted,
        };
        use hwa::controllers::motion_control::{
            MultiTimer, STEP_DRIVER, StepPlan, pause_ticker, resume_ticker, ticker_running,
        };

        // The end-stops to check are the ones of the world axis of the move
        let mut endstop_channel = hwa::CoordSel::empty();
        vdir.foreach_values(|coord, v| endstop_channel.set(coord, !v.is_zero()));
        let endstops = match check_endstops {
            true => endstop_channel,
            false => hwa::CoordSel::empty(),
        };
        // The direction is given in world coordinates: translate it to motors
        let vdir = Self::world_to_motor_direction(&vdir);

        let steps_to_advance: TVector<u32> = (vdir * module * steps_per_mm)
            .abs()
            .round()
            .map_values(|_, v| v.to_i32().and_then(|vi| Some(vi as u32)));
        // Multi-axis moves (e.g. X and Y together) step every axis proportionally: the axis with
        // the most steps sets the pace, so all of them start and end together (straight line)
        let mut n_major: u32 = 0;
        steps_to_advance.foreach_values(|_, v| n_major = n_major.max(*v));
        let mut dir_fwd = hwa::CoordSel::empty();
        vdir.foreach_values(|coord, v| dir_fwd.set(coord, v.is_defined_positive()));

        // After the motion still queued in the step executor (e.g. the moves before a G28)
        STEP_DRIVER.flush().await;

        if n_major == 0 {
            let triggered = check_endstops && self.step_actuator().end_stop_triggered(endstops);
            return !motion_aborted() && (!check_endstops || triggered);
        }

        // The ticker may be paused (steppers parked, e.g. a G30 after some idle time): it runs
        // for this move only
        let ticker_was_running = ticker_running();
        if !ticker_was_running {
            resume_ticker();
        }
        // The (all enabled) drivers and the directions of the first window are applied, whatever
        // was applied before
        STEP_DRIVER.reset();
        STEP_DRIVER.begin_watch();

        // Simulators may speed the homing moves up (the same steps and end-stop checks, faster)
        let step_frequency = (step_frequency * Contract::HOMING_STEP_FREQUENCY_SCALE).max(1);
        let total_us: u64 = (n_major as u64 * 1_000_000 / step_frequency).max(1);
        let window_us = (STEPPER_PLANNER_MICROSEGMENT_PERIOD_US as u64).max(1);
        // Steps scheduled so far, per motor
        let mut scheduled: TVector<u32> = TVector::zero();
        let mut t_us: u64 = 0;
        while t_us < total_us {
            // No more windows once an end-stop triggered (the ISR drops the queued ones)
            if STEP_DRIVER.watch_triggered() || motion_aborted() {
                break;
            }
            let t_end = (t_us + window_us).min(total_us);
            let width_us = (t_end - t_us) as u32;
            let mut multi_timer = MultiTimer::new();
            multi_timer.set_displace_width(width_us);
            let mut counts: TVector<u32> = TVector::new();
            steps_to_advance.foreach_values(|coord, n| {
                // Steps due by the end of the window (rounded), as a uniform rate along the move
                let due = ((*n as u64 * t_end + total_us / 2) / total_us) as u32;
                let done = scheduled.get_coord(coord).unwrap_or(0);
                let count = due.saturating_sub(done);
                if count > 0 {
                    counts.set_coord(coord, Some(count));
                    multi_timer.set_channel_uniform(
                        coord,
                        Some(width_us / count),
                        STEPPER_PLANNER_CLOCK_PERIOD_US,
                    );
                    scheduled.set_coord(coord, Some(done + count));
                }
            });
            multi_timer.set_max_count(&counts);
            STEP_DRIVER
                .push_plan(StepPlan::watched(
                    0,
                    multi_timer,
                    hwa::CoordSel::all_axis(),
                    dir_fwd,
                    endstops,
                ))
                .await;
            t_us = t_end;
        }
        STEP_DRIVER.flush().await;
        let (endstop_triggered, steps_advanced) = STEP_DRIVER.end_watch();
        if !ticker_was_running {
            pause_ticker();
        }
        let aborted = motion_aborted();
        if aborted {
            hwa::warn!("[motion_driver] Move interrupted (motion aborted)");
        }

        match position {
            Some(mut _p) => {
                // Steps advanced per axis, in the direction of each axis
                *_p += vdir.sign()
                    * steps_advanced.map_values(|_, c| Some(Real::from_lit(c.into(), 0)))
                    / steps_per_mm;
            }
            None => {}
        }

        // When checking end-stops, reaching the full travel means the end-stop was not found
        !aborted && (!check_endstops || endstop_triggered)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(all(
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis",
        not(feature = "with-motion-core-xy-kinematics")
    ))]
    #[test]
    fn homing_step_frequency_from_feed_rates() {
        // Marlin's HOMING_FEEDRATE_MM_M (51, 51, 5 mm/s) with 80/80/400 steps/mm
        let feeds = hwa::make_vector_real!(x = 51.0, y = 51.0, z = 5.0);
        let spm = hwa::make_vector_real!(x = 80.0, y = 80.0, z = 400.0);
        let rate = |feeds: &TVector<Real>, vdir: &TVector<Real>, divisor: f32| {
            MotionDriver::homing_step_frequency(feeds, vdir, &spm, Real::from_f32(divisor), 7)
        };
        let x = TVector::new().with_coord(CoordSel::X, Some(math::ONE.neg()));
        let z = TVector::new().with_coord(CoordSel::Z, Some(math::ONE));
        // Fast approach, and the slow one (bump divisors 2 and 4)
        assert_eq!(rate(&feeds, &x, 1.0), 4080);
        assert_eq!(rate(&feeds, &x, HOMING_BUMP_DIVISOR_XY), 2040);
        assert_eq!(rate(&feeds, &z, 1.0), 2000);
        assert_eq!(rate(&feeds, &z, HOMING_BUMP_DIVISOR_Z), 500);
        // X and Y together: the most limiting axis (Y, 0.8 · 63.75 = 51 mm/s) sets the pace of
        // the motor with the most steps (Y, 64 steps per unit along the move)
        let diagonal = hwa::make_vector_real!(x = 0.6, y = 0.8);
        assert_eq!(rate(&feeds, &diagonal, 1.0), 4080);
        // An axis of the move without a feed rate: the fixed step rate
        assert_eq!(rate(&hwa::make_vector_real!(x = 51.0, y = 51.0), &z, 1.0), 7);
        assert_eq!(rate(&TVector::new(), &x, 1.0), 7);
    }
}
