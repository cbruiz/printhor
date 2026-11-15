//! Motion Configuration
use crate::hwa;
#[allow(unused)]
use hwa::Contract;
#[allow(unused)]
use hwa::HwiContract;
use hwa::SyncMutexStrategy;
use hwa::math::{CoordSel, Real, TVector};

/// Configuration structure for motion parameters.
///
/// This structure holds various configuration parameters for controlling motion in a system.
/// It includes maximum acceleration, speed, jerk, travel speed, units per space magnitude,
/// machine bounds, micro-stepping values, flow rate, and speed rate.
///
/// # Fields
///
/// * `min_speed_su` - A `TVector` representing the minimum speed for the motion in `space_units`/s that discrete a single step does
/// * `max_speed_su` - A `TVector` representing the maximum speed for the motion in `space_units`/s that the hardware can provide
/// * `max_accel_su` - A `TVector` representing the maximum acceleration for the motion in `space_units`/s^2
/// * `max_jerk_su` - A `TVector` representing the maximum jerk for the motion in `space_units`/s^3
/// * `default_travel_speed` - The default travel speed in `space_units`/s.
/// * `units_per_space_magnitude` - A `TVector` representing the scale of `space_unit` by `space_magnitude`, not considering micro-stepping.
/// * `micro_steps_per_axis` - A `TVector` representing the micro-stepping scale for each axis.
/// * `world_center_wu` - A `TVector` representing the machine's world center.
/// * `world_size_wu` - A `TVector` representing the machine's world size.
/// * `nozzle_offset_wu` - The machine's nozzle offset in world units respect to homing end-stops or logical 0
/// * `probe_offset_wu` - The machine's nozzle offset in world units respect to homing end-stops or logical 0
/// * `flow_rate` - The flow rate for the motion, represented as a percentage.
/// * `speed_rate` - The speed rate for the motion, represented as a percentage.
/// # Example
///
/// ```rust
/// use printhor_hwa_common as hwa;
///
/// type MotionConfigMutexType = hwa::NoopRawMutexType;
///
/// let motion_config = hwa::make_static_controller!(
///     "MotionConfig",
///     MotionConfigMutexType,
///     hwa::controllers::MotionConfig,
///     hwa::controllers::MotionConfig::new()
///  );
/// ```

pub struct MotionConfig {
    cfg: hwa::StaticSyncController<hwa::types::MotionConfigMutexStrategy>,
}

impl MotionConfig {
    pub fn new(cfg: hwa::StaticSyncController<hwa::types::MotionConfigMutexStrategy>) -> Self {
        Self { cfg }
    }

    // Set max acceleration in space_units/s
    pub fn set_max_speed(&self, speed: TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.max_speed_su = speed;
        });
        hwa::info!(
            "[MotionConfig:UserConf] Max speed set to {:#?} {}/s",
            speed,
            Contract::SPACE_UNIT_MAGNITUDE
        );
    }
    /// Set max acceleration in space_units/s^2
    pub fn set_max_accel(&self, accel: TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.max_accel_su.assign(CoordSel::all_axis(), &accel);
        });
        hwa::info!(
            "[MotionConfig:UserConf] Max accel set to {:#?} {}/s^2",
            accel,
            Contract::SPACE_UNIT_MAGNITUDE
        );
    }
    //Set max jerk in space_units/s^3
    pub fn set_max_jerk(&self, jerk: TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.max_jerk_su.assign(CoordSel::all_axis(), &jerk);
        });
        hwa::info!(
            "[MotionConfig:UserConf] Max jerk set to {:#?} {}/s^3",
            jerk,
            Contract::SPACE_UNIT_MAGNITUDE
        );
    }
    pub fn set_flow_rate(&self, rate: u8) {
        self.cfg.apply_mut(|m| {
            m.flow_rate = rate;
        });
    }

    pub fn set_space_units_per_world_unit(&self, upm: TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.units_per_space_magnitude = upm;
        });
        hwa::info!(
            "[MotionConfig:MachineConf] Space units per {} set to {:#?}",
            Contract::SPACE_UNIT_MAGNITUDE,
            upm
        );
    }

    pub fn set_speed_rate(&self, rate: u8) {
        self.cfg.apply_mut(|m| {
            m.speed_rate = rate;
        });
    }

    pub fn set_default_travel_speed(&self, speed: Real) {
        self.cfg.apply_mut(|m| {
            m.default_travel_speed = speed;
        });
        hwa::info!(
            "[MotionConfig:UserConf] Default travel speed set to {:#?} {}/s",
            speed,
            Contract::SPACE_UNIT_MAGNITUDE
        );
    }

    pub fn set_micro_steps_per_axis(&self, micro_steps: TVector<u16>) {
        self.cfg.apply_mut(|m| {
            m.micro_steps_per_axis = micro_steps;
        });
        hwa::info!(
            "[MotionConfig:MachineConf] Micro-steps per space unit set to {:#?}",
            micro_steps
        );
    }

    pub fn set_world_center(&self, center: TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.world_center_wu = center;
        });
        hwa::info!(
            "[MotionConfig:MachineConf] World center (reference zero) set to [{:?}] {}",
            center,
            Contract::WORLD_UNIT_MAGNITUDE,
        );
    }

    pub fn set_world_size(&self, size: TVector<Real>) {
        self.cfg.apply_mut(|m| m.world_size_wu = size);
        hwa::info!(
            "[MotionConfig:MachineConf] World size set to [{:?}] {}",
            size,
            Contract::WORLD_UNIT_MAGNITUDE,
        );
        hwa::info!(
            "[MotionConfig:MachineConf] World bounds updated to: [ [{:?}], [{:?}] ] {}",
            -(size / hwa::math::TWO),
            (size / hwa::math::TWO),
            Contract::WORLD_UNIT_MAGNITUDE,
        );
    }

    pub fn set_nozzle_offset(&self, mut offset: TVector<Real>) {
        offset.set_coord(CoordSel::motion_relevant_axis().complement(), None);
        self.cfg.apply_mut(|m| m.nozzle_offset_wu = offset);
        hwa::info!(
            "[MotionConfig:MachineConf] Nozzle offset updated to world: [{:?}] {}",
            offset,
            Contract::WORLD_UNIT_MAGNITUDE,
        );
    }

    /// The probe position relative to the nozzle (world units)
    pub fn get_probe_offset(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.probe_offset_wu)
    }

    /// Sets the probe offset (M851) of the given axis
    pub fn update_probe_offset(&self, offset: &TVector<Real>) {
        let mut offset = *offset;
        offset.set_coord(CoordSel::motion_relevant_axis().complement(), None);
        self.cfg.apply_mut(|m| {
            m.probe_offset_wu
                .assign_if_set(CoordSel::motion_relevant_axis(), &offset);
        });
    }

    pub fn set_probe_offset(&self, mut offset: TVector<Real>) {
        offset.set_coord(CoordSel::motion_relevant_axis().complement(), None);
        self.cfg.apply_mut(|m| m.probe_offset_wu = offset);
        hwa::info!(
            "[MotionConfig:MachineConf] Probe offset updated to world: [{:?}] {}",
            offset,
            Contract::WORLD_UNIT_MAGNITUDE,
        );
    }

    /// Sets the home offsets (M206) of the given axis (E is ignored)
    pub fn update_home_offset(&self, offset: &TVector<Real>) {
        let mut offset = *offset;
        offset.set_coord(CoordSel::motion_relevant_axis().complement(), None);
        self.cfg.apply_mut(|m| {
            m.home_offset_wu
                .assign_if_set(CoordSel::motion_relevant_axis(), &offset);
        });
    }

    /// The home offsets (M206) in world units
    pub fn get_home_offset(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.home_offset_wu)
    }

    /// The logical position of the machine right after homing: the homing point shifted by the
    /// home offsets (M206)
    pub fn get_logical_homing_point(&self) -> TVector<Real> {
        let offset = self.get_home_offset();
        let mut point = Contract::DEFAULT_WORLD_HOMING_POINT_WU;
        point.apply_values(|coord, v| Some(*v + offset.get_coord(coord).unwrap_or(hwa::math::ZERO)));
        point
    }

    /// The native (machine) position right after the homing (`G28`), as Marlin with
    /// `Z_SAFE_HOMING`: X and Y at the safe homing point (the bed center, minus the probe XY
    /// offset so that the probe is over it, within the machine bounds), and Z
    /// [homing_z_clearance](hwa::drivers::motion_driver::homing_z_clearance) above the plane where
    /// the Z end-stop (or the probe) triggered. With the probe the nozzle is `-probe_offset.z`
    /// above the bed there (Marlin's convention: a negative offset is a probe below the nozzle),
    /// so Z0 is the bed.
    pub fn get_native_post_homing_position(&self) -> TVector<Real> {
        #[allow(unused_mut)]
        let mut point = Contract::DEFAULT_WORLD_HOMING_POINT_WU;
        #[allow(unused)]
        let bounds = self.get_machine_bounds();
        #[allow(unused)]
        let probe = self.get_probe_offset();
        #[allow(unused)]
        let probe_offset = |coord: CoordSel| {
            if cfg!(feature = "with-probe") {
                probe.get_coord(coord).unwrap_or(hwa::math::ZERO)
            } else {
                hwa::math::ZERO
            }
        };
        // A Cartesian head parks over the centre of the bed, a clearance above it. Not a joint space
        // machine (anthropomorphic legs): X/Y/Z are the coordinates of its first leg, whose homing
        // point is the home pose, as for every other leg
        #[cfg(all(
            any(feature = "with-x-axis", feature = "with-y-axis"),
            not(feature = "with-motion-anthropomorphic-kinematics")
        ))]
        for coord in [CoordSel::X, CoordSel::Y] {
            if let (Some(home), Some(size)) = (point.get_coord(coord), bounds.get_coord(coord)) {
                let safe = (size * hwa::math::HALF - probe_offset(coord))
                    .max(hwa::math::ZERO)
                    .min(size);
                point.set_coord(coord, Some(home + safe));
            }
        }
        #[cfg(all(feature = "with-z-axis", not(feature = "with-motion-anthropomorphic-kinematics")))]
        if let Some(z) = point.z {
            point.z = Some(
                z - probe_offset(CoordSel::Z) + hwa::drivers::motion_driver::homing_z_clearance(),
            );
        }
        point
    }

    /// The travel of the axes in logical coordinates `(min, max)`: from the end-stop (the homing
    /// point) to the end of the frame (the machine bounds), shifted by the home offsets (M206), as
    /// Marlin's soft end-stops. Only the axes with both a homing point and a size are set.
    pub fn get_logical_travel(&self) -> (TVector<Real>, TVector<Real>) {
        let offset = self.get_home_offset();
        let size = self.get_machine_bounds();
        let (mut min, mut max) = (TVector::new(), TVector::new());
        Contract::DEFAULT_WORLD_HOMING_POINT_WU.foreach_values(|coord, home| {
            if let Some(len) = size.get_coord(coord) {
                let low = *home + offset.get_coord(coord).unwrap_or(hwa::math::ZERO);
                min.set_coord(coord, Some(low));
                max.set_coord(coord, Some(low + len));
            }
        });
        (min, max)
    }

    /// The logical position of a native (machine) position: shifted by the home offsets (M206)
    pub fn logical_from_native(&self, native: &TVector<Real>) -> TVector<Real> {
        let offset = self.get_home_offset();
        let mut point = *native;
        point.apply_values(|coord, v| Some(*v + offset.get_coord(coord).unwrap_or(hwa::math::ZERO)));
        point
    }

    /// The logical position right after the homing (`G28`): see
    /// [Self::get_native_post_homing_position]
    pub fn get_post_homing_position(&self) -> TVector<Real> {
        self.logical_from_native(&self.get_native_post_homing_position())
    }

    /// The logical Z the nozzle is at when the probe (or the Z end-stop) triggers on the bed plane
    /// of the homing: `-probe_offset.z` above it
    pub fn get_logical_trigger_z(&self) -> Real {
        let z = self.get_logical_homing_point().z.unwrap_or(hwa::math::ZERO);
        if cfg!(feature = "with-probe") {
            z - self.get_probe_offset().z.unwrap_or(hwa::math::ZERO)
        } else {
            z
        }
    }

    pub fn get_units_per_space_magnitude(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.units_per_space_magnitude)
    }

    pub fn get_micro_steps(&self) -> TVector<u16> {
        self.cfg.apply(|m| m.micro_steps_per_axis)
    }

    pub fn get_micro_steps_as_vector(&self) -> TVector<Real> {
        self.get_micro_steps()
            .map_values(|_c, v| Some(Real::from_lit(v.into(), 0)))
    }

    pub fn get_steps_per_space_unit_as_vector(&self) -> TVector<Real> {
        self.get_units_per_space_magnitude() * self.get_micro_steps_as_vector()
    }

    pub fn get_flow_rate(&self) -> u8 {
        self.cfg.apply(|m| m.flow_rate)
    }

    pub fn get_speed_rate(&self) -> u8 {
        self.cfg.apply(|m| m.speed_rate)
    }

    /// Updates the max speed (space units/s) only for the given coordinates (M203)
    pub fn update_max_speed(&self, speed: &TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.max_speed_su.assign_if_set(CoordSel::all_axis(), speed);
        });
    }

    /// Updates the max acceleration (space units/s²) only for the given coordinates (M201)
    pub fn update_max_accel(&self, accel: &TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.max_accel_su.assign_if_set(CoordSel::all_axis(), accel);
        });
    }

    /// Updates the steps per space unit only for the given coordinates (M92).
    ///
    /// The value includes micro-stepping, so the full-step scale is derived from the current micro-steps.
    pub fn update_steps_per_space_unit(&self, steps: &TVector<Real>) {
        let micro_steps = self.get_micro_steps_as_vector();
        let units = steps.map(|coord, value| match value {
            Some(v) if v.is_defined_positive() => match micro_steps.get_coord(coord) {
                Some(ms) if ms.is_defined_positive() => Some(*v / ms),
                _ => None,
            },
            _ => None,
        });
        self.cfg.apply_mut(|m| {
            m.units_per_space_magnitude
                .assign_if_set(CoordSel::all_axis(), &units);
        });
        self.compute_min_speed();
    }

    /// Sets the acceleration limit (space units/s² along the path) for extruding moves (M204 P/S)
    pub fn set_print_accel(&self, accel: Option<Real>) {
        self.cfg.apply_mut(|m| m.print_accel_su = accel);
    }

    /// Sets the acceleration limit (space units/s² along the path) for travel moves (M204 T/S)
    pub fn set_travel_accel(&self, accel: Option<Real>) {
        self.cfg.apply_mut(|m| m.travel_accel_su = accel);
    }

    pub fn get_print_accel(&self) -> Option<Real> {
        self.cfg.apply(|m| m.print_accel_su)
    }

    pub fn get_travel_accel(&self) -> Option<Real> {
        self.cfg.apply(|m| m.travel_accel_su)
    }

    /// Firmware retraction settings (M207/M208)
    pub fn get_retraction(&self) -> RetractionConfig {
        self.cfg.apply(|m| m.retraction)
    }

    pub fn set_retraction(&self, retraction: RetractionConfig) {
        self.cfg.apply_mut(|m| m.retraction = retraction);
    }

    pub fn get_flow_rate_as_real(&self) -> Real {
        Real::new(self.get_flow_rate() as i64, 0) / hwa::math::ONE_HUNDRED
    }

    pub fn get_speed_rate_as_real(&self) -> Real {
        Real::new(self.get_speed_rate() as i64, 0) / hwa::math::ONE_HUNDRED
    }

    pub fn get_default_travel_speed(&self) -> Real {
        self.cfg.apply(|m| m.default_travel_speed)
    }

    /// Largest instantaneous velocity change at a junction (`space_units`/s, `0`: unlimited),
    /// see [Contract::DEFAULT_CORNER_SPEED_CHANGE_PS]
    pub fn set_corner_speed_change(&self, dv: Real) {
        let dv = dv.max(hwa::math::ZERO);
        self.cfg.apply_mut(|m| m.corner_speed_change_su_s = dv);
        hwa::info!(
            "[MotionConfig:UserConf] Corner speed change set to {:?} {}/s",
            dv,
            Contract::SPACE_UNIT_MAGNITUDE
        );
    }

    pub fn get_corner_speed_change(&self) -> Real {
        self.cfg.apply(|m| m.corner_speed_change_su_s)
    }

    pub fn get_min_speed(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.min_speed_su)
    }

    pub fn get_max_speed(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.max_speed_su)
    }

    pub fn get_max_feed_rate(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.max_feed_rate_su)
    }

    pub fn get_max_accel(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.max_accel_su)
    }

    pub fn get_max_jerk(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.max_jerk_su)
    }

    pub fn get_machine_bounds(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.world_size_wu)
    }

    /// Applies the machine defaults from the [Contract] (boot and M502)
    pub fn apply_defaults(&self) {
        self.set_max_speed(Contract::DEFAULT_MAX_SPEED_PS);
        self.set_max_accel(Contract::DEFAULT_MAX_ACCEL_PS);
        self.set_max_jerk(Contract::DEFAULT_MAX_JERK_PS);
        self.set_default_travel_speed(Contract::DEFAULT_TRAVEL_SPEED_PS);
        self.set_corner_speed_change(Contract::DEFAULT_CORNER_SPEED_CHANGE_PS);
        self.set_space_units_per_world_unit(Contract::DEFAULT_UNITS_PER_WU);
        self.set_micro_steps_per_axis(Contract::DEFAULT_MICRO_STEPS_PER_AXIS);
        self.set_world_center(Contract::DEFAULT_WORLD_CENTER_WU);
        self.set_world_size(Contract::DEFAULT_WORLD_SIZE_WU);
        self.set_nozzle_offset(TVector::new_with_const_value(hwa::math::ZERO));
        self.set_probe_offset(Contract::DEFAULT_PROBE_OFFSET_WU);
        self.cfg.apply_mut(|m| {
            m.home_offset_wu = TVector::new_with_coord(CoordSel::motion_relevant_axis(), Some(hwa::math::ZERO))
        });
        self.set_flow_rate(100);
        self.set_speed_rate(100);
        self.set_print_accel(None);
        self.set_travel_accel(None);
        self.set_retraction(RetractionConfig::defaults());
        // Compute min speed. Really useful because of discretion effects
        self.compute_min_speed();
    }

    pub fn compute_min_speed(&self) {
        let one_sample_period = Real::new(
            hwa::Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY.into(),
            0,
        )
        .recip();
        let one_step_period =
            Real::new(hwa::Contract::STEP_PLANNER_CLOCK_FREQUENCY.into(), 0).recip();
        let sample_rate = one_step_period / one_sample_period;
        let dt = TVector::new_with_coord(CoordSel::all_axis(), Some(one_step_period));
        let one_step_space_displacement = TVector::one()
            / self.get_micro_steps_as_vector()
            / self.get_units_per_space_magnitude();
        // v_max is calculated as the velocity doing one step at stepping frequency
        let v_max = if hwa::Contract::CLAMP_MAX_FEED_RATE {
            one_step_space_displacement / dt
        } else {
            TVector::new()
        };

        let v_min = if hwa::Contract::CLAMP_MIN_SPEED {
            (one_step_space_displacement / (dt * hwa::math::HALF)) * sample_rate
        } else {
            TVector::new()
        };

        hwa::info!(
            "[MotionConfig:MCUConf] `SegmentSampling`: {} Hz ({} us period)",
            Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY,
            crate::tasks::task_stepper::STEPPER_PLANNER_MICROSEGMENT_PERIOD_US,
        );
        hwa::info!(
            "[MotionConfig:MCUConf] `MicroSegmentInterpolation`: {} Hz ({} us period)",
            Contract::STEP_PLANNER_CLOCK_FREQUENCY,
            crate::tasks::task_stepper::STEPPER_PLANNER_CLOCK_PERIOD_US,
        );
        hwa::info!(
            "[MotionConfig:ComputedConf] Precision (1 step space width) is space: {:?} {}",
            one_step_space_displacement,
            Contract::SPACE_UNIT_MAGNITUDE
        );
        if hwa::Contract::CLAMP_MIN_SPEED {
            hwa::info!(
                "[MotionConfig:ComputedConf] Min speed (1 step at center of `SegmentSampling`) is space: {:?} {}/s",
                v_min,
                Contract::SPACE_UNIT_MAGNITUDE
            );
        } else {
            hwa::info!(
                "[MotionConfig:ComputedConf] Min speed (1 step at half `SegmentSampling`) is UNSET"
            );
        }
        if hwa::Contract::CLAMP_MAX_FEED_RATE {
            hwa::info!(
                "[MotionConfig:ComputedConf] Max feed rate (1 step per `MicroSegmentInterpolation`) is space: {:?} {}/s",
                v_max,
                Contract::SPACE_UNIT_MAGNITUDE
            );
        } else {
            hwa::info!(
                "[MotionConfig:ComputedConf] Max feed rate (1 step per `MicroSegmentInterpolation`) is UNSET"
            );
        }

        // The step plan executor does at most one step per tick and axis: whatever the planner
        // plans faster is executed at that speed (the motion takes longer than planned)
        let v_clock = one_step_space_displacement / dt;
        let max_speed = self.get_max_speed();
        let mut _clock_limited = false;
        max_speed.foreach_values(|coord, v| {
            if let Some(limit) = v_clock.get_coord(coord) {
                _clock_limited |= limit < *v;
            }
        });
        if _clock_limited {
            hwa::warn!(
                "[MotionConfig:ComputedConf] The step planner clock ({} Hz, one step per tick) limits the speed to {:?} {}/s, below the max speed {:?} {}/s: raise STEP_PLANNER_CLOCK_FREQUENCY",
                Contract::STEP_PLANNER_CLOCK_FREQUENCY,
                v_clock,
                Contract::SPACE_UNIT_MAGNITUDE,
                max_speed,
                Contract::SPACE_UNIT_MAGNITUDE
            );
        }

        self.cfg.apply_mut(|m| {
            m.min_speed_su = v_min;
            m.max_feed_rate_su = v_max;
        })
    }
}

impl Clone for MotionConfig {
    fn clone(&self) -> Self {
        MotionConfig::new(self.cfg.clone())
    }
}

/// Motion parameters.
///
/// This structure holds the motion configuration parameters.
/// It includes maximum acceleration, speed, jerk, travel speed, units per space magnitude,
/// machine bounds, micro-stepping values, flow rate, and speed rate.
///
/// # Fields
///
/// * `min_speed_su` - A `TVector` representing the minimum speed for the motion in `space_units`/s that discrete a single step does
/// * `max_speed_su` - A `TVector` representing the maximum speed for the motion in `space_units`/s that the hardware can provide
/// * `max_accel_su` - A `TVector` representing the maximum acceleration for the motion in `space_units`/s^2
/// * `max_jerk_su` - A `TVector` representing the maximum jerk for the motion in `space_units`/s^3
/// * `default_travel_speed` - The default travel speed in `space_units`/s.
/// * `units_per_space_magnitude` - A `TVector` representing the scale of `space_unit` by `space_magnitude`, not considering micro-stepping.
/// * `micro_steps_per_axis` - A `TVector` representing the micro-stepping scale for each axis.
/// * `world_center_wu` - A `TVector` representing the machine's world center.
/// * `world_size_wu` - A `TVector` representing the machine's world size.
/// * `nozzle_offset_wu` - The machine's nozzle offset in world units respect to homing end-stops or logical 0
/// * `probe_offset_wu` - The machine's nozzle offset in world units respect to homing end-stops or logical 0
/// * `flow_rate` - The flow rate for the motion, represented as a percentage.
/// * `speed_rate` - The speed rate for the motion, represented as a percentage.
pub struct MotionConfigContent {
    /// The minimum speed for the motion in `space_units`/s
    /// Computed as one single step by [Contract::MOTION_PLANNER_MICRO_SEGMENT_FREQUENCY] period
    min_speed_su: TVector<Real>,
    /// The maximum feed rate in `space_units`/s that the hardware can provide
    /// Computed as a constant steping at [Contract::STEP_PLANNER_CLOCK_FREQUENCY] period
    max_feed_rate_su: TVector<Real>,

    /// The maximum speed for the motion in `space_units`/s
    max_speed_su: TVector<Real>,
    /// The maximum acceleration for the motion in `space_units`/s^2
    max_accel_su: TVector<Real>,
    /// The maximum jerk for the motion in `space_units`/s^3
    max_jerk_su: TVector<Real>,

    /// The default travel speed in `space_units`/s.
    default_travel_speed: Real,

    /// Largest instantaneous velocity change at a junction in `space_units`/s (`0`: unlimited)
    corner_speed_change_su_s: Real,

    /// The scale of `space_unit` by `space_magnitude`, not considering micro-stepping
    units_per_space_magnitude: TVector<Real>,
    /// The micro-stepping scale for each axis
    micro_steps_per_axis: TVector<u16>,

    /// The machine's world center
    world_center_wu: TVector<Real>,
    /// the machine's world size
    world_size_wu: TVector<Real>,

    /// The machine's nozzle offset respect to homing end-stops or logical 0
    nozzle_offset_wu: TVector<Real>,
    /// The machine's probe offset respect to homing end-stops or logical 0
    probe_offset_wu: TVector<Real>,
    /// The home offsets (M206): logical position = native position + offset
    home_offset_wu: TVector<Real>,

    /// The flow rate for the motion, represented as a percentage.
    flow_rate: u8,
    /// The speed rate for the motion, represented as a percentage.
    pub speed_rate: u8,

    /// Acceleration limit along the path for extruding moves (M204 P)
    print_accel_su: Option<Real>,
    /// Acceleration limit along the path for travel moves (M204 T)
    travel_accel_su: Option<Real>,

    /// Firmware retraction settings
    retraction: RetractionConfig,
}

/// Firmware retraction settings (G10/G11, M207/M208)
#[derive(Clone, Copy, Debug)]
pub struct RetractionConfig {
    /// Retract length in world units
    pub length: Real,
    /// Retract feed rate in world units/s
    pub feed_rate: Real,
    /// Z lift on retraction in world units
    pub z_hop: Real,
    /// Extra length added on recover in world units
    pub recover_extra: Real,
    /// Recover feed rate in world units/s
    pub recover_feed_rate: Real,
}

impl RetractionConfig {
    pub const fn new() -> Self {
        Self {
            length: hwa::math::ZERO,
            feed_rate: hwa::math::ONE,
            z_hop: hwa::math::ZERO,
            recover_extra: hwa::math::ZERO,
            recover_feed_rate: hwa::math::ONE,
        }
    }

    /// Marlin-like defaults: 3 mm at 45 mm/s, recover at 25 mm/s
    pub fn defaults() -> Self {
        Self {
            length: Real::from_lit(3, 0),
            feed_rate: Real::from_lit(45, 0),
            z_hop: hwa::math::ZERO,
            recover_extra: hwa::math::ZERO,
            recover_feed_rate: Real::from_lit(25, 0),
        }
    }
}

impl MotionConfigContent {
    /// Creates a new `MotionConfig` instance with default values.
    /// See [MotionConfigContent]
    pub const fn new() -> Self {
        Self {
            min_speed_su: TVector::new_with_const_value(hwa::math::ZERO),
            max_feed_rate_su: TVector::new(),

            max_speed_su: TVector::new(),
            max_accel_su: TVector::new(),
            max_jerk_su: TVector::new(),

            default_travel_speed: hwa::math::ONE,
            corner_speed_change_su_s: hwa::math::ZERO,
            units_per_space_magnitude: TVector::new_with_const_value(hwa::math::ONE),
            micro_steps_per_axis: TVector::new_with_const_value(1),
            world_center_wu: TVector::new_with_const_value(hwa::math::ZERO),
            world_size_wu: TVector::new_with_const_value(hwa::math::ONE),

            nozzle_offset_wu: TVector::new_with_const_value(hwa::math::ZERO),
            probe_offset_wu: TVector::new_with_const_value(hwa::math::ZERO),
            home_offset_wu: TVector::new_with_coord(CoordSel::motion_relevant_axis(), Some(hwa::math::ZERO)),

            flow_rate: 100,
            speed_rate: 100,

            print_accel_su: None,
            travel_accel_su: None,

            retraction: RetractionConfig::new(),
        }
    }
}
