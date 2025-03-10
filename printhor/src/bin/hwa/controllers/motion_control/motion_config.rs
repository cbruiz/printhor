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

    pub fn set_probe_offset(&self, mut offset: TVector<Real>) {
        offset.set_coord(CoordSel::motion_relevant_axis().complement(), None);
        self.cfg.apply_mut(|m| m.probe_offset_wu = offset);
        hwa::info!(
            "[MotionConfig:MachineConf] Probe offset updated to world: [{:?}] {}",
            offset,
            Contract::WORLD_UNIT_MAGNITUDE,
        );
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

    fn get_flow_rate(&self) -> u8 {
        self.cfg.apply(|m| m.flow_rate)
    }

    fn get_speed_rate(&self) -> u8 {
        self.cfg.apply(|m| m.speed_rate)
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
                "[MotionConfig:ComputedConf] Min speed (1 step at half `SegmentSampling`) is space: {:?} {}/s",
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

    /// The flow rate for the motion, represented as a percentage.
    flow_rate: u8,
    /// The speed rate for the motion, represented as a percentage.
    pub speed_rate: u8,
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
            units_per_space_magnitude: TVector::new_with_const_value(hwa::math::ONE),
            micro_steps_per_axis: TVector::new_with_const_value(1),
            world_center_wu: TVector::new_with_const_value(hwa::math::ZERO),
            world_size_wu: TVector::new_with_const_value(hwa::math::ONE),

            nozzle_offset_wu: TVector::new_with_const_value(hwa::math::ZERO),
            probe_offset_wu: TVector::new_with_const_value(hwa::math::ZERO),

            flow_rate: 100,
            speed_rate: 100,
        }
    }
}
