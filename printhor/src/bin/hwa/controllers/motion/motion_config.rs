use crate::hwa;
use hwa::math::{CoordSel, Real, TVector};
#[allow(unused)]
use hwa::Contract;
#[allow(unused)]
use hwa::HwiContract;
use hwa::SyncMutexStrategy;

/// Configuration structure for motion parameters.
///
/// This structure holds various configuration parameters for controlling motion in a system.
/// It includes maximum acceleration, speed, jerk, travel speed, units per millimeter,
/// machine bounds, micro-stepping values, flow rate, and speed rate.
///
/// # Fields
///
/// * `max_accel` - A `TVector` representing the maximum acceleration for the motion.
/// * `max_speed` - A `TVector` representing the maximum speed for the motion.
/// * `max_jerk` - A `TVector` representing the maximum jerk for the motion.
/// * `default_travel_speed` - The default travel speed in units per second.
/// * `units_per_mm` - A `TVector` representing units per world-magnitude-unit, not considering micro-stepping.
/// * `machine_bounds` - A `TVector` representing the machine's motion bounds.
/// * `usteps` - An array of micro-stepping values for each axis.
/// * `flow_rate` - The flow rate for the motion, represented as a percentage.
/// * `speed_rate` - The speed rate for the motion, represented as a percentage.
///
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
            "[MotionConfig] Max speed set to {:#?} {}/s",
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
            "[MotionConfig] Max accel set to {:#?} {}/s^2",
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
            "[MotionConfig] Max jerk set to {:#?} {}/s^3",
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
            "[MotionConfig] Space units per {} set to {:#?}",
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
            "[MotionConfig] Default travel speed set to {:#?} {}/s",
            speed,
            Contract::SPACE_UNIT_MAGNITUDE
        );
    }

    pub fn set_micro_steps_per_axis(&self, micro_steps: TVector<u16>) {
        self.cfg.apply_mut(|m| {
            m.micro_steps_per_axis = micro_steps;
        });
        hwa::info!(
            "[MotionConfig] Micro-steps per space axis set to {:#?}",
            micro_steps
        );
    }

    pub fn set_world_center(&self, center: TVector<Real>) {
        self.cfg.apply_mut(|m| {
            m.world_center_wu = center;
        });
        hwa::info!(
            "[MotionConfig] World center (reference zero) set to [{:?}] {}",
            center,
            Contract::WORLD_UNIT_MAGNITUDE,
        );
    }

    pub fn set_world_size(&self, size: TVector<Real>) {
        self.cfg.apply_mut(|m| m.world_size_wu = size);
        hwa::info!(
            "[MotionConfig] World size set to [{:?}] {}",
            size,
            Contract::WORLD_UNIT_MAGNITUDE,
        );
        hwa::info!(
            "[MotionConfig] World bounds updated to: [ [{:?}], [{:?}] ] {}",
            -(size / hwa::math::TWO),
            (size / hwa::math::TWO),
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

    pub fn get_max_speed(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.max_speed_su)
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
}

impl Clone for MotionConfig {
    fn clone(&self) -> Self {
        MotionConfig::new(self.cfg.clone())
    }
}

pub struct MotionConfigContent {
    /// Maximum speed for the motion in space units/s.
    pub max_speed_su: TVector<Real>,
    /// Maximum accel for the motion in space units/s^2
    pub max_accel_su: TVector<Real>,
    /// Maximum jerk for the motion in space units/s^3
    pub max_jerk_su: TVector<Real>,

    /// Default travel speed in world magnitude units per second.
    pub default_travel_speed: Real,

    /// Units per space magnitude NOT considering micro-stepping.
    pub units_per_space_magnitude: TVector<Real>,

    /// Machine's motion bounds.
    pub world_center_wu: TVector<Real>,
    /// Machine's motion bounds.
    pub world_size_wu: TVector<Real>,
    /// Micro-stepping values for each axis.
    pub micro_steps_per_axis: TVector<u16>,
    /// Flow rate for the motion, represented as a percentage.
    pub flow_rate: u8,
    /// Speed rate for the motion, represented as a percentage.
    pub speed_rate: u8,
}

impl MotionConfigContent {
    /// Creates a new `MotionConfig` instance with default values.
    /// See [MotionConfigContent]
    pub const fn new() -> Self {
        Self {
            max_accel_su: TVector::new(),
            max_speed_su: TVector::new(),
            max_jerk_su: TVector::new(),
            units_per_space_magnitude: TVector::new(),
            world_center_wu: TVector::new(),
            world_size_wu: TVector::new(),
            micro_steps_per_axis: TVector::new(),
            default_travel_speed: Real::one(),
            flow_rate: 100,
            speed_rate: 100,
        }
    }
}
