use crate::{hwa, math};
use hwa::SyncMutexStrategy;
use math::{CoordSel, Real, TVector};

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
/// * `units_per_mm` - A `TVector` representing units per millimeter, not considering micro-stepping.
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

    pub fn set_max_speed(&self, speed: TVector<u32>) {
        self.cfg.apply_mut(|m| {
            m.max_speed.assign(CoordSel::all(), &speed);
        });
    }
    pub fn set_max_accel(&self, speed: TVector<u32>) {
        self.cfg.apply_mut(|m| {
            m.max_accel.assign(CoordSel::all(), &speed);
        });
    }
    pub fn set_max_jerk(&self, speed: TVector<u32>) {
        self.cfg.apply_mut(|m| {
            m.max_jerk.assign(CoordSel::all(), &speed);
        });
    }
    pub fn set_flow_rate(&self, rate: u8) {
        self.cfg.apply_mut(|m| {
            m.flow_rate = rate;
        });
    }

    pub fn set_units_per_mm(&self, x_spm: Real, y_spm: Real, z_spm: Real, e_spm: Real) {
        let v = TVector::from_coords(Some(x_spm), Some(y_spm), Some(z_spm), Some(e_spm));
        self.cfg.apply_mut(|m| {
            m.units_per_mm = v;
        });
    }

    pub fn set_speed_rate(&self, rate: u8) {
        self.cfg.apply_mut(|m| {
            m.speed_rate = rate;
        });
    }

    pub fn set_default_travel_speed(&self, speed: u16) {
        self.cfg.apply_mut(|m| {
            m.default_travel_speed = speed;
        });
    }

    pub fn set_micro_steps_per_axis(
        &self,
        x_micro_steps: u8,
        y_micro_steps: u8,
        z_micro_steps: u8,
        e_micro_steps: u8,
    ) {
        let micro_steps: [u16; 4] = [
            x_micro_steps.into(),
            y_micro_steps.into(),
            z_micro_steps.into(),
            e_micro_steps.into(),
        ];
        self.cfg.apply_mut(|m| {
            m.micro_steps_per_axis = micro_steps;
        });
    }

    pub fn set_machine_bounds(&self, x: u16, y: u16, z: u16) {
        let bounds = TVector::from_coords(
            Some(Real::new(x.into(), 0)),
            Some(Real::new(y.into(), 0)),
            Some(Real::new(z.into(), 0)),
            None,
        );
        self.cfg.apply_mut(|m| {
            m.machine_bounds = bounds;
        })
    }

    pub fn get_units_per_mm(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.units_per_mm)
    }

    pub fn get_micro_steps(&self) -> [u16; 4] {
        self.cfg.apply(|m| m.micro_steps_per_axis)
    }

    pub fn get_micro_steps_as_vector(&self) -> TVector<Real> {
        let micro_steps = self.get_micro_steps();
        TVector::from_coords(
            Some(Real::from_lit(micro_steps[0].into(), 0)),
            Some(Real::from_lit(micro_steps[1].into(), 0)),
            Some(Real::from_lit(micro_steps[2].into(), 0)),
            Some(Real::from_lit(micro_steps[3].into(), 0)),
        )
    }

    pub fn get_steps_per_unit_as_vector(&self) -> TVector<Real> {
        self.get_units_per_mm() * self.get_micro_steps_as_vector()
    }

    pub fn get_flow_rate(&self) -> u8 {
        self.cfg.apply(|m| m.flow_rate)
    }

    pub fn get_speed_rate(&self) -> u8 {
        self.cfg.apply(|m| m.speed_rate)
    }

    pub fn get_flow_rate_as_real(&self) -> Real {
        Real::new(self.get_flow_rate() as i64, 0) / math::ONE_HUNDRED
    }

    pub fn get_speed_rate_as_real(&self) -> Real {
        Real::new(self.get_speed_rate() as i64, 0) / math::ONE_HUNDRED
    }

    pub fn get_default_travel_speed(&self) -> u16 {
        self.cfg.apply(|m| m.default_travel_speed)
    }

    pub fn get_default_travel_speed_as_real(&self) -> Real {
        Real::new(self.get_default_travel_speed() as i64, 0)
    }

    pub fn get_max_speed(&self) -> TVector<u32> {
        self.cfg.apply(|m| m.max_speed)
    }

    pub fn get_max_speed_as_vector_real(&self) -> TVector<Real> {
        Self::to_real(self.get_max_speed())
    }

    pub fn get_max_accel(&self) -> TVector<u32> {
        self.cfg.apply(|m| m.max_accel)
    }

    pub fn get_max_accel_as_vector_real(&self) -> TVector<Real> {
        Self::to_real(self.get_max_accel())
    }

    pub fn get_max_jerk(&self) -> TVector<u32> {
        self.cfg.apply(|m| m.max_jerk)
    }

    pub fn get_max_jerk_as_vector_real(&self) -> TVector<Real> {
        Self::to_real(self.get_max_jerk())
    }

    pub fn get_machine_bounds(&self) -> TVector<Real> {
        self.cfg.apply(|m| m.machine_bounds)
    }

    fn to_real(src: TVector<u32>) -> TVector<Real> {
        src.map_coords(|c| Some(Real::from_lit(c as i64, 0)))
    }
}

impl Clone for MotionConfig {
    fn clone(&self) -> Self {
        MotionConfig::new(self.cfg.clone())
    }
}

pub struct MotionConfigContent {
    /// Maximum acceleration for the motion.
    pub max_accel: TVector<u32>,
    /// Maximum speed for the motion.
    pub max_speed: TVector<u32>,
    /// Maximum jerk for the motion.
    pub max_jerk: TVector<u32>,

    /// Default travel speed in units per second.
    pub default_travel_speed: u16,

    /// Units per millimeters NOT considering micro-stepping.
    pub units_per_mm: TVector<Real>,

    /// Machine's motion bounds.
    pub machine_bounds: TVector<Real>,
    /// Micro-stepping values for each axis.
    pub micro_steps_per_axis: [u16; 4],
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
            max_accel: TVector::new(),
            max_speed: TVector::new(),
            max_jerk: TVector::new(),
            units_per_mm: TVector::new(),
            machine_bounds: TVector::new(),
            micro_steps_per_axis: [0; 4],
            default_travel_speed: 1,
            flow_rate: 100,
            speed_rate: 100,
        }
    }

    /// Converts the micro-stepping values to a `TVector` of `Real` numbers.
    ///
    /// # Returns
    /// A `TVector` containing the micro-stepping values as `Real` numbers.
    pub fn get_usteps_as_vector(&self) -> TVector<Real> {
        TVector::from_coords(
            Some(Real::new(self.micro_steps_per_axis[0].into(), 0)),
            Some(Real::new(self.micro_steps_per_axis[1].into(), 0)),
            Some(Real::new(self.micro_steps_per_axis[2].into(), 0)),
            Some(Real::new(self.micro_steps_per_axis[3].into(), 0)),
        )
    }
}
