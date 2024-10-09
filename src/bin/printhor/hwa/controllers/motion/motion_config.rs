use crate::math::Real;
use crate::tgeo::TVector;

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
/// type MotionConfigMutexType = hwa::NoopMutex;
///
/// let motion_config = hwa::make_static_controller!(
///     "MotionConfig",
///     MotionConfigMutexType,
///     hwa::controllers::MotionConfig,
///     hwa::controllers::MotionConfig::new()
///  );
/// ```
pub struct MotionConfig {
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

impl MotionConfig {
    /// Creates a new `MotionConfig` instance with default values.
    /// See [MotionConfig]
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
