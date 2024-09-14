use crate::math::Real;
use crate::tgeo::TVector;

/// Type alias for the motion configuration's mutex type.
pub type MotionConfigMutexType = printhor_hwa_common::InterruptControllerMutexType;

/// Type alias for a reference to the motion configuration.
pub type MotionConfigRef = printhor_hwa_common::ControllerRef<MotionConfigMutexType, MotionConfig>;

/// Configuration structure for motion parameters.
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
    pub usteps: [u16; 4],
    /// Flow rate for the motion, represented as a percentage.
    pub flow_rate: u8,
    /// Speed rate for the motion, represented as a percentage.
    pub speed_rate: u8,
}

impl MotionConfig {
    /// Creates a new `MotionConfig` instance with default values.
    pub const fn new() -> Self {
        Self {
            max_accel: TVector::new(),
            max_speed: TVector::new(),
            max_jerk: TVector::new(),
            units_per_mm: TVector::new(),
            machine_bounds: TVector::new(),
            usteps: [0; 4],
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
            Some(Real::new(self.usteps[0].into(), 0)),
            Some(Real::new(self.usteps[1].into(), 0)),
            Some(Real::new(self.usteps[2].into(), 0)),
            Some(Real::new(self.usteps[3].into(), 0)),
        )
    }
}
