use crate::math::Real;
use crate::tgeo::TVector;

pub type MotionConfigMutexType = printhor_hwa_common::InterruptControllerMutexType;

pub type MotionConfigRef = printhor_hwa_common::ControllerRef<MotionConfigMutexType, MotionConfig>;

pub struct MotionConfig {
    pub max_accel: TVector<u32>,
    pub max_speed: TVector<u32>,
    pub max_jerk: TVector<u32>,

    pub default_travel_speed: u16,

    // Units per millimeters NOT considering micro-stepping
    pub units_per_mm: TVector<Real>,

    pub machine_bounds: TVector<Real>,
    // Micro-stepping
    pub usteps: [u16; 4],
    pub flow_rate: u8,
    pub speed_rate: u8,
}

impl MotionConfig {
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

    pub fn get_usteps_as_vector(&self) -> TVector<Real> {
        TVector::from_coords(
            Some(Real::new(self.usteps[0].into(), 0)),
            Some(Real::new(self.usteps[1].into(), 0)),
            Some(Real::new(self.usteps[2].into(), 0)),
            Some(Real::new(self.usteps[3].into(), 0)),
        )
    }
}
