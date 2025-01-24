use printhor_hwa_common as hwa;
use embedded_hal_1::i2c::{ErrorKind, ErrorType, Operation};
use pwm_pca9685;
use printhor_hwa_common::math::{CoordSel, Real};

pub struct I2CDevice();

pub struct MockedI2c {
    pwm: pwm_pca9685::Pca9685<I2CDevice>,
    state: [pwm_pca9685::ChannelOnOffControl; 16],
}

impl MockedI2c {
    pub async fn new() -> Self {

        let dev = I2CDevice();

        let address = pwm_pca9685::Address::from((false, false, false, false, false, false));
        let mut pwm = pwm_pca9685::Pca9685::new(dev, address).unwrap();

        pwm.set_prescale(100).unwrap();
        pwm.enable().unwrap();
        hwa::debug!("[mocked_i2c] setting...");

        // period = 20 ms = 4095 count
        // dmin(-90) = 1ms = 204.75 count
        // dmax(+90) = 2ms = 409.50 count
        // dmed(0) = 1.5ms = (204.75 + 409.50)/2 = 307.125 count
        // count_per_deg = (409.50 - 204.75) / 180 = 1.13750000

        // count(angle) = round(307.125 + (1.13750000 * angle))

        let mut instance = Self {
            pwm,
            state: [pwm_pca9685::ChannelOnOffControl{
                on: 0,
                off: 0,
                full_on: false,
                full_off: false,
            }; 16],
        };

        for _servo in CoordSel::all_axis().iter() {
            instance.set_angle(_servo, &hwa::math::ZERO);
        }
        instance.apply().await;
        hwa::debug!("[mocked_i2c] setting done");
        instance
    }

    pub fn set_angle(&mut self, axis: hwa::math::CoordSel, angle: &hwa::math::Real) -> bool {
        let pwm = (
            (Real::from_f32(307.125f32) + (Real::from_f32(1.1375f32) * (*angle))).round().to_i32().unwrap()
        ).max(205).min(409) as u16;
        let index = axis.index();
        if index < 16 {
            if self.state[index].off != pwm {
                hwa::debug!("[task_motion_broadcast] set PWM [{:?}] [{:?}] = {:?} -> {:?}", angle, index, self.state[index].off, pwm );
                self.state[index].off = pwm;
                true
            }
            else {
                false
            }
        }
        else {
            false
        }
    }

    pub async fn apply(&mut self) -> () {
        self.pwm.set_all_channels(&self.state).unwrap();
    }
}

#[derive(Debug)]
pub enum I2CError {

}

impl embedded_hal_1::i2c::Error for I2CError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for I2CDevice { type Error = I2CError; }

impl<A> embedded_hal_1::i2c::I2c<A> for I2CDevice
where A: embedded_hal_1::i2c::AddressMode + core::fmt::Debug
{
    fn transaction(&mut self, _address: A, _operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        hwa::debug!("Wrote something at {:?}", _address);
        Ok(())
    }
}
