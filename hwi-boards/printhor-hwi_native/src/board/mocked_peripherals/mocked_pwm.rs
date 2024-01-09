use super::mocked_pin::MockedIOPin;
use super::mocked_pin::PinStateRef;
use embedded_hal_02::Pwm;

pub type PwmChannel = u8;

pub struct MockedPwm {
    #[allow(unused)]
    p: MockedIOPin,
}
impl MockedPwm {
    #[allow(unused)]
    pub(crate) const fn new(id: u8, pin_state: PinStateRef) -> Self {
        Self {
            p: MockedIOPin::new(id, pin_state),
        }
    }
}

impl Pwm for MockedPwm {
    fn set_period<P>(&mut self, _: P) where P: Into<u16> {  }
    fn set_duty(&mut self, _: <Self as Pwm>::Channel, _: <Self as Pwm>::Duty) {  }
    fn get_max_duty(&self) -> <Self as Pwm>::Duty { 0u16  }
    fn get_duty(&self, _: <Self as Pwm>::Channel) -> <Self as Pwm>::Duty { 0u16  }
    fn get_period(&self) -> <Self as Pwm>::Time { 0u16 }
    fn enable(&mut self, _: <Self as Pwm>::Channel) { }
    fn disable(&mut self, _: <Self as Pwm>::Channel) {  }
    type Duty = u16;
    type Time = u16;
    type Channel = u8;
}